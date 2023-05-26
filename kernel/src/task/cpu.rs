use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use bitflags::bitflags;
use lazy_static::lazy_static;

use syscall_table::syscall_func;

use crate::arch;
use crate::config::CPU_NUM;
use crate::fs::vfs;
use crate::sbi::shutdown;
use crate::sync::IntrLock;
use crate::task::context::Context;
use crate::task::INIT_PROCESS;
use crate::task::process::{Process, ProcessState};
use crate::task::schedule::schedule;
use crate::trap::TrapFrame;

#[derive(Debug)]
pub struct CPU {
    pub process: Option<Arc<Process>>,
    pub context: Context,
    pub intr_lock: IntrLock<usize>,
}

impl CPU {
    const fn empty() -> Self {
        Self {
            process: None,
            context: Context::empty(),
            intr_lock: IntrLock::new(0),
        }
    }
    pub fn take_process(&mut self) -> Option<Arc<Process>> {
        self.process.take()
    }
    pub fn get_context_raw_ptr(&self) -> *const Context {
        &self.context as *const Context
    }
    pub fn get_context_mut_raw_ptr(&mut self) -> *mut Context {
        &mut self.context as *mut Context
    }
}

/// save info for each cpu
static mut CPU_MANAGER: [CPU; CPU_NUM] = [CPU::empty(); CPU_NUM];

/// the global process pool
type ProcessPool = VecDeque<Arc<Process>>;
lazy_static! {
    pub static ref PROCESS_MANAGER: IntrLock<ProcessPool> = IntrLock::new(ProcessPool::new());
}

/// get the current cpu info
pub fn current_cpu() -> &'static mut CPU {
    let hart_id = arch::hart_id();
    unsafe { &mut CPU_MANAGER[hart_id] }
}

/// get the current_process
pub fn current_process() -> Option<&'static Arc<Process>> {
    let cpu = current_cpu();
    cpu.process.as_ref()
}

/// get the current process's token (root ppn)
pub fn current_user_token() -> usize {
    let process = current_process().unwrap();
    process.token()
}

/// get the current process's trap frame
pub fn current_trap_frame() -> &'static mut TrapFrame {
    let process = current_process().unwrap();
    process.trap_frame()
}

#[syscall_func(93)]
pub fn do_exit(exit_code: i32) -> isize {
    let c_process = current_process().unwrap();
    let exit_code = (exit_code & 0xff) << 8;
    if c_process.get_pid() == 0 {
        println!("init process exit with code {}", exit_code);
        shutdown();
    }
    {
        let init = INIT_PROCESS.clone();
        c_process.children().iter().for_each(|child| {
            child.update_parent(init.clone());
            init.insert_child(child.clone());
        });
    }
    c_process.update_state(ProcessState::Zombie);
    c_process.update_exit_code(exit_code);
    c_process.recycle();
    schedule();
    0
}

#[syscall_func(124)]
pub fn do_suspend() -> isize {
    let process = current_process().unwrap();
    process.update_state(ProcessState::Ready);
    schedule();
    0
}

#[syscall_func(172)]
pub fn get_pid() -> isize {
    let process = current_process().unwrap();
    process.get_pid()
}

#[syscall_func(173)]
pub fn get_ppid() -> isize {
    let process = current_process().unwrap();
    let parent = process.access_inner().parent.clone();
    if parent.is_none() {
        return 0;
    } else {
        parent.unwrap().upgrade().unwrap().get_pid()
    }
}

#[syscall_func(220)]
pub fn do_fork() -> isize {
    let process = current_process().unwrap();
    let new_process = process.fork();
    if new_process.is_none() {
        return -1;
    }
    let new_process = new_process.unwrap();
    let mut process_pool = PROCESS_MANAGER.lock();
    // update return value
    let trap_frame = new_process.trap_frame();
    trap_frame.update_res(0);
    let pid = new_process.get_pid();
    process_pool.push_back(new_process);
    pid
}

#[syscall_func(221)]
pub fn do_exec(path: *const u8, args_ptr: *const usize, _env: *const usize) -> isize {
    let process = current_process().unwrap();
    let str = process.transfer_str(path);
    let mut data = Vec::new();
    // get the args and push them into the new process stack
    let mut args = Vec::new();
    let mut start = args_ptr as *mut usize;
    loop {
        let arg = process.transfer_raw_ptr(start);
        if *arg == 0 {
            break;
        }
        args.push(*arg);
        start = unsafe { start.add(1) };
    }
    let args = args
        .into_iter()
        .map(|arg| {
            let mut arg = process.transfer_str(arg as *const u8);
            arg.push('\0');
            arg
        })
        .collect::<Vec<String>>();
    if vfs::read_all(&str, &mut data) {
        let argc = args.len();
        let res = process.exec(data.as_slice(), args);
        if res.is_err() {
            return res.err().unwrap() as isize;
        }
        return argc as isize;
    } else {
        -1
    }
}

/// Please care about the exit code,it may be null
#[syscall_func(260)]
pub fn wait_pid(pid: isize, exit_code: *mut i32, options: u32, _rusage: *const u8) -> isize {
    let process = current_process().unwrap().clone();
    loop {
        if process
            .children()
            .iter()
            .find(|child| child.get_pid() == pid || pid == -1)
            .is_none()
        {
            return -1;
        }
        let children = process.children();
        let res = children.iter().enumerate().find(|(_, child)| {
            child.state() == ProcessState::Zombie && (child.get_pid() == pid || pid == -1)
        });
        let res = res.map(|(index, _)| index);
        drop(children);
        if let Some(index) = res {
            let child = process.remove_child(index);
            assert_eq!(Arc::strong_count(&child), 1);
            if !exit_code.is_null() {
                let exit_code_ref = process.transfer_raw_ptr(exit_code);
                *exit_code_ref = child.exit_code();
            }
            return child.get_pid() as isize;
        } else {
            let wait_options = WaitOptions::from_bits(options).unwrap();
            if wait_options.contains(WaitOptions::WNOHANG) {
                return 0;
            } else {
                do_suspend();
            }
        }
    }
}

#[syscall_func(214)]
pub fn do_brk(addr: usize) -> isize {
    let process = current_process().unwrap();
    let mut inner = process.access_inner();
    let heap_info = inner.heap_info();
    if addr == 0 {
        return heap_info.end as isize;
    }
    if addr < heap_info.start {
        return -1;
    }
    if addr > heap_info.end {
        let additional = addr - heap_info.end;
        let res = inner.extend_heap(additional);
        if res.is_err() {
            return -1;
        }
    }
    addr as isize
}

bitflags! {
    pub struct WaitOptions:u32 {
        const WNOHANG = 1;
        const WUNTRACED = 2;
        const WCONTINUED = 8;
    }
}
