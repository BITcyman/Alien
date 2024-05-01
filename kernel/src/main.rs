#![feature(atomic_from_mut)]
#![feature(ip_in_core)]
#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

#[macro_use]
extern crate log;
#[macro_use]
extern crate syscall_table;
#[macro_use]
extern crate platform;
extern crate alloc;
extern crate unwinder;
use alloc::boxed::Box;
pub use syscall_table::*;
mod fs;
mod gui;
mod ipc;
mod mm;
mod net;
mod system;
mod task;
mod time;
mod trap;

use crate::task::DriverTaskImpl;
use core::hint::spin_loop;
use core::sync::atomic::{AtomicBool, Ordering};
use platform::platform_machine_info;

/// 多核启动标志
static STARTED: AtomicBool = AtomicBool::new(false);

#[no_mangle]
fn main(hart_id: usize) {
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(kernel_init(spawner, hart_id)).unwrap();
    });
}

use embassy_executor::Executor;
use embassy_executor::Spawner;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn kernel_init(spawner: Spawner, hart_id: usize){
    if STARTED
        .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
        .is_ok()
    {
        println!("Boot hart {}", hart_id);
        let machine_info = platform_machine_info();
        println!("{:#?}", machine_info);
        mem::init_memory_system(machine_info.memory.end, true);
        interrupt::init_plic(machine_info.plic.start);
        shim::register_task_func(Box::new(DriverTaskImpl));
        devices::init_device();
        vfs::init_filesystem().expect("init filesystem failed");
        trap::init_trap_subsystem();
        arch::allow_access_user_memory();
        task::init_task().await;
        // register all syscall
        syscall_table::init_init_array!();
        STARTED.store(false, Ordering::Relaxed);
    } else {
        while STARTED.load(Ordering::Relaxed) {
            spin_loop();
        }
        mem::init_memory_system(0, false);
        arch::allow_access_user_memory();
        trap::init_trap_subsystem();
        println!("hart {} start", arch::hart_id());
    }
    time::set_next_trigger();
    test().await;
    println!("Begin run task...");
    task::schedule::run_task();
}


async fn test(){
    let f1 = async {
        println!("========= async test f1 ==============");
    };
    let f2 = async {
        println!("========= async test f2 ==============");
    };
    let f3 = async {
        println!("========= async test f3 ==============");
    };
    f3.await;
    f2.await;
    f1.await;
}
