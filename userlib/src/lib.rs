#![feature(panic_info_message)]
#![no_std]
#![feature(linkage)]
#![allow(unused)]

extern crate alloc;
use crate::heap::init_heap;
use crate::process::exit;
use crate::syscall::sys_shutdown;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec::Vec;

pub mod common;
pub mod dbfs;
pub mod fs;
mod heap;
pub mod io;
mod macros;
mod panic;
pub mod process;
mod sys;
mod syscall;
pub mod thread;
pub mod time;

#[no_mangle]
fn _start(argc: usize, argv: usize) -> ! {
    init_heap();
    let argv = parse_args(argc, argv);
    exit(unsafe { main(argc, argv) });
}

fn parse_args(argc: usize, argv: usize) -> Vec<String> {
    let mut args = Vec::new();
    for i in 0..argc {
        let arg = unsafe { *(argv as *const *const u8).add(i) };
        let len = unsafe { common::strlen(arg) };
        let arg = unsafe {
            let slice = core::slice::from_raw_parts(arg, len);
            core::str::from_utf8_unchecked(slice)
        };
        args.push(arg.to_string());
    }
    args
}

#[linkage = "weak"]
#[no_mangle]
fn main(argc: usize, argv: Vec<String>) -> i32 {
    panic!("Cannot find main!");
}

pub fn shutdown() -> ! {
    sys_shutdown();
    panic!("Shutdown failed!");
}