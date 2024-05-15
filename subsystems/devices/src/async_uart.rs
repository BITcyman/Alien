use fdt::Fdt;
use platform::println;
use interrupt::register_device_to_plic;
use drivers::uart::{LowUartDriver, Uart, Uart16550, Uart8250};
use device_interface::UartDevice;

use alloc::boxed::Box;
use alloc::sync::Arc;




pub fn basic_uart_test(){

    // let dtb_ptr = platform::platform_dtb_ptr();
    // let dtb = unsafe { Fdt::from_ptr(dtb_ptr as *const u8).unwrap() };
    // for node in dtb.all_nodes() {
    //     println!("    {}", node.name);
    // }

    let (base_addr, irq) = (0x12000000, 0x2d);
    println!("Init addtional uart, base_addr:{:#x},irq:{}", base_addr, irq);
    
    let mut uart = Uart8250::new(base_addr);
    let uart = Arc::new(Uart::new(Box::new(uart)));
    crate::uart::init_uart2(uart.clone());
    register_device_to_plic(irq, uart.clone());

    println!("Register new uart to plic successfullt");
    
    println!("{:?}",uart.get());

    println!("Basic Uart Test finished");

}