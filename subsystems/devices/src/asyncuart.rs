
use spin::Once;
use async_uart_driver::serials::AsyncSerial;
use alloc::sync::Arc;
use drivers::uart;
use fdt::Fdt;
use interrupt::register_device_to_plic;
use log::info;
use platform::println;
use heapless::spsc::Queue;

use crate::prob::Probe;

use device_interface::DeviceBase;

pub const DEFAULT_TX_BUFFER_SIZE: usize = 5256;
pub const DEFAULT_RX_BUFFER_SIZE: usize = 5256;


pub struct AsyncSerialDevice {
    inner: Arc<AsyncSerial>,
}

unsafe impl Send for AsyncSerialDevice{ }
unsafe impl Sync for AsyncSerialDevice{ }

impl DeviceBase for AsyncSerialDevice {
    fn hand_irq(&self) {
        self.inner.interrupt_handler();
    }
}

impl AsyncSerialDevice {
    pub fn new(inner: Arc<AsyncSerial>) -> Self {
        Self {
            inner,
        }
    }

    pub fn get_inner(&self) -> Arc<AsyncSerial> {
        self.inner.clone()
    }
}

pub static ASYNC_SERIAL_DEVICE: Once<Arc<AsyncSerialDevice>> = Once::new();
const BAUD_RATE: usize = 6_250_000;

const LOOP_TIME: usize = 2;    
const HALF_FIFO_DEPTH: usize = 20;

pub fn init_async_serial(async_serial: Arc<AsyncSerialDevice>) {
    ASYNC_SERIAL_DEVICE.call_once(|| async_serial);
}



pub async fn init_async_uart(){
    println!("[devices/src] init async_uart");

    let dtb_ptr = platform::platform_dtb_ptr();

    let dtb = unsafe { Fdt::from_ptr(dtb_ptr as *const u8).unwrap() };
    match dtb.probe_serial() {
        Some(serial) => {
            let (base_addr, irq) = (serial.base_addr, serial.irq);
            println!("Init async serial, base_addr:{:#x},irq:{}", base_addr, irq);
            type RxBuffer = Queue<u8, DEFAULT_RX_BUFFER_SIZE>;
            type TxBuffer = Queue<u8, DEFAULT_TX_BUFFER_SIZE>;
            static mut DRIVER_RX_BUFFER: RxBuffer = RxBuffer::new();
            static mut DRIVER_TX_BUFFER: TxBuffer = TxBuffer::new();
            let (rx_pro, rx_con) = unsafe { DRIVER_RX_BUFFER.split() };
            let (tx_pro, tx_con) = unsafe { DRIVER_TX_BUFFER.split() };
            let async_serial = AsyncSerial::new(
                base_addr, 
                rx_pro, 
                rx_con, 
                tx_pro, 
                tx_con
            );
            let async_serial = Arc::new(async_serial);
            println!("before hardware_init");
            async_serial.hardware_init(BAUD_RATE);
            let async_serial_device = Arc::new(
                AsyncSerialDevice::new(async_serial)
            );
            init_async_serial(async_serial_device.clone());
            register_device_to_plic(irq, async_serial_device.clone());

            static WRITE_BUF: [u8;12] = [65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76];
            static mut READ_BUF:[u8;12] =  [0; 12];
            for _ in 0..LOOP_TIME {
                println!("before write");
                async_serial_device.get_inner().write(&WRITE_BUF as &[u8]).await;
                async_serial_device.get_inner().read( unsafe {
                    &mut READ_BUF
                }).await;
                println!("{:?}", unsafe {& mut READ_BUF} );
            }
        },
        None => {
            println!("There is no serial device");
        },
    }
}
