
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


const BAUD_RATE: usize = 6_250_000;
const HALF_FIFO_DEPTH: usize = 20;

pub static ASYNC_SERIAL_DEVICE: Once<Arc<AsyncSerialDevice>> = Once::new();
pub fn init_async_serial(async_serial: Arc<AsyncSerialDevice>) {
    ASYNC_SERIAL_DEVICE.call_once(|| async_serial);
}

pub static ASYNC_SERIAL_DEVICE3: Once<Arc<AsyncSerialDevice>> = Once::new();
pub fn init_async_serial3(async_serial: Arc<AsyncSerialDevice>) {
    ASYNC_SERIAL_DEVICE3.call_once(|| async_serial);
}

pub static ASYNC_SERIAL_DEVICE4: Once<Arc<AsyncSerialDevice>> = Once::new();
pub fn init_async_serial4(async_serial: Arc<AsyncSerialDevice>) {
    ASYNC_SERIAL_DEVICE4.call_once(|| async_serial);
}


pub async fn init_async_uart(){
    println!("[devices/src] init async_uart");

    const LOOP_TIME: usize = 1;    
    let dtb_ptr = platform::platform_dtb_ptr();

    let dtb = unsafe { Fdt::from_ptr(dtb_ptr as *const u8).unwrap() };
    match dtb.probe_serial() {
        Some(serial) => {
            // let (base_addr, irq) = (serial.base_addr, serial.irq);
            let (base_addr, irq) = (0x10003000, 13);
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

pub async fn auto_driver_test(){
    println!("[devices/src] auto_driver_test");

    const LOOP_TIME: usize = 5;    

    let (base_addr3, irq3) = (0x10004000, 14);
    let (base_addr4, irq4) = (0x10005000, 15);
    // println!("Init async serial, base_addr:{:#x},irq:{}", base_addr, irq);

    type RxBuffer = Queue<u8, DEFAULT_RX_BUFFER_SIZE>;
    type TxBuffer = Queue<u8, DEFAULT_TX_BUFFER_SIZE>;
    static mut DRIVER_RX_BUFFER3: RxBuffer = RxBuffer::new();
    static mut DRIVER_RX_BUFFER4: RxBuffer = RxBuffer::new();
    static mut DRIVER_TX_BUFFER3: TxBuffer = TxBuffer::new();
    static mut DRIVER_TX_BUFFER4: TxBuffer = TxBuffer::new();

    let (rx_pro3, rx_con3) = unsafe { DRIVER_RX_BUFFER3.split() };
    let (tx_pro3, tx_con3) = unsafe { DRIVER_TX_BUFFER3.split() };

    let (rx_pro4, rx_con4) = unsafe { DRIVER_RX_BUFFER4.split() };
    let (tx_pro4, tx_con4) = unsafe { DRIVER_TX_BUFFER4.split() };

    let async_serial3 = AsyncSerial::new(
        base_addr3, 
        rx_pro3, 
        rx_con3, 
        tx_pro3, 
        tx_con3
    );

    let async_serial4 = AsyncSerial::new(
        base_addr4, 
        rx_pro4, 
        rx_con4, 
        tx_pro4, 
        tx_con4
    );

    let async_serial3 = Arc::new(async_serial3);
    let async_serial4 = Arc::new(async_serial4);

    println!("before hardware_init");

    async_serial3.hardware_init(BAUD_RATE);
    async_serial4.hardware_init(BAUD_RATE);

    let async_serial_device3 = Arc::new(
        AsyncSerialDevice::new(async_serial3)
    );
    let async_serial_device4 = Arc::new(
        AsyncSerialDevice::new(async_serial4)
    );

    init_async_serial3(async_serial_device3.clone());
    init_async_serial4(async_serial_device4.clone());

    register_device_to_plic(irq3, async_serial_device3.clone());
    register_device_to_plic(irq4, async_serial_device4.clone());

    static WRITE_BUF3: [u8;12] = [65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76];
    static mut READ_BUF3: [[u8;12]; LOOP_TIME] =  [[0; 12]; LOOP_TIME];
    static WRITE_BUF4: [u8;12] = [97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108];
    static mut READ_BUF4: [[u8;12]; LOOP_TIME] =  [[0; 12]; LOOP_TIME];

    for i in 0..LOOP_TIME {
        println!("before write");
        async_serial_device3.get_inner().write(&WRITE_BUF3 as &[u8]).await;
        async_serial_device3.get_inner().read( unsafe {
            &mut READ_BUF3[i]
        }).await;

        async_serial_device4.get_inner().write(&WRITE_BUF4 as &[u8]).await;
        async_serial_device4.get_inner().read( unsafe {
            &mut READ_BUF4[i]
        }).await;

        // async_serial_device3.get_inner().run_exec();
        // async_serial_device4.get_inner().run_exec();
        
        // async_serial_device3.get_inner();

        // println!("{:?}", unsafe {& mut READ_BUF} );
    }
}
