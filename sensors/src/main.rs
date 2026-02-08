#![allow(warnings)]
#![no_std]
#![no_main]

mod gps;
mod gyro;
mod uart_subscriber;

use gps::read_gps_line;
use gyro::*;

use crate::uart_subscriber::*;
use core::fmt::Write;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
//use defmt::*;
//use defmt_rtt as _; // global logger
//use log::info;
//use panic_halt as _;
use stm32_usbd::UsbBus;
use stm32f3xx_hal::gpio::gpioa::{PA2, PA3};
use stm32f3xx_hal::gpio::{Alternate, PushPull, AF7};
use stm32f3xx_hal::{
    delay::Delay,
    i2c::I2c,
    pac,
    prelude::*,
    serial::{config::Config, Serial},
    time::rate::Baud,
    timer::Timer,
};

macro_rules! log_info {
    ($tx:expr, $($arg:tt)*) => {
        let _ = writeln!($tx, "[INFO] {}", format_args!($($arg)*));
    };
}
/*
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    // Clock setup (leave defaults, they are fine)
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    /*
        User LEDs → GPIO mapping (STM32F334 Discovery)
        LED	Color	GPIO pin
        LD3	Red	    PB6
        LD4	Orange	PB8
        LD5	Green	PB9
        LD6	Blue	PB7

    */
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut led = gpiob
        .pb7
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut afrl = &mut gpiob.afrl;

    /*
    SB14, 16 (VCP RX, TX) (2)
        - OFF PA2, PA3 of STM32F103CBT6 are not connected to PB4, PB3 of STM32F334C8T6
        - ON PA2, PA3 of STM32F103CBT6 are connected to PB4, PB3 of STM32F334C8T6,
          then SW0 cannot be used and SB17 must be OFF
    */

    let tx_pin = gpiob
        .pb3
        .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, afrl);
    let rx_pin = gpiob
        .pb4
        .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, afrl);

    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(Baud::new(9600)),
        clocks,
        &mut rcc.apb1,
    );
    rtt_init_print!();
    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    timer.start(1.seconds());
    rprintln!("Hello from STM32F338!");
    let mut counter = 0;
    loop {
        //rprintln!("setting high counter={}", counter);
        //led.set_high().unwrap(); // turn LED **on** (3.3V)
        //nb::block!(timer.wait()).unwrap();
        //rprintln!("setting low counter={}", counter);
        //led.set_low().unwrap(); // turn LED off
        //nb::block!(timer.wait()).unwrap();
        rprintln!("toggle counter={}", counter);
        led.toggle().unwrap();
        // Wait until timer expires
        nb::block!(timer.wait()).unwrap();
        //rprintln!("waited counter={}", counter);
        cortex_m::asm::nop();
        counter += 1;
    }
}
*/
/*
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    // Clock setup (leave defaults, they are fine)
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut afrl = &mut gpioa.afrl;

    let tx_pin = gpioa
        .pa2
        .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, afrl);
    let rx_pin = gpioa
        .pa3
        .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, afrl);

    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(Baud::new(9600)),
        clocks,
        &mut rcc.apb1,
    );
     //5 Hz update rate
    let cmd = b"$PMTK220,200*2C\r\n"; // 200ms interval → 5 Hz
    for &b in cmd.iter() {
        nb::block!(serial.write(b)).unwrap();
    }
    rtt_init_print!();
    writeln!(serial, "hello world").unwrap();
    rprintln!("Hello from STM32F338!");
    // Send a byte
    //let test_byte = b'A';
    //nb::block!(serial.write(test_byte)).unwrap();
    //rprintln!("Sent: {}", test_byte as char);

    // Read it back
    // // Small delay to let the byte physically propagate to RX

    //let received = nb::block!(serial.read()).unwrap();
    //rprintln!("Received: {}", received as char);
    loop {
        // Read it back
        if let Some(line) = read_gps_line(&mut serial) {
            rprintln!("GPS trimmed: {}", line.trim());
        }
        //match nb::block!(serial.read()) {
        //Ok(received) => rprintln!("Received: {}", received as char),
        //Err(err) => rprintln!("USART read error: {:?}", err),
        //}
    }
}
*/

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("");
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);
    //---- GPIO ----
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let scl = gpiob
        .pb6
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let sda = gpiob
        .pb7
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    //---- I2C ----
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        embedded_time::rate::Hertz(400_000),
        clocks,
        &mut rcc.apb1,
    );
    //---- Wake up IMU ----
    i2c.write(IMU_ADDR, &[PWR_MGMT_1, 0x00]).unwrap(); // --when we stop program midflight we can
                                                       //have stale state on the bus.
    loop {
        match i2c.write(IMU_ADDR, &[PWR_MGMT_1, 0x00]) {
            Ok(_) => break, // success
            Err(stm32f3xx_hal::i2c::Error::Arbitration) => {
                rprintln!("Error: Arbitration");

                cortex_m::asm::delay(10_000);
            }
            Err(e) => panic!("I2C error: {:?}", e),
        }
    }
    //---- WHO_AM_I test ----
    let mut whoami = [0u8];
    i2c.write_read(IMU_ADDR, &[WHO_AM_I], &mut whoami).unwrap();

    //whoami should usually be 0x68
    let _whoami = whoami[0];

    //Check which gyro sensitivities we are using
    let mut buf = [0u8];
    i2c.write_read(IMU_ADDR, &[0x1B], &mut buf).unwrap();
    let gyro_config = buf[0];
    let gyro_fs_sel = (gyro_config >> 3) & 0b11;
    rprintln!("GYRO FS_SEL={}", gyro_fs_sel);
    i2c.write_read(IMU_ADDR, &[0x1C], &mut buf).unwrap();
    let accel_config = buf[0];
    let accel_fs_sel = (accel_config >> 3) & 0b11;
    rprintln!("ACCEL FS_SEL={}", accel_fs_sel);
    assert!(gyro_fs_sel == 0);
    assert!(accel_fs_sel == 0);

    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    timer.start(100.milliseconds());
    loop {
        nb::block!(timer.wait()).unwrap();
        let mut buf = [0u8; 14];
        i2c.write_read(IMU_ADDR, &[ACCEL_XOUT_H], &mut buf).unwrap();

        let ax = i16::from_be_bytes([buf[0], buf[1]]);
        let ay = i16::from_be_bytes([buf[2], buf[3]]);
        let az = i16::from_be_bytes([buf[4], buf[5]]);

        let gx = i16::from_be_bytes([buf[8], buf[9]]);
        let gy = i16::from_be_bytes([buf[10], buf[11]]);
        let gz = i16::from_be_bytes([buf[12], buf[13]]);
        rprintln!("{},{},{},{},{},{}", ax, ay, az, gx, gy, gz);
        cortex_m::asm::nop();
    }
}

//#[entry]
//fn main() -> ! {
//let dp = pac::Peripherals::take().unwrap();

//// Clock setup (leave defaults, they are fine)
//let mut flash = dp.FLASH.constrain();
//let mut rcc = dp.RCC.constrain();
//let clocks = rcc.cfgr.freeze(&mut flash.acr);

//// GPIOA
//let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//let mut led = gpioa
//.pa5
//.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

//// --- UART ---
//let tx_pin = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
//let rx_pin = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

//let serial = Serial::usart2(
//dp.USART2,
//(tx_pin, rx_pin),
//Config::default().baudrate(115_200.bps()),
//clocks,
//&mut rcc.apb1,
//);

//let (mut tx, _rx) = serial.split();

//unsafe {
//LOGGER = Some(UartLogger { uart: &mut tx });
//log::set_logger(LOGGER.as_ref().unwrap()).unwrap();
//log::set_max_level(log::LevelFilter::Info);
//}

//info!("STM32 booted");
//info!("STM32 booted");
//let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
//timer.start(1.seconds());

//loop {
//led.toggle().unwrap();

//info!("LED toggled");

//// Wait until timer expires
//nb::block!(timer.wait()).unwrap();
//}
//}
