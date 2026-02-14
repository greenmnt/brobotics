#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use mpu6050_firmware::{Mpu6050, MPU6050_DEFAULT_ADDR};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal::{i2c::I2c, pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("MPU6050 DMP Quaternion Reader starting...");

    let dp = pac::Peripherals::take().unwrap();

    // DWT cycle counter for timestamps
    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut dwt = core_peripherals.DWT;
    let mut dcb = core_peripherals.DCB;
    dcb.enable_trace();
    dwt.enable_cycle_counter();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);

    // GPIO setup
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // I2C setup at 400 kHz
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        embedded_time::rate::Hertz(400_000),
        clocks,
        &mut rcc.apb1,
    );

    let mpu = Mpu6050::new(MPU6050_DEFAULT_ADDR);

    // Set device-specific calibration offsets
    rprintln!("Setting sensor offsets...");
    mpu.set_gyro_offsets(&mut i2c, 220, 76, -85).unwrap();
    mpu.set_accel_z_offset(&mut i2c, 1788).unwrap();

    // Run DMP initialization
    rprintln!("Initializing DMP...");
    mpu.dmp_initialize(&mut i2c, |ms| cortex_m::asm::delay(8_000 * ms))
        .unwrap();
    rprintln!("DMP initialized successfully.");

    rprintln!("Entering quaternion read loop...");
    let cycles_per_us = clocks.sysclk().0 / 1_000_000;

    loop {
        match mpu.read_dmp_quaternion(&mut i2c) {
            Ok(Some(q)) => {
                let total_us = DWT::cycle_count() / cycles_per_us;
                let seconds = total_us / 1_000_000;
                let micros = total_us % 1_000_000;
                rprintln!(
                    "{}.{:06},{:.4},{:.4},{:.4},{:.4}",
                    seconds,
                    micros,
                    q.w,
                    q.x,
                    q.y,
                    q.z
                );
            }
            Ok(None) => {} // No packet available or FIFO overflow (auto-reset)
            Err(e) => {
                rprintln!("I2C error: {:?}", e);
            }
        }

        // Small delay to avoid excessive bus polling (~1 ms)
        cortex_m::asm::delay(8_000);
    }
}
