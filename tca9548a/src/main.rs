#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use embedded_hal::blocking::i2c::Write;
use mpu6050_firmware::{Mpu6050, MPU6050_DEFAULT_ADDR};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal::{i2c::I2c, pac, prelude::*};

const TCA9548A_ADDR: u8 = 0x70;
const NUM_SENSORS: usize = 1;

/// Select a single TCA9548A channel (0..7).
/// Writes the channel bitmask to the mux — all subsequent I2C traffic
/// goes to the selected channel's SDA/SCL pair.
fn tca_select<I, E>(i2c: &mut I, channel: u8) -> Result<(), E>
where
    I: Write<Error = E>,
{
    i2c.write(TCA9548A_ADDR, &[1 << channel])
}

fn detect_connected_sensors<I2C>(i2c: &mut I2C) -> [Option<u8>; 8]
where
    I2C: Write,
    I2C::Error: core::fmt::Debug,
{
    let mut channels = [None; 8];

    for ch in 0..8 {
        // select TCA channel
        let mask = 1 << ch;
        match i2c.write(TCA9548A_ADDR, &[mask]) {
            Ok(_) => {},
            Err(e) => rprintln!("Error writing to tca: {:?}", e),
        }
        let result = i2c.write(MPU6050_DEFAULT_ADDR, &[0x75]); // WHO_AM_I register
        if result.is_ok() {
            channels[ch as usize] = Some(ch);
        }
    }

    channels
}
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("TCA9548A multi-sensor reader starting...");

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


    // Scan the I2C bus to verify what devices are present.
    // Use a 1-byte read probe — empty writes may not work on STM32F3 I2C v2.
    rprintln!("Scanning I2C bus...");
    let mut found = 0u8;
    for addr in 0x03u8..0x78 {
        let mut buf = [0u8; 1];
        if i2c.write_read(addr, &[0x00], &mut buf).is_ok() {
            rprintln!("  0x{:02X} ACK", addr);
            found += 1;
        }
    }
    rprintln!("Scan done, {} device(s) found.", found);

    let detected = detect_connected_sensors(&mut i2c);
    let num_connected = detected.iter().filter(|c| c.is_some()).count();
    rprintln!("detected: {:?}", detected);
    if num_connected == 0 {
        panic!["no devices found"];
    }

    // Both MPU6050s share the same I2C address — the TCA9548A mux
    // routes traffic to the correct physical sensor.
    let mpu = Mpu6050::new(MPU6050_DEFAULT_ADDR);

    // --- Initialise each sensor on its channel ---
    for ch in 0..num_connected as u8 {
        let mut tried_num = 0;
        loop {
            tried_num += 1;
            if tried_num > 2 {
                panic!["ch: {:?} couldnt connect", ch];
            }
            cortex_m::asm::delay(50_000);
            rprintln!("Selecting channel {}...", ch);
            match tca_select(&mut i2c, ch) {
                Ok(()) => rprintln!("  Channel {} selected.", ch),
                Err(e) => {
                    rprintln!("  TCA select ch {} failed: {:?}", ch, e);
                    continue;
                }
            }

            rprintln!("  Initialising sensor on channel {}...", ch);

            // Offsets are device-specific — calibrate per sensor if needed
            if let Err(e) = mpu.set_gyro_offsets(&mut i2c, 220, 76, -85) {
                rprintln!("  set_gyro_offsets failed: {:?}", e);
                continue;
            }
            if let Err(e) = mpu.set_accel_z_offset(&mut i2c, 1788) {
                rprintln!("  set_accel_z_offset failed: {:?}", e);
                continue;
            }
            if let Err(e) = mpu.dmp_initialize(&mut i2c, |ms| cortex_m::asm::delay(8_000 * ms)) {
                rprintln!("  dmp_initialize failed: {:?}", e);
                continue;
            }
            rprintln!("  Sensor {} ready.", ch);
            break
        }
    }

    rprintln!("Entering quaternion read loop...");
    let cycles_per_us = clocks.sysclk().0 / 1_000_000;

    loop {
        for ch in 0..num_connected as u8 {
            if tca_select(&mut i2c, ch).is_err() {
                continue;
            }

            match mpu.read_dmp_quaternion(&mut i2c) {
                Ok(Some(q)) => {
                    let total_us = DWT::cycle_count() / cycles_per_us;
                    let seconds = total_us / 1_000_000;
                    let micros = total_us % 1_000_000;
                    rprintln!(
                        "{},{}.{:06},{:.4},{:.4},{:.4},{:.4}",
                        ch,
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
                    rprintln!("I2C error on ch {}: {:?}", ch, e);
                }
            }
        }

        // Small delay to avoid excessive bus polling (~1 ms)
        cortex_m::asm::delay(8_000);
    }
}
