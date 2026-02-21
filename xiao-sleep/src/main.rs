#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use nrf52840_hal::{self as hal, pac::Peripherals, pac::CorePeripherals, twim, Twim};
use panic_halt as _;

const MPU6050_ADDR: u8 = 0x68;
const REG_PWR_MGMT_1: u8 = 0x6B;

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);

    // Blue LED (active LOW) — blinks during the reset window
    let mut led = port0.p0_06.into_push_pull_output(hal::gpio::Level::High);
    let mut delay = hal::Delay::new(cp.SYST);

    // 3-second window: blink blue LED so you can double-tap reset for bootloader
    for _ in 0..6 {
        let _ = led.set_low();  // on
        delay.delay_ms(250u32);
        let _ = led.set_high(); // off
        delay.delay_ms(250u32);
    }

    // Init I2C to talk to MPU6050 before sleeping
    let scl = port0.p0_05.into_floating_input().degrade();
    let sda = port0.p0_04.into_floating_input().degrade();
    let mut i2c = Twim::new(p.TWIM0, twim::Pins { scl, sda }, twim::Frequency::K400);

    // Put MPU6050 into sleep mode: set bit 6 of PWR_MGMT_1 (3.9mA → ~10µA)
    let _ = i2c.write(MPU6050_ADDR, &[REG_PWR_MGMT_1, 0x40]);

    // LED off, enter nRF52840 System OFF (~0.5µA). Only a reset wakes it.
    let _ = led.set_high();
    p.POWER.systemoff.write(|w| w.systemoff().enter());

    loop {
        cortex_m::asm::wfi();
    }
}
