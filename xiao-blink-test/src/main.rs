#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use nrf52840_hal::{self as hal, gpio::Level, pac::Peripherals};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);

    // XIAO nRF52840 onboard LEDs (active LOW)
    // Red = P0.26, Green = P0.30, Blue = P0.06
    let mut led_red = port0.p0_26.into_push_pull_output(Level::High);
    let mut led_green = port0.p0_30.into_push_pull_output(Level::High);
    let mut led_blue = port0.p0_06.into_push_pull_output(Level::High);

    let mut timer = hal::delay::Delay::new(hal::pac::CorePeripherals::take().unwrap().SYST);

    loop {
        led_red.set_low().unwrap();
        timer.delay_ms(200u32);
        led_red.set_high().unwrap();

        led_green.set_low().unwrap();
        timer.delay_ms(200u32);
        led_green.set_high().unwrap();

        led_blue.set_low().unwrap();
        timer.delay_ms(200u32);
        led_blue.set_high().unwrap();

        timer.delay_ms(400u32);
    }
}
