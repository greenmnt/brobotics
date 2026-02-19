#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52840_hal::pac::Peripherals;
use panic_halt as _;

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();

    // Enter System OFF (~0.5µA). Only a reset (double-tap) wakes it.
    p.POWER.systemoff.write(|w| w.systemoff().enter());

    // Should never reach here, but satisfy the -> ! return type
    loop {
        cortex_m::asm::wfi();
    }
}
