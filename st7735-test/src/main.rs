#![no_std]
#![no_main]

mod st7735;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal::{pac, prelude::*, spi::Spi};

use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};

// RGB565 color constants
const RED: u16 = 0xF800;
const GREEN: u16 = 0x07E0;
const BLUE: u16 = 0x001F;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("ST7735 Display Test starting...");

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);

    // GPIO setup
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    // SPI1 pins: PA5 = SCK, PA6 = MISO (unused but required), PA7 = MOSI
    let sck = gpioa
        .pa5
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    // Control pins: PA2 = DC, PA3 = RST, PA4 = CS
    let dc = gpioa
        .pa2
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let rst = gpioa
        .pa3
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let cs = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    // SPI1: Mode 0 (default), ~4 MHz
    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        4.MHz(),
        clocks,
        &mut rcc.apb2,
    );

    rprintln!("SPI initialized, creating display driver...");

    let mut display = st7735::ST7735::new(spi, dc, rst, cs);
    display.init();
    rprintln!("Display initialized.");

    // Color cycle test: red, green, blue (1 second each)
    rprintln!("Filling red...");
    display.fill_screen(RED);
    cortex_m::asm::delay(8_000_000); // ~1s at 8MHz

    rprintln!("Filling green...");
    display.fill_screen(GREEN);
    cortex_m::asm::delay(8_000_000);

    rprintln!("Filling blue...");
    display.fill_screen(BLUE);
    cortex_m::asm::delay(8_000_000);

    // Clear to black and draw text
    display.fill_screen(0x0000);

    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);
    let _ = Text::new("Hello MPU!", Point::new(10, 64), style).draw(&mut display);
    rprintln!("Text drawn. Done!");

    loop {
        //NOTE: never use cortex_m::asm::wfi(); it can lock out the debugger
        cortex_m::asm::nop();
    }
}
