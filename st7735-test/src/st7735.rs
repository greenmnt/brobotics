//! Minimal ST7735 driver for 1.44" 128x128 displays using embedded-hal 0.2 traits.

use embedded_hal::blocking::spi::Write as SpiWrite;
use embedded_hal::digital::v2::OutputPin;

use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Size},
    pixelcolor::{Rgb565, raw::RawU16, raw::RawData},
    Pixel,
};

// ST7735 commands
const SWRESET: u8 = 0x01;
const SLPOUT: u8 = 0x11;
const NORON: u8 = 0x13;
const INVON: u8 = 0x21;
const DISPON: u8 = 0x29;
const CASET: u8 = 0x2A;
const RASET: u8 = 0x2B;
const RAMWR: u8 = 0x2C;
const FRMCTR1: u8 = 0xB1;
const COLMOD: u8 = 0x3A;
const MADCTL: u8 = 0x36;

// 1.44" 128x128 module offsets (panel is 132x162, windowed)
const COL_OFFSET: u16 = 2;
const ROW_OFFSET: u16 = 3;

pub const WIDTH: u16 = 128;
pub const HEIGHT: u16 = 128;

pub struct ST7735<SPI, DC, RST, CS> {
    spi: SPI,
    dc: DC,
    rst: RST,
    cs: CS,
}

impl<SPI, DC, RST, CS, E> ST7735<SPI, DC, RST, CS>
where
    SPI: SpiWrite<u8, Error = E>,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
{
    pub fn new(spi: SPI, dc: DC, rst: RST, cs: CS) -> Self {
        Self { spi, dc, rst, cs }
    }

    /// Send a command byte (DC low)
    fn write_command(&mut self, cmd: u8) {
        let _ = self.dc.set_low();
        let _ = self.cs.set_low();
        let _ = self.spi.write(&[cmd]);
        let _ = self.cs.set_high();
    }

    /// Send data bytes (DC high)
    fn write_data(&mut self, data: &[u8]) {
        let _ = self.dc.set_high();
        let _ = self.cs.set_low();
        let _ = self.spi.write(data);
        let _ = self.cs.set_high();
    }

    /// Send a command followed by data
    fn write_command_data(&mut self, cmd: u8, data: &[u8]) {
        self.write_command(cmd);
        self.write_data(data);
    }

    /// Hardware reset + initialization sequence for 1.44" 128x128 ST7735
    pub fn init(&mut self) {
        // Hardware reset
        let _ = self.rst.set_high();
        delay_ms(5);
        let _ = self.rst.set_low();
        delay_ms(20);
        let _ = self.rst.set_high();
        delay_ms(20);

        // Software reset
        self.write_command(SWRESET);
        delay_ms(150);

        // Exit sleep
        self.write_command(SLPOUT);
        delay_ms(150);

        // Frame rate control (normal mode): RTNA=0x01, FPA=0x2C, BPA=0x2D
        self.write_command_data(FRMCTR1, &[0x01, 0x2C, 0x2D]);

        // Pixel format: 16-bit RGB565
        self.write_command_data(COLMOD, &[0x05]);

        // Memory access control: row/col addressing, RGB order
        self.write_command_data(MADCTL, &[0x08]);

        // Display inversion on (required for many 1.44" modules)
        self.write_command(INVON);

        // Normal display mode on
        self.write_command(NORON);
        delay_ms(10);

        // Display on
        self.write_command(DISPON);
        delay_ms(100);
    }

    /// Set the drawing window (column and row address)
    pub fn set_address_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let x0 = x0 + COL_OFFSET;
        let x1 = x1 + COL_OFFSET;
        let y0 = y0 + ROW_OFFSET;
        let y1 = y1 + ROW_OFFSET;

        self.write_command(CASET);
        self.write_data(&[
            (x0 >> 8) as u8, x0 as u8,
            (x1 >> 8) as u8, x1 as u8,
        ]);

        self.write_command(RASET);
        self.write_data(&[
            (y0 >> 8) as u8, y0 as u8,
            (y1 >> 8) as u8, y1 as u8,
        ]);

        self.write_command(RAMWR);
    }

    /// Fill the entire screen with a single RGB565 color
    pub fn fill_screen(&mut self, color: u16) {
        self.set_address_window(0, 0, WIDTH - 1, HEIGHT - 1);

        let hi = (color >> 8) as u8;
        let lo = color as u8;
        // Write in row-sized chunks to keep stack usage reasonable
        let mut row_buf = [0u8; 256]; // 128 pixels * 2 bytes = 256
        for i in 0..128 {
            row_buf[i * 2] = hi;
            row_buf[i * 2 + 1] = lo;
        }

        let _ = self.dc.set_high();
        let _ = self.cs.set_low();
        for _ in 0..HEIGHT {
            let _ = self.spi.write(&row_buf);
        }
        let _ = self.cs.set_high();
    }

    /// Write a single pixel at (x, y) with an RGB565 color
    pub fn set_pixel(&mut self, x: u16, y: u16, color: u16) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        self.set_address_window(x, y, x, y);
        self.write_data(&[(color >> 8) as u8, color as u8]);
    }
}

// DrawTarget implementation for embedded-graphics
impl<SPI, DC, RST, CS, E> DrawTarget for ST7735<SPI, DC, RST, CS>
where
    SPI: SpiWrite<u8, Error = E>,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            let x = coord.x;
            let y = coord.y;
            if x >= 0 && x < WIDTH as i32 && y >= 0 && y < HEIGHT as i32 {
                let raw: u16 = RawU16::from(color).into_inner();
                self.set_pixel(x as u16, y as u16, raw);
            }
        }
        Ok(())
    }
}

impl<SPI, DC, RST, CS, E> OriginDimensions for ST7735<SPI, DC, RST, CS>
where
    SPI: SpiWrite<u8, Error = E>,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

/// Rough delay in milliseconds at 8 MHz sysclk
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(8_000 * ms);
}
