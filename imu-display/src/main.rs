#![no_std]
#![no_main]

mod dmp_firmware;
mod st7735;

use cortex_m_rt::entry;
use dmp_firmware::DMP_FIRMWARE;
use cortex_m::peripheral::DWT;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use libm::{asinf, atan2f};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal::{i2c::I2c, pac, prelude::*, spi::Spi};

use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};

use core::fmt::Write as FmtWrite;

// ── MPU6050 constants ──────────────────────────────────────────────
const MPU_ADDR: u8 = 0x68;
const DMP_PACKET_SIZE: usize = 42;

const PWR_MGMT_1: u8 = 0x6B;
const USER_CTRL: u8 = 0x6A;
const SMPLRT_DIV: u8 = 0x19;
const CONFIG: u8 = 0x1A;
const GYRO_CONFIG: u8 = 0x1B;
const INT_ENABLE: u8 = 0x38;
const INT_STATUS: u8 = 0x3A;
const FIFO_COUNT_H: u8 = 0x72;
const FIFO_R_W: u8 = 0x74;
const I2C_SLV0_ADDR: u8 = 0x25;
const BANK_SEL: u8 = 0x6D;
const MEM_START_ADDR: u8 = 0x6E;
const MEM_R_W: u8 = 0x6F;
const DMP_CFG_1: u8 = 0x70;
const DMP_CFG_2: u8 = 0x71;
const XG_OFFS_TC: u8 = 0x00;
const MOT_THR: u8 = 0x1F;
const MOT_DUR: u8 = 0x20;
const ZRMOT_THR: u8 = 0x21;
const ZRMOT_DUR: u8 = 0x22;

const MPU6050_RA_XG_OFFS_USRH: u8 = 0x13;
const MPU6050_RA_YG_OFFS_USRH: u8 = 0x15;
const MPU6050_RA_ZG_OFFS_USRH: u8 = 0x17;
const MPU6050_RA_WHO_AM_I: u8 = 0x75;
const MPU6050_RA_ZA_OFFS_H: u8 = 0x0A;

const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

// ── MPU6050 register macros ────────────────────────────────────────
macro_rules! write_reg {
    ($i2c:expr, $reg:expr, $val:expr) => {
        $i2c.write(MPU_ADDR, &[$reg, $val]).unwrap()
    };
}

macro_rules! read_reg {
    ($i2c:expr, $reg:expr) => {{
        let mut buf = [0u8; 1];
        $i2c.write_read(MPU_ADDR, &[$reg], &mut buf).unwrap();
        buf[0]
    }};
}

macro_rules! write_bit {
    ($i2c:expr, $reg:expr, $bit:expr, $val:expr) => {{
        let mut b = read_reg!($i2c, $reg);
        if $val {
            b |= 1 << $bit;
        } else {
            b &= !(1 << $bit);
        }
        write_reg!($i2c, $reg, b);
    }};
}

macro_rules! write_bits {
    ($i2c:expr, $reg:expr, $start_bit:expr, $len:expr, $val:expr) => {{
        let mut b = read_reg!($i2c, $reg);
        let shift = $start_bit - $len + 1;
        let mask: u8 = ((1u16 << $len) - 1) as u8;
        b &= !(mask << shift);
        b |= ($val & mask) << shift;
        write_reg!($i2c, $reg, b);
    }};
}

macro_rules! write_word {
    ($i2c:expr, $reg:expr, $val:expr) => {{
        let v: u16 = $val as u16;
        $i2c.write(MPU_ADDR, &[$reg, (v >> 8) as u8, v as u8])
            .unwrap();
    }};
}

macro_rules! read_i16 {
    ($i2c:expr, $reg:expr) => {{
        let mut buf = [0u8; 2];
        $i2c.write_read(MPU_ADDR, &[$reg], &mut buf).unwrap();
        ((buf[0] as i16) << 8) | (buf[1] as i16)
    }};
}

// ── MPU6050 functions ──────────────────────────────────────────────
fn upload_dmp_firmware<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    let mut addr: u16 = 0;
    let mut remaining = DMP_FIRMWARE.len();
    let mut src_offset = 0;

    while remaining > 0 {
        let bank = (addr >> 8) as u8;
        let mem_addr = (addr & 0xFF) as u8;
        let bank_remaining = 256 - (mem_addr as usize);
        let chunk_size = remaining.min(16).min(bank_remaining);

        i2c.write(MPU_ADDR, &[BANK_SEL, bank]).unwrap();
        i2c.write(MPU_ADDR, &[MEM_START_ADDR, mem_addr]).unwrap();

        let mut buf = [0u8; 17];
        buf[0] = MEM_R_W;
        buf[1..1 + chunk_size].copy_from_slice(&DMP_FIRMWARE[src_offset..src_offset + chunk_size]);
        i2c.write(MPU_ADDR, &buf[..1 + chunk_size]).unwrap();

        addr += chunk_size as u16;
        src_offset += chunk_size;
        remaining -= chunk_size;
    }
}

fn write_dmp_memory<I>(i2c: &mut I, bank: u8, offset: u8, data: &[u8])
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    i2c.write(MPU_ADDR, &[BANK_SEL, bank]).unwrap();
    i2c.write(MPU_ADDR, &[MEM_START_ADDR, offset]).unwrap();
    let mut buf = [0u8; 17];
    buf[0] = MEM_R_W;
    let len = data.len().min(16);
    buf[1..1 + len].copy_from_slice(&data[..len]);
    i2c.write(MPU_ADDR, &buf[..1 + len]).unwrap();
}

fn set_sensor_offsets<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    write_word!(i2c, MPU6050_RA_XG_OFFS_USRH, 220);
    write_word!(i2c, MPU6050_RA_YG_OFFS_USRH, 76);
    write_word!(i2c, MPU6050_RA_ZG_OFFS_USRH, -85i16);
    let dev_id = read_reg!(i2c, MPU6050_RA_WHO_AM_I);
    let reg = if dev_id < 0x38 {
        MPU6050_RA_ZA_OFFS_H
    } else {
        0x7D
    };
    write_word!(i2c, reg, 1788);
}

fn dmp_initialize<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    rprintln!("Resetting MPU6050...");
    write_bit!(i2c, PWR_MGMT_1, 7, true);
    cortex_m::asm::delay(8_000 * 30);
    write_bit!(i2c, PWR_MGMT_1, 6, false);

    i2c.write(MPU_ADDR, &[BANK_SEL, 0x70]).unwrap();
    i2c.write(MPU_ADDR, &[MEM_START_ADDR, 0x06]).unwrap();
    let _hw_rev = read_reg!(i2c, MEM_R_W);

    i2c.write(MPU_ADDR, &[BANK_SEL, 0x00]).unwrap();
    write_reg!(i2c, I2C_SLV0_ADDR, 0x7F);
    write_bit!(i2c, USER_CTRL, 5, false);
    write_reg!(i2c, I2C_SLV0_ADDR, 0x68);
    write_bit!(i2c, USER_CTRL, 1, true);
    cortex_m::asm::delay(8_000 * 20);

    write_bits!(i2c, PWR_MGMT_1, 2, 3, 0x03);
    write_reg!(i2c, INT_ENABLE, 0x12);
    write_reg!(i2c, SMPLRT_DIV, 4);
    write_bits!(i2c, CONFIG, 5, 3, 0x01);
    write_bits!(i2c, CONFIG, 2, 3, 0x03);
    write_bits!(i2c, GYRO_CONFIG, 4, 2, 0x03);

    rprintln!("Uploading DMP firmware...");
    upload_dmp_firmware(i2c);

    write_dmp_memory(i2c, 0x02, 0x16, &[0x00, 0x01]);
    write_reg!(i2c, DMP_CFG_1, 0x03);
    write_reg!(i2c, DMP_CFG_2, 0x00);
    write_bit!(i2c, XG_OFFS_TC, 0, false);
    write_reg!(i2c, MOT_THR, 2);
    write_reg!(i2c, ZRMOT_THR, 156);
    write_reg!(i2c, MOT_DUR, 80);
    write_reg!(i2c, ZRMOT_DUR, 0);

    write_bit!(i2c, USER_CTRL, 6, true);
    write_bit!(i2c, USER_CTRL, 3, true);
    write_bit!(i2c, USER_CTRL, 7, false);
    write_bit!(i2c, USER_CTRL, 2, true);
    let _ = read_reg!(i2c, INT_STATUS);
    write_bit!(i2c, USER_CTRL, 7, true);
    rprintln!("DMP initialized.");
}

// ── Quaternion → Euler ─────────────────────────────────────────────
fn quat_to_euler(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    // Roll (rotation around X)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (rotation around Y)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        // Gimbal lock
        if sinp > 0.0 {
            90.0
        } else {
            -90.0
        }
    } else {
        asinf(sinp) * RAD_TO_DEG
    };

    // Yaw (rotation around Z)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    (pitch, roll, yaw)
}

fn quat_to_angle(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    use libm::{atan2f, sqrtf};
    let gx = 2.0 * (x * z - w * y);
    let gy = 2.0 * (w * x + y * z);
    let gz = w * w - x * x - y * y + z * z;

    let yaw = atan2f(2.0 * x * y - 2.0 * w * z, 2.0 * w * w + 2.0 * x * x - 1.0);
    let pitch = atan2f(gx, sqrtf(gy * gy + gz * gz));
    let roll = atan2f(gy, gz);
    let deg = 180.0 / core::f32::consts::PI;
    (pitch * deg, roll * deg, yaw * deg)
}

// ── Tiny formatting buffer ─────────────────────────────────────────
struct LineBuf {
    buf: [u8; 32],
    pos: usize,
}

impl LineBuf {
    fn new() -> Self {
        Self {
            buf: [b' '; 32],
            pos: 0,
        }
    }

    fn as_str(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.buf[..self.pos]) }
    }
}

impl FmtWrite for LineBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.pos;
        let len = bytes.len().min(remaining);
        self.buf[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
        Ok(())
    }
}

//  ── Main ───────────────────────────────────────────────────────────
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("IMU Display starting...");

    let dp = pac::Peripherals::take().unwrap();
    
    //for clock cycles
    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut dwt = core_peripherals.DWT;
    let mut dcb = core_peripherals.DCB;
    dcb.enable_trace();
    dwt.enable_cycle_counter();


    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);

    // ── I2C1 (MPU6050): PB6=SCL, PB7=SDA ──
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        embedded_time::rate::Hertz(400_000),
        clocks,
        &mut rcc.apb1,
    );

    // ── SPI1 (ST7735): PA5=SCK, PA6=MISO(unused), PA7=MOSI ──
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let sck = gpioa
        .pa5
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let dc = gpioa
        .pa2
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let rst = gpioa
        .pa3
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let cs = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let spi = Spi::new(dp.SPI1, (sck, miso, mosi), 4.MHz(), clocks, &mut rcc.apb2);

    // ── Init display ──
    rprintln!("Initializing display...");
    let mut display = st7735::ST7735::new(spi, dc, rst, cs);
    display.init();
    display.fill_screen(0x0000); // black

    // Draw static labels
    let label_style = MonoTextStyle::new(&FONT_9X15, Rgb565::CSS_GRAY);
    let _ = Text::new("Pitch:", Point::new(4, 30), label_style).draw(&mut display);
    let _ = Text::new("Roll:", Point::new(4, 62), label_style).draw(&mut display);
    let _ = Text::new("Yaw:", Point::new(4, 94), label_style).draw(&mut display);

    let value_style = MonoTextStyle::new(&FONT_9X15, Rgb565::CSS_LIME_GREEN);
    let black_style = MonoTextStyle::new(&FONT_9X15, Rgb565::BLACK);

    // ── Init MPU6050 DMP ──
    rprintln!("Initializing MPU6050 DMP...");
    set_sensor_offsets(&mut i2c);
    dmp_initialize(&mut i2c);

    rprintln!("Running...");

    let mut packet = [0u8; DMP_PACKET_SIZE];
    let mut prev_pitch = LineBuf::new();
    let mut prev_roll = LineBuf::new();
    let mut prev_yaw = LineBuf::new();

    // Seed previous buffers so first erase works
    let _ = core::write!(&mut prev_pitch, "---.-");
    let _ = core::write!(&mut prev_roll, "---.-");
    let _ = core::write!(&mut prev_yaw, "---.-");

    loop {
        // Read FIFO count
        let mut fifo_buf = [0u8; 2];
        i2c.write_read(MPU_ADDR, &[FIFO_COUNT_H], &mut fifo_buf)
            .unwrap();
        let fifo_count = u16::from_be_bytes(fifo_buf);

        if fifo_count > 200 {
            write_bit!(i2c, USER_CTRL, 2, true);
            cortex_m::asm::delay(8_000);
            continue;
        }

        if fifo_count >= DMP_PACKET_SIZE as u16 {
            // Drain to latest packet
            let mut count = fifo_count;
            while count >= 2 * DMP_PACKET_SIZE as u16 {
                i2c.write_read(MPU_ADDR, &[FIFO_R_W], &mut packet).unwrap();
                count -= DMP_PACKET_SIZE as u16;
            }
            i2c.write_read(MPU_ADDR, &[FIFO_R_W], &mut packet).unwrap();

            let qw = i16::from_be_bytes([packet[0], packet[1]]) as f32 / 16384.0;
            let qx = i16::from_be_bytes([packet[4], packet[5]]) as f32 / 16384.0;
            let qy = i16::from_be_bytes([packet[8], packet[9]]) as f32 / 16384.0;
            let qz = i16::from_be_bytes([packet[12], packet[13]]) as f32 / 16384.0;
            let cycles_per_us = clocks.sysclk().0 / 1_000_000;
            let total_us = DWT::cycle_count() / cycles_per_us;
            let seconds = total_us / 1_000_000;
            let micros = total_us % 1_000_000;
            rprintln!(
                "{}.{:06},{:.4},{:.4},{:.4},{:.4}",
                seconds,
                micros,
                qw,
                qx,
                qy,
                qz
            );

            //let (pitch, roll, yaw) = quat_to_euler(qw, qx, qy, qz);
            let (pitch, roll, yaw) = quat_to_angle(qw, qx, qy, qz);

            // Format new values
            let mut pitch_buf = LineBuf::new();
            let mut roll_buf = LineBuf::new();
            let mut yaw_buf = LineBuf::new();
            let _ = core::write!(&mut pitch_buf, "{:.1}", pitch);
            let _ = core::write!(&mut roll_buf, "{:.1}", roll);
            let _ = core::write!(&mut yaw_buf, "{:.1}", yaw);

            // Erase previous text by overwriting with black, then draw new
            let val_x = 4;
            let _ = Text::new(prev_pitch.as_str(), Point::new(val_x, 50), black_style)
                .draw(&mut display);
            let _ = Text::new(pitch_buf.as_str(), Point::new(val_x, 50), value_style)
                .draw(&mut display);

            let _ = Text::new(prev_roll.as_str(), Point::new(val_x, 82), black_style)
                .draw(&mut display);
            let _ =
                Text::new(roll_buf.as_str(), Point::new(val_x, 82), value_style).draw(&mut display);

            let _ = Text::new(prev_yaw.as_str(), Point::new(val_x, 114), black_style)
                .draw(&mut display);
            let _ =
                Text::new(yaw_buf.as_str(), Point::new(val_x, 114), value_style).draw(&mut display);

            prev_pitch = pitch_buf;
            prev_roll = roll_buf;
            prev_yaw = yaw_buf;
        }

        cortex_m::asm::delay(8_000 * 50); // ~50ms between updates
    }
}
