#![no_std]
#![no_main]

mod dmp_firmware;

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use dmp_firmware::DMP_FIRMWARE;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal::{i2c::I2c, pac, prelude::*};

const MPU_ADDR: u8 = 0x68;
const DMP_PACKET_SIZE: usize = 42;

// MPU6050 register addresses
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

/// Write a single register
macro_rules! write_reg {
    ($i2c:expr, $reg:expr, $val:expr) => {
        $i2c.write(MPU_ADDR, &[$reg, $val]).unwrap()
    };
}

/// Read a single register
macro_rules! read_reg {
    ($i2c:expr, $reg:expr) => {{
        let mut buf = [0u8; 1];
        $i2c.write_read(MPU_ADDR, &[$reg], &mut buf).unwrap();
        buf[0]
    }};
}

/// Write a single bit in a register (read-modify-write)
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

/// Write a bit field in a register (read-modify-write)
/// start_bit is MSB position, len is number of bits (same convention as i2cdevlib)
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

/// Write a 16-bit word to consecutive registers (MSB first)
macro_rules! write_word {
    ($i2c:expr, $reg:expr, $val:expr) => {{
        let v: u16 = $val as u16;
        let buf = [
            $reg,
            (v >> 8) as u8, // MSB
            v as u8,        // LSB
        ];
        $i2c.write(MPU_ADDR, &buf).unwrap();
    }};
}

/// Read a signed 16-bit value from consecutive registers (MSB first)
macro_rules! read_i16 {
    ($i2c:expr, $reg:expr) => {{
        let mut buf = [0u8; 2];
        $i2c.write_read(MPU_ADDR, &[$reg], &mut buf).unwrap();

        // Big-endian signed value
        ((buf[0] as i16) << 8) | (buf[1] as i16)
    }};
}

/// Upload the DMP firmware blob to MPU6050 memory banks.
/// Writes in 16-byte chunks via BANK_SEL / MEM_START_ADDR / MEM_R_W.
fn upload_dmp_firmware<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
    //I::Error: core::fmt::Debug,
{
    let mut addr: u16 = 0;
    let mut remaining = DMP_FIRMWARE.len();
    let mut src_offset = 0;

    while remaining > 0 {
        let bank = (addr >> 8) as u8;
        let mem_addr = (addr & 0xFF) as u8;

        // How many bytes until end of this 256-byte bank
        let bank_remaining = 256 - (mem_addr as usize);
        // Chunk size: min of 16, remaining firmware, remaining bank space
        let chunk_size = remaining.min(16).min(bank_remaining);

        // Set bank and start address
        i2c.write(MPU_ADDR, &[BANK_SEL, bank]).unwrap();
        i2c.write(MPU_ADDR, &[MEM_START_ADDR, mem_addr]).unwrap();

        // Build write buffer: register byte + data
        let mut buf = [0u8; 17]; // 1 register + max 16 data
        buf[0] = MEM_R_W;
        buf[1..1 + chunk_size].copy_from_slice(&DMP_FIRMWARE[src_offset..src_offset + chunk_size]);
        i2c.write(MPU_ADDR, &buf[..1 + chunk_size]).unwrap();

        addr += chunk_size as u16;
        src_offset += chunk_size;
        remaining -= chunk_size;
    }
}

/// Write data to a specific DMP memory bank/offset
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

pub const MPU6050_RA_XG_OFFS_USRH: u8 = 0x13;
pub const MPU6050_RA_YG_OFFS_USRH: u8 = 0x15;
pub const MPU6050_RA_ZG_OFFS_USRH: u8 = 0x17;
pub const MPU6050_RA_WHO_AM_I: u8 = 0x75;
pub const MPU6050_RA_ZA_OFFS_H: u8 = 0x0A;

fn set_sensor_offsets<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    let current_x_offset = read_i16!(i2c, MPU6050_RA_XG_OFFS_USRH);
    let current_y_offset = read_i16!(i2c, MPU6050_RA_YG_OFFS_USRH);
    let current_z_offset = read_i16!(i2c, MPU6050_RA_ZG_OFFS_USRH);
    rprintln!("Current offsets");
    rprintln!(
        "x={},y={},z={}",
        current_x_offset,
        current_y_offset,
        current_z_offset
    );

    rprintln!("Setting offsets");
    rprintln!("/tset_x_gyro_offset"); //MPU6050_RA_XG_OFFS_USRH

    write_word!(i2c, MPU6050_RA_XG_OFFS_USRH, 220);
    rprintln!("/tset_y_gyro_offset"); //MPU6050_RA_YG_OFFS_USRH
    write_word!(i2c, MPU6050_RA_YG_OFFS_USRH, 76);
    rprintln!("/tset_z_gyro_offset"); //MPU6050_RA_ZG_OFFS_USRH
    write_word!(i2c, MPU6050_RA_ZG_OFFS_USRH, -85i16);
    rprintln!("/tset_z_accel_offset");
    {
        let dev_id = read_reg!(i2c, MPU6050_RA_WHO_AM_I);
        rprintln!("dev_id: {}", dev_id);
        let reg = if dev_id < 0x38 {
            MPU6050_RA_ZA_OFFS_H // MPU6050 / MPU9150
        } else {
            0x7D // MPU6500 / MPU9250
        };
        write_word!(i2c, reg, 1788);
    }
}

/// Perform the full DMP initialization sequence (mirrors dmpInitialize() from C++)
fn dmp_initialize<I>(i2c: &mut I)
where
    I: Write + WriteRead,
    <I as Write>::Error: core::fmt::Debug,
    <I as WriteRead>::Error: core::fmt::Debug,
{
    // 1. Reset device (bit 7 of PWR_MGMT_1)
    rprintln!("Resetting MPU6050...");
    write_bit!(i2c, PWR_MGMT_1, 7, true);
    cortex_m::asm::delay(8_000 * 30); // ~30ms at 8MHz

    // 2. Disable sleep (bit 6 of PWR_MGMT_1)
    write_bit!(i2c, PWR_MGMT_1, 6, false);

    // 3. Hardware revision check: set memory bank 0x10 with userBank and prefetch flags
    //    setMemoryBank(0x10, true, true) => bank = 0x10 | 0x20 | 0x40 = 0x70
    rprintln!("Checking hardware revision...");
    i2c.write(MPU_ADDR, &[BANK_SEL, 0x70]).unwrap();
    i2c.write(MPU_ADDR, &[MEM_START_ADDR, 0x06]).unwrap();
    let hw_rev = read_reg!(i2c, MEM_R_W);
    rprintln!("HW revision: {}", hw_rev);

    // 4. Reset BANK_SEL to 0
    i2c.write(MPU_ADDR, &[BANK_SEL, 0x00]).unwrap();

    // 5. Set I2C_SLV0_ADDR to 0x7F
    write_reg!(i2c, I2C_SLV0_ADDR, 0x7F);

    // 6. Disable I2C master mode: clear bit 5 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 5, false);

    // 7. Set I2C_SLV0_ADDR to 0x68 (self)
    write_reg!(i2c, I2C_SLV0_ADDR, 0x68);

    // 8. Reset I2C master: set bit 1 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 1, true);
    cortex_m::asm::delay(8_000 * 20); // ~20ms at 8MHz

    // 9. Set clock source to PLL_ZGYRO: bits [2:0] of PWR_MGMT_1 = 0x03
    write_bits!(i2c, PWR_MGMT_1, 2, 3, 0x03);

    // 10. Enable DMP and FIFO overflow interrupts: INT_ENABLE = 0x12
    write_reg!(i2c, INT_ENABLE, 0x12);

    // 11. Sample rate divider: SMPLRT_DIV = 4 (200Hz)
    write_reg!(i2c, SMPLRT_DIV, 4);

    // 12. External frame sync: CONFIG bits [5:3] = 1
    write_bits!(i2c, CONFIG, 5, 3, 0x01);

    // 13. DLPF mode: CONFIG bits [2:0] = 3 (42Hz)
    write_bits!(i2c, CONFIG, 2, 3, 0x03);

    // 14. Gyro full-scale range: GYRO_CONFIG bits [4:3] = 3 (±2000°/s)
    write_bits!(i2c, GYRO_CONFIG, 4, 2, 0x03);

    // 15. Upload DMP firmware (1929 bytes)
    rprintln!("Uploading DMP firmware ({} bytes)...", DMP_FIRMWARE.len());
    upload_dmp_firmware(i2c);
    rprintln!("DMP firmware uploaded.");

    // 16. Write FIFO rate divisor to DMP memory: bank 0x02, offset 0x16, data [0x00, 0x01]
    write_dmp_memory(i2c, 0x02, 0x16, &[0x00, 0x01]);

    // 17. Set DMP config registers
    write_reg!(i2c, DMP_CFG_1, 0x03);
    write_reg!(i2c, DMP_CFG_2, 0x00);

    // 18. Clear OTP bank valid: clear bit 0 of XG_OFFS_TC
    write_bit!(i2c, XG_OFFS_TC, 0, false);

    // 19. Motion detection thresholds
    write_reg!(i2c, MOT_THR, 2);
    write_reg!(i2c, ZRMOT_THR, 156);
    write_reg!(i2c, MOT_DUR, 80);
    write_reg!(i2c, ZRMOT_DUR, 0);

    // 20. Enable FIFO: set bit 6 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 6, true);

    // 21. Reset DMP: set bit 3 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 3, true);

    // 22. Disable DMP: clear bit 7 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 7, false);

    // 23. Reset FIFO: set bit 2 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 2, true);

    // 24. Clear interrupts by reading INT_STATUS
    let _ = read_reg!(i2c, INT_STATUS);

    // 25. Enable DMP: set bit 7 of USER_CTRL
    write_bit!(i2c, USER_CTRL, 7, true);

    rprintln!("DMP initialized successfully.");
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("MPU6050 DMP Quaternion Reader starting...");

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

    // GPIO setup
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // I2C setup at 400kHz
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        embedded_time::rate::Hertz(400_000),
        clocks,
        &mut rcc.apb1,
    );

    set_sensor_offsets(&mut i2c);

    // Run DMP initialization
    dmp_initialize(&mut i2c);

    rprintln!("Entering quaternion read loop...");

    let mut packet = [0u8; DMP_PACKET_SIZE];

    loop {
        // Read FIFO count (big-endian u16 from registers 0x72-0x73)
        let mut fifo_count_buf = [0u8; 2];
        i2c.write_read(MPU_ADDR, &[FIFO_COUNT_H], &mut fifo_count_buf)
            .unwrap();
        let fifo_count = u16::from_be_bytes(fifo_count_buf);

        if fifo_count > 200 {
            // FIFO overflow — reset it
            rprintln!("FIFO overflow ({}), resetting...", fifo_count);
            write_bit!(i2c, USER_CTRL, 2, true);
            cortex_m::asm::delay(8_000); // ~1ms
            continue;
        }

        if fifo_count >= DMP_PACKET_SIZE as u16 {
            // Drain old packets, keep only the latest
            let mut count = fifo_count;
            while count >= 2 * DMP_PACKET_SIZE as u16 {
                // Discard one packet
                i2c.write_read(MPU_ADDR, &[FIFO_R_W], &mut packet).unwrap();
                count -= DMP_PACKET_SIZE as u16;
            }

            // Read the latest packet
            i2c.write_read(MPU_ADDR, &[FIFO_R_W], &mut packet).unwrap();

            let cycles_per_us = clocks.sysclk().0 / 1_000_000;
            let total_us = DWT::cycle_count() / cycles_per_us;
            let seconds = total_us / 1_000_000;
            let micros = total_us % 1_000_000;

            // Extract quaternion from DMP packet:
            // w = bytes[0:1], x = bytes[4:5], y = bytes[8:9], z = bytes[12:13]
            // Big-endian i16, normalized by dividing by 16384.0
            let qw = i16::from_be_bytes([packet[0], packet[1]]) as f32 / 16384.0;
            let qx = i16::from_be_bytes([packet[4], packet[5]]) as f32 / 16384.0;
            let qy = i16::from_be_bytes([packet[8], packet[9]]) as f32 / 16384.0;
            let qz = i16::from_be_bytes([packet[12], packet[13]]) as f32 / 16384.0;
            rprintln!(
                "{}.{:06},{:.4},{:.4},{:.4},{:.4}",
                seconds,
                micros,
                qw,
                qx,
                qy,
                qz
            );
        }

        // Small delay to avoid excessive bus polling (~1ms)
        cortex_m::asm::delay(8_000);
    }
}
