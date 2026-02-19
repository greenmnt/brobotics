#![no_std]

mod dmp_firmware;

pub use dmp_firmware::{DMP_FIRMWARE, DMP_PACKET_SIZE};

use embedded_hal::blocking::i2c::{Write, WriteRead};

// ---------------------------------------------------------------------------
// MPU6050 register addresses
// ---------------------------------------------------------------------------

pub const MPU6050_DEFAULT_ADDR: u8 = 0x68;

pub const REG_XG_OFFS_TC: u8 = 0x00;
pub const REG_ZA_OFFS_H: u8 = 0x0A;
pub const REG_XG_OFFS_USRH: u8 = 0x13;
pub const REG_YG_OFFS_USRH: u8 = 0x15;
pub const REG_ZG_OFFS_USRH: u8 = 0x17;
pub const REG_SMPLRT_DIV: u8 = 0x19;
pub const REG_CONFIG: u8 = 0x1A;
pub const REG_GYRO_CONFIG: u8 = 0x1B;
pub const REG_MOT_THR: u8 = 0x1F;
pub const REG_MOT_DUR: u8 = 0x20;
pub const REG_ZRMOT_THR: u8 = 0x21;
pub const REG_ZRMOT_DUR: u8 = 0x22;
pub const REG_I2C_SLV0_ADDR: u8 = 0x25;
pub const REG_INT_ENABLE: u8 = 0x38;
pub const REG_INT_STATUS: u8 = 0x3A;
pub const REG_USER_CTRL: u8 = 0x6A;
pub const REG_PWR_MGMT_1: u8 = 0x6B;
pub const REG_BANK_SEL: u8 = 0x6D;
pub const REG_MEM_START_ADDR: u8 = 0x6E;
pub const REG_MEM_R_W: u8 = 0x6F;
pub const REG_DMP_CFG_1: u8 = 0x70;
pub const REG_DMP_CFG_2: u8 = 0x71;
pub const REG_FIFO_COUNT_H: u8 = 0x72;
pub const REG_FIFO_R_W: u8 = 0x74;
pub const REG_WHO_AM_I: u8 = 0x75;

// ---------------------------------------------------------------------------
// Quaternion output
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// ---------------------------------------------------------------------------
// MPU6050 driver
//
// The struct stores only the I2C address so the bus can be shared between
// multiple devices (e.g. via a TCA9548A mux). All methods take `&mut I`
// so the caller retains ownership of the bus.
// ---------------------------------------------------------------------------

pub struct Mpu6050 {
    addr: u8,
}

impl Mpu6050 {
    pub const fn new(addr: u8) -> Self {
        Self { addr }
    }

    // -----------------------------------------------------------------------
    // Register-level I/O
    // -----------------------------------------------------------------------

    /// Write a single register.
    pub fn write_reg<I, E>(&self, i2c: &mut I, reg: u8, val: u8) -> Result<(), E>
    where
        I: Write<Error = E>,
    {
        i2c.write(self.addr, &[reg, val])
    }

    /// Read a single register.
    pub fn read_reg<I, E>(&self, i2c: &mut I, reg: u8) -> Result<u8, E>
    where
        I: WriteRead<Error = E>,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Read-modify-write a single bit in a register.
    pub fn write_bit<I, E>(&self, i2c: &mut I, reg: u8, bit: u8, val: bool) -> Result<(), E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        let mut b = self.read_reg(i2c, reg)?;
        if val {
            b |= 1 << bit;
        } else {
            b &= !(1 << bit);
        }
        self.write_reg(i2c, reg, b)
    }

    /// Read-modify-write a bit field in a register.
    /// `start_bit` is the MSB position, `len` is the number of bits
    /// (same convention as i2cdevlib).
    pub fn write_bits<I, E>(
        &self,
        i2c: &mut I,
        reg: u8,
        start_bit: u8,
        len: u8,
        val: u8,
    ) -> Result<(), E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        let mut b = self.read_reg(i2c, reg)?;
        let shift = start_bit - len + 1;
        let mask: u8 = ((1u16 << len) - 1) as u8;
        b &= !(mask << shift);
        b |= (val & mask) << shift;
        self.write_reg(i2c, reg, b)
    }

    /// Write a 16-bit value to consecutive registers (MSB first).
    pub fn write_word<I, E>(&self, i2c: &mut I, reg: u8, val: i16) -> Result<(), E>
    where
        I: Write<Error = E>,
    {
        let v = val as u16;
        i2c.write(self.addr, &[reg, (v >> 8) as u8, v as u8])
    }

    /// Read a signed 16-bit value from consecutive registers (MSB first).
    pub fn read_i16<I, E>(&self, i2c: &mut I, reg: u8) -> Result<i16, E>
    where
        I: WriteRead<Error = E>,
    {
        let mut buf = [0u8; 2];
        i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(((buf[0] as i16) << 8) | (buf[1] as i16))
    }

    // -----------------------------------------------------------------------
    // DMP firmware upload
    // -----------------------------------------------------------------------

    /// Upload the DMP firmware blob to MPU6050 memory banks.
    /// Writes in 16-byte chunks via BANK_SEL / MEM_START_ADDR / MEM_R_W.
    fn upload_dmp_firmware<I, E>(&self, i2c: &mut I) -> Result<(), E>
    where
        I: Write<Error = E>,
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
            i2c.write(self.addr, &[REG_BANK_SEL, bank])?;
            i2c.write(self.addr, &[REG_MEM_START_ADDR, mem_addr])?;

            // Build write buffer: register byte + data
            let mut buf = [0u8; 17]; // 1 register + max 16 data
            buf[0] = REG_MEM_R_W;
            buf[1..1 + chunk_size]
                .copy_from_slice(&DMP_FIRMWARE[src_offset..src_offset + chunk_size]);
            i2c.write(self.addr, &buf[..1 + chunk_size])?;

            addr += chunk_size as u16;
            src_offset += chunk_size;
            remaining -= chunk_size;
        }

        Ok(())
    }

    /// Write data to a specific DMP memory bank/offset.
    fn write_dmp_memory<I, E>(
        &self,
        i2c: &mut I,
        bank: u8,
        offset: u8,
        data: &[u8],
    ) -> Result<(), E>
    where
        I: Write<Error = E>,
    {
        i2c.write(self.addr, &[REG_BANK_SEL, bank])?;
        i2c.write(self.addr, &[REG_MEM_START_ADDR, offset])?;
        let mut buf = [0u8; 17];
        buf[0] = REG_MEM_R_W;
        let len = data.len().min(16);
        buf[1..1 + len].copy_from_slice(&data[..len]);
        i2c.write(self.addr, &buf[..1 + len])?;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Sensor configuration
    // -----------------------------------------------------------------------

    /// Set gyroscope user offsets (X, Y, Z).
    pub fn set_gyro_offsets<I, E>(
        &self,
        i2c: &mut I,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), E>
    where
        I: Write<Error = E>,
    {
        self.write_word(i2c, REG_XG_OFFS_USRH, x)?;
        self.write_word(i2c, REG_YG_OFFS_USRH, y)?;
        self.write_word(i2c, REG_ZG_OFFS_USRH, z)?;
        Ok(())
    }

    /// Set accelerometer Z-axis offset.
    /// Automatically selects the correct register based on device ID
    /// (0x0A for MPU6050/MPU9150, 0x7D for MPU6500/MPU9250).
    pub fn set_accel_z_offset<I, E>(&self, i2c: &mut I, offset: i16) -> Result<(), E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        let dev_id = self.read_reg(i2c, REG_WHO_AM_I)?;
        let reg = if dev_id < 0x38 {
            REG_ZA_OFFS_H // MPU6050 / MPU9150
        } else {
            0x7D // MPU6500 / MPU9250
        };
        self.write_word(i2c, reg, offset)
    }

    // -----------------------------------------------------------------------
    // DMP initialization (mirrors dmpInitialize() from C++ i2cdevlib)
    // -----------------------------------------------------------------------

    /// Perform the full 25-step DMP initialization sequence.
    ///
    /// `delay_ms` is a closure that delays for the given number of milliseconds.
    /// Example: `|ms| cortex_m::asm::delay(8_000 * ms)`
    pub fn dmp_initialize<I, E>(
        &self,
        i2c: &mut I,
        mut delay_ms: impl FnMut(u32),
    ) -> Result<(), E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        // 1. Reset device (bit 7 of PWR_MGMT_1)
        self.write_bit(i2c, REG_PWR_MGMT_1, 7, true)?;
        delay_ms(30);

        // 2. Disable sleep (bit 6 of PWR_MGMT_1)
        self.write_bit(i2c, REG_PWR_MGMT_1, 6, false)?;

        // 3. Hardware revision check: set memory bank 0x10 with userBank and prefetch
        //    setMemoryBank(0x10, true, true) => bank = 0x10 | 0x20 | 0x40 = 0x70
        i2c.write(self.addr, &[REG_BANK_SEL, 0x70])?;
        i2c.write(self.addr, &[REG_MEM_START_ADDR, 0x06])?;
        let _hw_rev = self.read_reg(i2c, REG_MEM_R_W)?;

        // 4. Reset BANK_SEL to 0
        i2c.write(self.addr, &[REG_BANK_SEL, 0x00])?;

        // 5. Set I2C_SLV0_ADDR to 0x7F
        self.write_reg(i2c, REG_I2C_SLV0_ADDR, 0x7F)?;

        // 6. Disable I2C master mode: clear bit 5 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 5, false)?;

        // 7. Set I2C_SLV0_ADDR to 0x68 (self)
        self.write_reg(i2c, REG_I2C_SLV0_ADDR, 0x68)?;

        // 8. Reset I2C master: set bit 1 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 1, true)?;
        delay_ms(20);

        // 9. Set clock source to PLL_ZGYRO: bits [2:0] of PWR_MGMT_1 = 0x03
        self.write_bits(i2c, REG_PWR_MGMT_1, 2, 3, 0x03)?;

        // 10. Enable DMP and FIFO overflow interrupts: INT_ENABLE = 0x12
        self.write_reg(i2c, REG_INT_ENABLE, 0x12)?;

        // 11. Sample rate divider: SMPLRT_DIV = 4 (200 Hz)
        self.write_reg(i2c, REG_SMPLRT_DIV, 4)?;

        // 12. External frame sync: CONFIG bits [5:3] = 1
        self.write_bits(i2c, REG_CONFIG, 5, 3, 0x01)?;

        // 13. DLPF mode: CONFIG bits [2:0] = 3 (42 Hz)
        self.write_bits(i2c, REG_CONFIG, 2, 3, 0x03)?;

        // 14. Gyro full-scale range: GYRO_CONFIG bits [4:3] = 3 (+-2000 deg/s)
        self.write_bits(i2c, REG_GYRO_CONFIG, 4, 2, 0x03)?;

        // 15. Upload DMP firmware (1929 bytes)
        self.upload_dmp_firmware(i2c)?;

        // 16. Write FIFO rate divisor to DMP memory: bank 0x02, offset 0x16
        //     [0x00, 0x00] = every sample = 200 Hz
        self.write_dmp_memory(i2c, 0x02, 0x16, &[0x00, 0x00])?;

        // 17. Set DMP config registers
        self.write_reg(i2c, REG_DMP_CFG_1, 0x03)?;
        self.write_reg(i2c, REG_DMP_CFG_2, 0x00)?;

        // 18. Clear OTP bank valid: clear bit 0 of XG_OFFS_TC
        self.write_bit(i2c, REG_XG_OFFS_TC, 0, false)?;

        // 19. Motion detection thresholds
        self.write_reg(i2c, REG_MOT_THR, 2)?;
        self.write_reg(i2c, REG_ZRMOT_THR, 156)?;
        self.write_reg(i2c, REG_MOT_DUR, 80)?;
        self.write_reg(i2c, REG_ZRMOT_DUR, 0)?;

        // 20. Enable FIFO: set bit 6 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 6, true)?;

        // 21. Reset DMP: set bit 3 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 3, true)?;

        // 22. Disable DMP: clear bit 7 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 7, false)?;

        // 23. Reset FIFO: set bit 2 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 2, true)?;

        // 24. Clear interrupts by reading INT_STATUS
        let _ = self.read_reg(i2c, REG_INT_STATUS)?;

        // 25. Enable DMP: set bit 7 of USER_CTRL
        self.write_bit(i2c, REG_USER_CTRL, 7, true)?;

        Ok(())
    }

    // -----------------------------------------------------------------------
    // FIFO operations
    // -----------------------------------------------------------------------

    /// Read the FIFO byte count (big-endian u16 from registers 0x72-0x73).
    pub fn read_fifo_count<I, E>(&self, i2c: &mut I) -> Result<u16, E>
    where
        I: WriteRead<Error = E>,
    {
        let mut buf = [0u8; 2];
        i2c.write_read(self.addr, &[REG_FIFO_COUNT_H], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Reset the FIFO (set bit 2 of USER_CTRL).
    pub fn reset_fifo<I, E>(&self, i2c: &mut I) -> Result<(), E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        self.write_bit(i2c, REG_USER_CTRL, 2, true)
    }

    /// Read bytes from the FIFO register into `buf`.
    pub fn read_fifo_bytes<I, E>(&self, i2c: &mut I, buf: &mut [u8]) -> Result<(), E>
    where
        I: WriteRead<Error = E>,
    {
        i2c.write_read(self.addr, &[REG_FIFO_R_W], buf)
    }

    // -----------------------------------------------------------------------
    // Quaternion extraction
    // -----------------------------------------------------------------------

    /// Parse a quaternion from a raw DMP packet.
    ///
    /// Quaternion components are at offsets [0:1], [4:5], [8:9], [12:13]
    /// as big-endian i16 values, normalised by dividing by 16384.0.
    pub fn parse_quaternion(packet: &[u8]) -> Quaternion {
        Quaternion {
            w: i16::from_be_bytes([packet[0], packet[1]]) as f32 / 16384.0,
            x: i16::from_be_bytes([packet[4], packet[5]]) as f32 / 16384.0,
            y: i16::from_be_bytes([packet[8], packet[9]]) as f32 / 16384.0,
            z: i16::from_be_bytes([packet[12], packet[13]]) as f32 / 16384.0,
        }
    }

    // -----------------------------------------------------------------------
    // High-level DMP quaternion reading
    // -----------------------------------------------------------------------

    /// Read the latest DMP quaternion from the FIFO.
    ///
    /// Returns `Ok(Some(q))` if a quaternion was read, `Ok(None)` if no
    /// packet is available or the FIFO overflowed (auto-reset).
    /// Drains old packets so only the most recent data is returned.
    pub fn read_dmp_quaternion<I, E>(&self, i2c: &mut I) -> Result<Option<Quaternion>, E>
    where
        I: Write<Error = E> + WriteRead<Error = E>,
    {
        let fifo_count = self.read_fifo_count(i2c)?;

        if fifo_count > 200 {
            // FIFO overflow — reset it
            self.reset_fifo(i2c)?;
            return Ok(None);
        }

        if fifo_count < DMP_PACKET_SIZE as u16 {
            return Ok(None);
        }

        // Drain old packets, keep only the latest
        let mut count = fifo_count;
        let mut packet = [0u8; DMP_PACKET_SIZE];
        while count >= 2 * DMP_PACKET_SIZE as u16 {
            self.read_fifo_bytes(i2c, &mut packet)?;
            count -= DMP_PACKET_SIZE as u16;
        }

        // Read the latest packet
        self.read_fifo_bytes(i2c, &mut packet)?;

        Ok(Some(Self::parse_quaternion(&packet)))
    }
}
