#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use mpu6050_firmware::Mpu6050;
use nrf52840_hal::{self as hal, gpio::Level, pac::Peripherals, twim, Twim};
use panic_halt as _;

// ---------------------------------------------------------------------------
// BLE advertising
// ---------------------------------------------------------------------------

const BLE_ADV_CHANNELS: [(u8, u32); 3] = [
    (37, 2),  // 2402 MHz
    (38, 26), // 2426 MHz
    (39, 80), // 2480 MHz
];

/// Build a BLE advertising PDU with name + manufacturer data (timestamp + quaternion).
/// Manufacturer data layout: [timestamp_u32_BE, w_i16_BE, x_i16_BE, y_i16_BE, z_i16_BE]
/// Returns (total_length, mfr_data_offset) where offset points to the timestamp byte.
fn build_adv_packet(buf: &mut [u8; 64]) -> (usize, usize) {
    // Read unique device address from factory FICR registers
    let ficr = unsafe { &*nrf52840_hal::pac::FICR::ptr() };
    let a0 = ficr.deviceaddr[0].read().bits();
    let a1 = ficr.deviceaddr[1].read().bits();
    let mut addr = [
        (a0 & 0xFF) as u8,
        ((a0 >> 8) & 0xFF) as u8,
        ((a0 >> 16) & 0xFF) as u8,
        ((a0 >> 24) & 0xFF) as u8,
        (a1 & 0xFF) as u8,
        ((a1 >> 8) & 0xFF) as u8,
    ];
    // Set two MSBs of the top byte to mark as random static address (BLE requirement)
    addr[5] |= 0xC0;
    let name = b"XIAO-Rust";

    let mut i = 0;

    // PDU header
    buf[i] = 0x42; // ADV_NONCONN_IND, TxAdd=1 (random)
    i += 1;
    let len_idx = i;
    buf[i] = 0; // placeholder
    i += 1;

    // AdvA (6 bytes)
    buf[i..i + 6].copy_from_slice(&addr);
    i += 6;
    let payload_start = i;

    // AD: Flags (3 bytes)
    buf[i] = 0x02; i += 1;
    buf[i] = 0x01; i += 1;
    buf[i] = 0x06; i += 1;

    // AD: Complete Local Name (11 bytes)
    buf[i] = (name.len() as u8) + 1; i += 1;
    buf[i] = 0x09; i += 1;
    buf[i..i + name.len()].copy_from_slice(name);
    i += name.len();

    // AD: Manufacturer Specific Data
    // Length = 1(type) + 2(company) + 4(timestamp) + 8(quaternion) = 15
    buf[i] = 0x0F; i += 1;
    buf[i] = 0xFF; i += 1; // AD type
    buf[i] = 0xFF; i += 1; // company ID low
    buf[i] = 0xFF; i += 1; // company ID high
    let mfr_data_offset = i;
    // Timestamp (4 bytes) + quaternion (8 bytes) = 12 bytes, zeroed
    for j in 0..12 {
        buf[i + j] = 0;
    }
    // Set identity quaternion: w=16384 at offset +4
    buf[i + 4] = 0x40;
    buf[i + 5] = 0x00;
    i += 12;

    // Total AD: 3 + 11 + 16 = 30 bytes (under 31 limit)
    let payload_len = i - payload_start;
    buf[len_idx] = (6 + payload_len) as u8;

    (i, mfr_data_offset)
}

/// Encode timestamp (ms) and quaternion into the packet buffer at mfr_data_offset.
/// Layout: [ts_u32_BE (4 bytes), w_i16_BE, x_i16_BE, y_i16_BE, z_i16_BE (8 bytes)]
fn encode_payload(buf: &mut [u8; 64], offset: usize, timestamp_ms: u32, w: f32, x: f32, y: f32, z: f32) {
    // Timestamp
    let ts = timestamp_ms.to_be_bytes();
    buf[offset] = ts[0];
    buf[offset + 1] = ts[1];
    buf[offset + 2] = ts[2];
    buf[offset + 3] = ts[3];

    // Quaternion (Q14)
    for (j, val) in [w, x, y, z].iter().enumerate() {
        let clamped = val.max(-1.0).min(1.0);
        let ival = (clamped * 16384.0) as i16;
        let bytes = ival.to_be_bytes();
        buf[offset + 4 + j * 2] = bytes[0];
        buf[offset + 4 + j * 2 + 1] = bytes[1];
    }
}

fn radio_init(radio: &nrf52840_hal::pac::RADIO) {
    radio.mode.write(|w| w.mode().ble_1mbit());

    radio.pcnf0.write(|w| unsafe {
        w.lflen().bits(8)
         .s0len().bit(true)
         .s1len().bits(0)
         .plen()._8bit()
    });

    radio.pcnf1.write(|w| unsafe {
        w.maxlen().bits(37)
         .statlen().bits(0)
         .balen().bits(3)
         .endian().little()
         .whiteen().enabled()
    });

    radio.base0.write(|w| unsafe { w.bits(0x89BED600) });
    radio.prefix0.write(|w| unsafe { w.bits(0x0000008E) });
    radio.txaddress.write(|w| unsafe { w.txaddress().bits(0) });

    radio.crccnf.write(|w| w.len().three().skipaddr().skip());
    radio.crcpoly.write(|w| unsafe { w.crcpoly().bits(0x0000065B) });
    radio.crcinit.write(|w| unsafe { w.crcinit().bits(0x00555555) });

    radio.txpower.write(|w| w.txpower()._0d_bm());
}

fn radio_send(radio: &nrf52840_hal::pac::RADIO, buf: &[u8; 64], channel: u8, freq: u32) {
    radio.frequency.write(|w| unsafe { w.frequency().bits(freq as u8) });
    radio.datawhiteiv.write(|w| unsafe { w.datawhiteiv().bits(channel & 0x3F) });
    radio.packetptr.write(|w| unsafe { w.bits(buf.as_ptr() as u32) });

    radio.shorts.write(|w| {
        w.ready_start().enabled().end_disable().enabled()
    });

    radio.events_disabled.reset();
    radio.events_ready.reset();
    radio.events_end.reset();

    radio.tasks_txen.write(|w| unsafe { w.bits(1) });

    while radio.events_disabled.read().bits() == 0 {}
}

// ---------------------------------------------------------------------------
// LED diagnostic helper
// ---------------------------------------------------------------------------

fn blink<P: OutputPin>(led: &mut P, timer: &mut hal::delay::Delay, n: u32, on_ms: u32, off_ms: u32) {
    for _ in 0..n {
        let _ = led.set_low();  // on (active LOW)
        timer.delay_ms(on_ms);
        let _ = led.set_high(); // off
        timer.delay_ms(off_ms);
    }
}

// ---------------------------------------------------------------------------
// DWT cycle counter → milliseconds
// ---------------------------------------------------------------------------

/// Enable the DWT cycle counter.
fn dwt_enable(cp: &mut cortex_m::Peripherals) {
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

/// Read DWT cycle count and convert to milliseconds (nRF52840 runs at 64 MHz).
fn millis_since(start_cycles: u32) -> u32 {
    let now = cortex_m::peripheral::DWT::cycle_count();
    now.wrapping_sub(start_cycles) / 64_000
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = hal::pac::CorePeripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);

    let mut led_red = port0.p0_26.into_push_pull_output(Level::High);
    let mut led_green = port0.p0_30.into_push_pull_output(Level::High);
    let mut led_blue = port0.p0_06.into_push_pull_output(Level::High);

    // Enable DWT cycle counter for timestamps
    dwt_enable(&mut cp);
    let start_cycles = cortex_m::peripheral::DWT::cycle_count();

    let mut timer = hal::delay::Delay::new(cp.SYST);

    // ===== STAGE 1: Entry — 3x RED =====
    blink(&mut led_red, &mut timer, 3, 300, 300);
    timer.delay_ms(500u32);

    // ===== STAGE 2: I2C init on external header pins =====
    // XIAO nRF52840 header: D4=P0.04 (SDA), D5=P0.05 (SCL)
    let scl = port0.p0_05.into_floating_input().degrade();
    let sda = port0.p0_04.into_floating_input().degrade();
    let twim_pins = twim::Pins { scl, sda };
    let mut i2c = Twim::new(p.TWIM0, twim_pins, twim::Frequency::K400);
    blink(&mut led_green, &mut timer, 3, 300, 300);
    timer.delay_ms(500u32);

    // ===== STAGE 3: MPU6050 DMP init =====
    let mpu = Mpu6050::new(0x68);

    match mpu.dmp_initialize(&mut i2c, |ms| timer.delay_ms(ms)) {
        Ok(()) => {
            for _ in 0..3 {
                led_green.set_low().unwrap();
                led_blue.set_low().unwrap();
                timer.delay_ms(300u32);
                led_green.set_high().unwrap();
                led_blue.set_high().unwrap();
                timer.delay_ms(300u32);
            }
        }
        Err(_) => {
            loop {
                blink(&mut led_red, &mut timer, 1, 500, 500);
            }
        }
    }
    timer.delay_ms(500u32);

    // ===== Post-DMP: 3x WHITE =====
    for _ in 0..3 {
        led_red.set_low().unwrap();
        led_green.set_low().unwrap();
        led_blue.set_low().unwrap();
        timer.delay_ms(300u32);
        led_red.set_high().unwrap();
        led_green.set_high().unwrap();
        led_blue.set_high().unwrap();
        timer.delay_ms(300u32);
    }
    timer.delay_ms(500u32);

    // ===== STAGE 4: HF clock + RADIO =====
    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while p.CLOCK.events_hfclkstarted.read().bits() == 0 {}
    radio_init(&p.RADIO);

    for _ in 0..3 {
        led_red.set_low().unwrap();
        led_blue.set_low().unwrap();
        timer.delay_ms(300u32);
        led_red.set_high().unwrap();
        led_blue.set_high().unwrap();
        timer.delay_ms(300u32);
    }
    timer.delay_ms(500u32);

    // ===== Build BLE packet =====
    let mut pkt_buf = [0u8; 64];
    let (_pkt_len, mfr_offset) = build_adv_packet(&mut pkt_buf);

    // All 3 LEDs on briefly = entering main loop
    led_red.set_low().unwrap();
    led_green.set_low().unwrap();
    led_blue.set_low().unwrap();
    timer.delay_ms(500u32);
    led_red.set_high().unwrap();
    led_green.set_high().unwrap();
    led_blue.set_high().unwrap();
    timer.delay_ms(300u32);

    let mut led_counter: u32 = 0;
    let mut led_on = false;

    // ===== Main loop: read DMP quaternion → BLE broadcast =====
    loop {
        match mpu.read_dmp_quaternion(&mut i2c) {
            Ok(Some(q)) => {
                let ts = millis_since(start_cycles);
                encode_payload(&mut pkt_buf, mfr_offset, ts, q.w, q.x, q.y, q.z);

                for &(ch, freq) in &BLE_ADV_CHANNELS {
                    radio_send(&p.RADIO, &pkt_buf, ch, freq);
                }

                led_counter += 1;
                if led_counter >= 50 {
                    led_counter = 0;
                    if led_on {
                        led_blue.set_high().unwrap();
                    } else {
                        led_blue.set_low().unwrap();
                    }
                    led_on = !led_on;
                }
            }
            Ok(None) => {
                cortex_m::asm::delay(64_000); // ~1ms at 64MHz
            }
            Err(_) => {
                blink(&mut led_red, &mut timer, 1, 50, 50);
            }
        }
    }
}
