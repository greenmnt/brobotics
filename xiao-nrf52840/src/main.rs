#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use nrf52840_hal::{self as hal, gpio::Level, pac::Peripherals};
use panic_halt as _;

/// BLE advertising channel frequencies (indices 37, 38, 39).
/// The RADIO FREQUENCY register expects: freq_mhz = 2400 + value.
const BLE_ADV_CHANNELS: [(u8, u32); 3] = [
    (37, 2),  // 2402 MHz
    (38, 26), // 2426 MHz
    (39, 80), // 2480 MHz
];

/// Build a BLE advertising PDU (ADV_NONCONN_IND).
/// Returns (buffer, total_length).
fn build_adv_packet(buf: &mut [u8; 64]) -> usize {
    // Random static address (top 2 bits = 11)
    let addr: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xDE];

    // AD fields
    let name = b"XIAO-Rust";

    // AD: Flags (type 0x01): LE General Discoverable + BR/EDR Not Supported
    let flags_ad: [u8; 3] = [0x02, 0x01, 0x06];
    // AD: Complete Local Name (type 0x09)
    let name_len = name.len() as u8;
    // name AD = [len+1, 0x09, name...]

    let payload_len = flags_ad.len() + 2 + name.len();
    // PDU header: ADV_NONCONN_IND type = 0x02, TxAdd=1 (random address)
    // Header byte 0: PDU type (4 bits) | RFU (1) | ChSel (1) | TxAdd (1) | RxAdd (1)
    // ADV_NONCONN_IND = 0b0010, TxAdd=1 → 0b0100_0010 = 0x42
    let pdu_type: u8 = 0x42;
    let pdu_length: u8 = 6 + payload_len as u8; // 6 = address

    let mut i = 0;
    // S0 (1 byte): PDU type
    buf[i] = pdu_type; i += 1;
    // LENGTH (1 byte): PDU payload length (address + AD data)
    buf[i] = pdu_length; i += 1;
    // AdvA: advertiser address (little-endian, 6 bytes)
    buf[i..i+6].copy_from_slice(&addr); i += 6;
    // AD: Flags
    buf[i..i+3].copy_from_slice(&flags_ad); i += 3;
    // AD: Complete Local Name
    buf[i] = name_len + 1; i += 1;
    buf[i] = 0x09; i += 1;
    buf[i..i+name.len()].copy_from_slice(name); i += name.len();

    i
}

/// Configure the RADIO peripheral for BLE 1M PHY advertising.
fn radio_init(radio: &nrf52840_hal::pac::RADIO) {
    // BLE 1Mbps mode
    radio.mode.write(|w| w.mode().ble_1mbit());

    // Packet config for BLE:
    // S0 = 1 byte, LENGTH = 8 bits, S1 = 0, PLEN = 8 bits
    radio.pcnf0.write(|w| unsafe {
        w.lflen().bits(8)      // LENGTH field = 8 bits
         .s0len().bit(true)    // S0 = 1 byte
         .s1len().bits(0)      // S1 = 0
         .plen()._8bit()       // Preamble = 8 bits (BLE 1M)
    });

    // PCNF1: max payload, base address length, whitening, endianness
    radio.pcnf1.write(|w| unsafe {
        w.maxlen().bits(37)       // max PDU length for advertising
         .statlen().bits(0)
         .balen().bits(3)         // 3 bytes base address (+ 1 prefix = 4 total)
         .endian().little()
         .whiteen().enabled()     // BLE uses whitening
    });

    // BLE advertising access address: 0x8E89BED6
    // Base address = lower bytes, prefix = top byte
    // nRF RADIO: access address = PREFIX[n] ++ BASE[n]
    // For BLE: AA = 0x8E89BED6
    // BASE0 = 0x89BED600 (shifted left by 8), PREFIX0.AP0 = 0x8E
    radio.base0.write(|w| unsafe { w.bits(0x89BED600) });
    radio.prefix0.write(|w| unsafe { w.bits(0x0000008E) });

    // Use logical address 0
    radio.txaddress.write(|w| unsafe { w.txaddress().bits(0) });

    // CRC config for BLE: 3 bytes, skip address, init = 0x555555
    radio.crccnf.write(|w| {
        w.len().three()
         .skipaddr().skip()
    });
    radio.crcpoly.write(|w| unsafe { w.crcpoly().bits(0x0000065B) });
    radio.crcinit.write(|w| unsafe { w.crcinit().bits(0x00555555) });

    // TX power: 0 dBm
    radio.txpower.write(|w| w.txpower()._0d_bm());
}

/// Transmit one advertising packet on a given channel.
fn radio_send(radio: &nrf52840_hal::pac::RADIO, buf: &[u8; 64], channel: u8, freq: u32) {
    // Set frequency
    radio.frequency.write(|w| unsafe { w.frequency().bits(freq as u8) });

    // Set data whitening IV = channel index
    radio.datawhiteiv.write(|w| unsafe { w.datawhiteiv().bits(channel & 0x3F) });

    // Set packet pointer
    radio.packetptr.write(|w| unsafe { w.bits(buf.as_ptr() as u32) });

    // Shortcuts: READY -> START, END -> DISABLE
    radio.shorts.write(|w| {
        w.ready_start().enabled()
         .end_disable().enabled()
    });

    // Clear events
    radio.events_disabled.reset();
    radio.events_ready.reset();
    radio.events_end.reset();

    // Enable TX
    radio.tasks_txen.write(|w| unsafe { w.bits(1) });

    // Wait for DISABLED event (transmission complete)
    while radio.events_disabled.read().bits() == 0 {}
}

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);

    // XIAO nRF52840 LEDs (active LOW)
    let mut led_blue = port0.p0_06.into_push_pull_output(Level::High);

    let mut timer = hal::delay::Delay::new(hal::pac::CorePeripherals::take().unwrap().SYST);

    // Start the HF clock (required for RADIO)
    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while p.CLOCK.events_hfclkstarted.read().bits() == 0 {}

    // Configure RADIO for BLE advertising
    radio_init(&p.RADIO);

    // Build the advertising packet
    let mut pkt_buf = [0u8; 64];
    build_adv_packet(&mut pkt_buf);

    loop {
        // Advertise on all 3 channels
        for &(ch, freq) in &BLE_ADV_CHANNELS {
            radio_send(&p.RADIO, &pkt_buf, ch, freq);
        }

        // Toggle blue LED
        led_blue.set_low().unwrap();  // on
        timer.delay_ms(100u32);
        led_blue.set_high().unwrap(); // off

        // Advertising interval ~200ms
        timer.delay_ms(100u32);
    }
}
