use rtt_target::rprintln;
use stm32f3xx_hal::{
    gpio::{Alternate, Gpioa, Pin, PushPull, U},
    pac::USART2,
    prelude::*,
    serial::{config::Config, Serial},
};

type TxPinType = Pin<Gpioa, U<2>, Alternate<PushPull, 7>>;
type RxPinType = Pin<Gpioa, U<3>, Alternate<PushPull, 7>>;
type SerialType = Serial<USART2, (TxPinType, RxPinType)>;

pub fn read_gps_line(serial: &mut SerialType) -> Option<heapless::String<128>> {
    let mut line: heapless::Vec<u8, 128> = heapless::Vec::new(); // max 128 bytes per line

    loop {
        match nb::block!(serial.read()) {
            Ok(byte) => {
                line.push(byte).ok(); // ignore push errors if line is full

                if byte == b'\n' {
                    // End of NMEA sentence
                    if let Ok(string) = core::str::from_utf8(&line) {
                        let mut s: heapless::String<128> = heapless::String::new();
                        s.push_str(string).ok();
                        return Some(s);
                    } else {
                        line.clear(); // discard invalid UTF-8
                        return None;
                    }
                }
            }
            Err(_err) => {
                // Ignore read errors (like WouldBlock)
            }
        }
    }
}
