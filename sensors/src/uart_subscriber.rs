use core::fmt::Write;
use log::{info, Level, Log, Metadata, Record, SetLoggerError};
use stm32f3xx_hal::{
    gpio::{Alternate, Gpioa, Pin, PushPull, U},
    pac::USART2,
    prelude::*,
    serial::{config::Config, Serial},
};

pub struct UARTLogger<T: embedded_hal::serial::Write<u8>> {
    serial: T,
}

impl<T: embedded_hal::serial::Write<u8>> UARTLogger<T> {
    pub const fn new(serial: T) -> Self {
        UARTLogger { serial }
    }

    fn write_byte(&mut self, byte: u8) {
        // block until written
        nb::block!(self.serial.write(byte)).ok();
    }

    fn write_str(&mut self, s: &str) {
        for b in s.bytes() {
            self.write_byte(b);
        }
    }
    pub fn log(&mut self, s: &str) {
        for b in s.bytes() {
            nb::block!(self.serial.write(b)).ok();
        }
        nb::block!(self.serial.write(b'\n')).ok();
    }
}

type TxPinType = Pin<Gpioa, U<2>, Alternate<PushPull, 7>>;
type RxPinType = Pin<Gpioa, U<3>, Alternate<PushPull, 7>>;
type SerialType = Serial<USART2, (TxPinType, RxPinType)>;
pub static mut LOGGER: Option<UARTLogger<SerialType>> = None;

//impl<T: embedded_hal::serial::Write<u8>> log::Log for UARTLogger<T> {
//fn enabled(&self, _: &Metadata) -> bool {
//true
//}

//fn log(&self, record: &Record) {
//// Because we have &self (not &mut self), we need unsafe to access a static mut
//unsafe {
//if let Some(ref mut logger) = LOGGER {
//logger.write_str(record.args().to_string().as_str());
//logger.write_byte(b'\n');
//}
//}
//}

//fn flush(&self) {}
//}

//pub static mut LOGGER: Option<UARTLogger<YourSerialType>> = None;
/*

tx_pin => stm32f3xx_hal::gpio::Pin<Gpioa, U<2>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>
rx_pin => stm32f3xx_hal::gpio::Pin<Gpioa, U<3>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>
 - from the source code

pub struct Serial<Usart, Pins> {
    usart: Usart,
    pins: Pins,
}


serial => Serial<Usart, Pins> => Serial<Usart, (TxPin, RxPin)>
Serial<USART2, (Pin<Gpioa, U<2>, Alternate<PushPull, 7>>, Pin<Gpioa, U<3>, ...>)>`

U is defined in stm32f3xx_hal::gpio::U

*/
