#!/usr/bin/env bash
RUSTFLAGS="-C link-arg=-Tlink.x" cargo b -p sensors --release --target thumbv7em-none-eabihf 
probe-rs run --chip STM32F334C8 target/thumbv7em-none-eabihf/release/sensors
