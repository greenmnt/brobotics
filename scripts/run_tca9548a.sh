#!/usr/bin/env bash
NAME=tca9548a
RUSTFLAGS="-C link-arg=-Tlink.x" cargo b -p $NAME --release --target thumbv7em-none-eabihf 
probe-rs run --chip STM32F334C8 --no-timestamps target/thumbv7em-none-eabihf/release/$NAME "$@" | tee /tmp/imu_in



