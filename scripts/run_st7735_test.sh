#!/usr/bin/env bash
set -euo pipefail

NAME=st7735-test
RUSTFLAGS="-C link-arg=-Tlink.x" cargo b -p $NAME --release --target thumbv7em-none-eabihf
probe-rs run --chip STM32F334C8 target/thumbv7em-none-eabihf/release/$NAME "$@"
