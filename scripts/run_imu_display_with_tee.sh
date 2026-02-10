#!/usr/bin/env bash
set -e

set -euo pipefail

# Start socat in background
socat PTY,raw,echo=0,link=/tmp/imu_in PTY,raw,echo=0,link=/tmp/imu_out &
SOCAT_PID=$!

# Kill socat when runner exits
trap "kill $SOCAT_PID" EXIT
NAME=imu-display
RUSTFLAGS="-C link-arg=-Tlink.x" cargo b -p $NAME --release --target thumbv7em-none-eabihf
probe-rs run --chip STM32F334C8 --no-timestamps target/thumbv7em-none-eabihf/release/$NAME "$@" | tee /tmp/imu_in

