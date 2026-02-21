#!/bin/bash
# Live BLE skeleton visualization
# Usage: ./scripts/run_live.sh
# Requires: sudo (for BLE raw HCI access)

ADDR1="E3:C3:6B:90:28:B7"  # channel 0 → ShoulderRight
ADDR2="E0:7B:5E:3E:D1:48"  # channel 1 → ElbowRight

sudo ./target/debug/ble-scan --multi --addr "$ADDR1" --addr "$ADDR2" \
  | WINIT_UNIX_BACKEND=x11 cargo run -p body_viz -- --live ShoulderRight ElbowRight
