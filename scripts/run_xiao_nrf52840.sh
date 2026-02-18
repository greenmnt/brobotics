#!/usr/bin/env bash
set -euo pipefail

NAME=xiao-nrf52840
DIR="$(cd "$(dirname "$0")/.." && pwd)"
ELF="$DIR/target/thumbv7em-none-eabihf/release/$NAME"
BIN="$ELF.bin"
UF2="$ELF.uf2"

# Build
cd "$DIR/$NAME"
cargo b --release

# ELF -> binary -> UF2 (nRF52840 family, app starts at 0x27000)
rust-objcopy -O binary "$ELF" "$BIN"
uf2conv "$BIN" --base 0x27000 --family 0xADA52840 --output "$UF2"

# Copy to mounted bootloader drive (double-tap reset first)
UF2_MOUNT=$(findmnt -rn -o TARGET -S LABEL=XIAO-SENSE 2>/dev/null || true)
if [ -z "$UF2_MOUNT" ]; then
    UF2_MOUNT=$(findmnt -rn -o TARGET -S LABEL="XIAO BLE" 2>/dev/null || true)
fi

if [ -n "$UF2_MOUNT" ]; then
    cp "$UF2" "$UF2_MOUNT"/
    sync
    echo "Flashed to $UF2_MOUNT — board will reset automatically."
else
    echo "UF2 ready: $UF2"
    echo "Double-tap reset on the XIAO, then copy the .uf2 to the mounted drive."
fi
