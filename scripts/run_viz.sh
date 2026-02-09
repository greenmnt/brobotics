#!/bin/bash
# Force X11 backend — kiss3d's winit 0.24 doesn't support Wayland properly
WINIT_UNIX_BACKEND=x11 cargo run -p viz --release --target x86_64-unknown-linux-gnu


