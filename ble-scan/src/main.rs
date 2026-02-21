//! Raw HCI BLE scanner — exclusive HCI access, bypasses BlueZ entirely.
//!
//! Takes exclusive control of hci0 via HCI_CHANNEL_USER, configures LE scanning
//! with duplicate filtering disabled, and receives every advertising packet.
//! BlueZ automatically reclaims the adapter when the scanner exits.
//!
//! Usage:
//!   sudo ./target/debug/ble-scan              # 5-column from any XIAO
//!   sudo ./target/debug/ble-scan --addr AA:BB:CC:DD:EE:FF    # 5-column from one sensor only
//!   sudo ./target/debug/ble-scan --multi      # 6-column: channel,timestamp,w,x,y,z
//!   sudo ./target/debug/ble-scan --multi --addr AA:BB:.. --addr 11:22:..
//!                                             # 6-column, pin channel 0/1 to addresses
//!
//! Requires CAP_NET_ADMIN (run with sudo, or use setcap).

use std::collections::HashMap;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::{mem, process, ptr};

static RUNNING: AtomicBool = AtomicBool::new(true);

// ---- HCI / Bluetooth constants ----

const AF_BLUETOOTH: i32 = 31;
const BTPROTO_HCI: i32 = 1;
const HCI_CHANNEL_USER: u16 = 1;

// ioctl: _IOW('H', 202, int)
const HCIDEVDOWN: libc::c_ulong = 0x400448CA;

const HCI_COMMAND_PKT: u8 = 0x01;
const HCI_EVENT_PKT: u8 = 0x04;
const HCI_EV_CMD_COMPLETE: u8 = 0x0E;
const HCI_EV_CMD_STATUS: u8 = 0x0F;
const HCI_EV_LE_META: u8 = 0x3E;
const LE_ADV_REPORT: u8 = 0x02;

const fn hci_opcode(ogf: u16, ocf: u16) -> u16 {
    (ogf << 10) | ocf
}
const HCI_RESET: u16 = hci_opcode(0x03, 0x0003);
const SET_EVENT_MASK: u16 = hci_opcode(0x03, 0x0001);
const LE_SET_SCAN_PARAMS: u16 = hci_opcode(0x08, 0x000B);
const LE_SET_SCAN_ENABLE: u16 = hci_opcode(0x08, 0x000C);

// ---- Low-level structs ----

#[repr(C)]
struct SockaddrHci {
    hci_family: u16,
    hci_dev: u16,
    hci_channel: u16,
}

// ---- Signal handler (no SA_RESTART so read() returns EINTR) ----

extern "C" fn on_sigint(_: libc::c_int) {
    RUNNING.store(false, Ordering::SeqCst);
}

// ---- HCI device control ----

/// Bring hci device down via ioctl (takes it from BlueZ).
fn hci_dev_down(dev_id: u16) -> io::Result<()> {
    unsafe {
        let fd = libc::socket(AF_BLUETOOTH, libc::SOCK_RAW | libc::SOCK_CLOEXEC, BTPROTO_HCI);
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let ret = libc::ioctl(fd, HCIDEVDOWN, dev_id as libc::c_int);
        let err = io::Error::last_os_error();
        libc::close(fd);
        if ret < 0 {
            return Err(err);
        }
    }
    Ok(())
}

/// Open HCI socket with exclusive user channel access.
/// The bind itself brings the device UP — no separate power-on needed.
fn hci_open_user(dev_id: u16) -> io::Result<i32> {
    unsafe {
        let fd = libc::socket(AF_BLUETOOTH, libc::SOCK_RAW | libc::SOCK_CLOEXEC, BTPROTO_HCI);
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let addr = SockaddrHci {
            hci_family: AF_BLUETOOTH as u16,
            hci_dev: dev_id,
            hci_channel: HCI_CHANNEL_USER,
        };
        if libc::bind(
            fd,
            &addr as *const _ as *const libc::sockaddr,
            mem::size_of::<SockaddrHci>() as libc::socklen_t,
        ) < 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }
        Ok(fd)
    }
}

// ---- HCI command helpers ----

fn hci_write_cmd(fd: i32, opcode: u16, params: &[u8]) -> io::Result<()> {
    let mut buf = [0u8; 260];
    let len = 4 + params.len();
    buf[0] = HCI_COMMAND_PKT;
    buf[1] = (opcode & 0xFF) as u8;
    buf[2] = (opcode >> 8) as u8;
    buf[3] = params.len() as u8;
    buf[4..4 + params.len()].copy_from_slice(params);
    let n = unsafe { libc::write(fd, buf.as_ptr() as *const libc::c_void, len) };
    if n < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// Send HCI command and wait for Command Complete. Returns status byte.
/// No socket filter needed — HCI_CHANNEL_USER delivers all events.
fn hci_send_req(fd: i32, opcode: u16, params: &[u8]) -> io::Result<u8> {
    hci_write_cmd(fd, opcode, params)?;

    let mut buf = [0u8; 260];
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);

    loop {
        if std::time::Instant::now() > deadline {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "HCI command timeout",
            ));
        }
        let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            let e = io::Error::last_os_error();
            if e.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(e);
        }
        let n = n as usize;
        if n < 1 || buf[0] != HCI_EVENT_PKT {
            continue;
        }

        if n >= 7 && buf[1] == HCI_EV_CMD_COMPLETE {
            let op = u16::from_le_bytes([buf[4], buf[5]]);
            if op == opcode {
                return Ok(buf[6]);
            }
        }
        if n >= 7 && buf[1] == HCI_EV_CMD_STATUS {
            let op = u16::from_le_bytes([buf[5], buf[6]]);
            if op == opcode {
                return Ok(buf[3]);
            }
        }
    }
}

/// Reset controller and configure LE scanning with no duplicate filtering.
fn hci_init_scanning(fd: i32) -> io::Result<()> {
    // Reset controller to known state
    eprint!("  HCI Reset...");
    let st = hci_send_req(fd, HCI_RESET, &[])?;
    if st != 0 {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("status 0x{:02X}", st),
        ));
    }
    eprintln!(" ok");

    // Set Event Mask: default + LE Meta Event (bit 61)
    // Default: 0x00001FFFFFFFFFFF → with bit 61: 0x20001FFFFFFFFFFF
    eprint!("  Set Event Mask...");
    let mask: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x20];
    let st = hci_send_req(fd, SET_EVENT_MASK, &mask)?;
    if st != 0 {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("status 0x{:02X}", st),
        ));
    }
    eprintln!(" ok");

    // LE Set Scan Parameters: passive, 10ms interval/window, accept all
    eprint!("  LE Scan Parameters...");
    let params: [u8; 7] = [
        0x00,       // passive scan
        0x10, 0x00, // interval: 10ms (16 * 0.625ms)
        0x10, 0x00, // window: 10ms
        0x00,       // own address type: public
        0x00,       // filter policy: accept all
    ];
    let st = hci_send_req(fd, LE_SET_SCAN_PARAMS, &params)?;
    if st != 0 {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("status 0x{:02X}", st),
        ));
    }
    eprintln!(" ok");

    // LE Set Scan Enable: on, NO duplicate filtering
    eprint!("  LE Scan Enable (no dedup)...");
    let st = hci_send_req(fd, LE_SET_SCAN_ENABLE, &[0x01, 0x00])?;
    if st != 0 {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("status 0x{:02X}", st),
        ));
    }
    eprintln!(" ok");

    Ok(())
}

// ---- AD data parsing ----

fn find_ad(data: &[u8], target_type: u8) -> Option<&[u8]> {
    let mut i = 0;
    while i + 1 < data.len() {
        let len = data[i] as usize;
        if len == 0 || i + 1 + len > data.len() {
            break;
        }
        if data[i + 1] == target_type {
            return Some(&data[i + 2..i + 1 + len]);
        }
        i += 1 + len;
    }
    None
}

fn has_xiao_name(ad: &[u8]) -> bool {
    find_ad(ad, 0x09) // Complete Local Name
        .map_or(false, |name| name.windows(4).any(|w| w == b"XIAO"))
}

/// Decode manufacturer-specific quaternion payload.
/// Layout after company ID (0xFFFF): [ts_u32_BE] [w_i16_BE] [x_i16_BE] [y_i16_BE] [z_i16_BE]
fn decode_mfr_quaternion(ad: &[u8]) -> Option<(u32, f32, f32, f32, f32)> {
    let mfr = find_ad(ad, 0xFF)?;
    if mfr.len() < 14 {
        return None;
    }
    let company = u16::from_le_bytes([mfr[0], mfr[1]]);
    if company != 0xFFFF {
        return None;
    }
    let p = &mfr[2..];
    let ts = u32::from_be_bytes([p[0], p[1], p[2], p[3]]);
    let w = i16::from_be_bytes([p[4], p[5]]) as f32 / 16384.0;
    let x = i16::from_be_bytes([p[6], p[7]]) as f32 / 16384.0;
    let y = i16::from_be_bytes([p[8], p[9]]) as f32 / 16384.0;
    let z = i16::from_be_bytes([p[10], p[11]]) as f32 / 16384.0;
    Some((ts, w, x, y, z))
}

/// Parse "AA:BB:CC:DD:EE:FF" → [u8; 6] in HCI byte order (reversed).
fn parse_addr(s: &str) -> Option<[u8; 6]> {
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        return None;
    }
    let mut addr = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        addr[5 - i] = u8::from_str_radix(p, 16).ok()?;
    }
    Some(addr)
}

// ---- Main ----

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let multi = args.iter().any(|a| a == "--multi");

    // Parse --addr arguments: each specifies a BLE address in channel order
    // e.g. --addr AA:BB:CC:DD:EE:FF --addr 11:22:33:44:55:66
    //       → channel 0 = AA:BB:..., channel 1 = 11:22:...
    let mut pinned_addrs: Vec<[u8; 6]> = Vec::new();
    {
        let mut i = 1;
        while i < args.len() {
            if args[i] == "--addr" {
                i += 1;
                if i >= args.len() {
                    eprintln!("--addr requires a value (AA:BB:CC:DD:EE:FF)");
                    process::exit(1);
                }
                match parse_addr(&args[i]) {
                    Some(a) => {
                        eprintln!(
                            "pinned channel {} = {}",
                            pinned_addrs.len(),
                            args[i].to_uppercase()
                        );
                        pinned_addrs.push(a);
                    }
                    None => {
                        eprintln!("invalid BLE address: {}", args[i]);
                        process::exit(1);
                    }
                }
            }
            i += 1;
        }
    }

    // Install SIGINT handler without SA_RESTART so read() returns EINTR
    unsafe {
        let mut sa: libc::sigaction = mem::zeroed();
        sa.sa_sigaction = on_sigint as usize;
        sa.sa_flags = 0;
        libc::sigaction(libc::SIGINT, &sa, ptr::null_mut());
    }

    // Take exclusive control of hci0: bring down, then open user channel
    eprintln!("Taking exclusive control of hci0 (BlueZ will resume on exit)...");
    if let Err(e) = hci_dev_down(0) {
        eprintln!("Failed to bring hci0 down: {}", e);
        eprintln!("Try: sudo ./target/debug/ble-scan");
        process::exit(1);
    }

    let fd = match hci_open_user(0) {
        Ok(fd) => fd,
        Err(e) => {
            eprintln!("Failed to open HCI user channel: {}", e);
            process::exit(1);
        }
    };

    eprintln!("Initializing controller...");
    if let Err(e) = hci_init_scanning(fd) {
        eprintln!("Failed: {}", e);
        unsafe { libc::close(fd); }
        process::exit(1);
    }

    if multi {
        eprintln!("Scanning (--multi: ch,ts,w,x,y,z)  Ctrl-C to stop");
    } else {
        eprintln!("Scanning (ts,w,x,y,z)  Ctrl-C to stop");
    }

    let stdout = io::stdout();
    let mut out = stdout.lock();
    let mut sensors: HashMap<[u8; 6], u8> = HashMap::new();
    // Pre-populate pinned address→channel mappings
    for (ch, &a) in pinned_addrs.iter().enumerate() {
        sensors.insert(a, ch as u8);
    }
    let mut next_ch: u8 = pinned_addrs.len() as u8;
    let mut seen: std::collections::HashSet<[u8; 6]> = std::collections::HashSet::new();
    let mut count: u64 = 0;
    let mut buf = [0u8; 512];

    while RUNNING.load(Ordering::Relaxed) {
        let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n <= 0 {
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                eprintln!("read: {}", e);
            }
            break;
        }
        let n = n as usize;

        // HCI event: [0x04] [event=0x3E] [param_len] [subevent=0x02] [num_reports] ...
        if n < 5 || buf[0] != HCI_EVENT_PKT || buf[1] != HCI_EV_LE_META {
            continue;
        }
        if buf[3] != LE_ADV_REPORT {
            continue;
        }

        let num = buf[4] as usize;
        let mut off = 5;

        for _ in 0..num {
            // event_type(1) + addr_type(1) + addr(6) + data_len(1) = 9 bytes header
            if off + 9 > n {
                break;
            }
            let mut addr = [0u8; 6];
            addr.copy_from_slice(&buf[off + 2..off + 8]);
            let dlen = buf[off + 8] as usize;
            off += 9;

            if off + dlen + 1 > n {
                break; // +1 for RSSI byte
            }
            let ad = &buf[off..off + dlen];
            off += dlen + 1;

            if !has_xiao_name(ad) {
                continue;
            }
            if let Some((ts, w, x, y, z)) = decode_mfr_quaternion(ad) {
                // Single mode with pinned addr: only output that sensor
                if !multi && !pinned_addrs.is_empty() && !pinned_addrs.contains(&addr) {
                    continue;
                }

                let ch = *sensors.entry(addr).or_insert_with(|| {
                    let c = next_ch;
                    next_ch += 1;
                    c
                });
                if seen.insert(addr) {
                    let pinned = pinned_addrs.iter().any(|a| *a == addr);
                    eprintln!(
                        "sensor {} = {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}{}",
                        ch, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
                        if pinned { " (pinned)" } else { "" }
                    );
                }

                if multi {
                    let _ =
                        writeln!(out, "{},{},{:.6},{:.6},{:.6},{:.6}", ch, ts, w, x, y, z);
                } else {
                    let _ = writeln!(out, "{},{:.6},{:.6},{:.6},{:.6}", ts, w, x, y, z);
                }
                let _ = out.flush();
                count += 1;
            }
        }
    }

    eprintln!("\nReceived {} packets", count);
    // Disable scan, close socket — BlueZ reclaims hci0 automatically
    let _ = hci_write_cmd(fd, LE_SET_SCAN_ENABLE, &[0x00, 0x00]);
    unsafe { libc::close(fd); }
    eprintln!("Done (BlueZ reclaiming hci0).");
}
