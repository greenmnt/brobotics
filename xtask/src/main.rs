use std::path::{Path, PathBuf};
use std::process::{self, Command};
use std::{env, fs};

const UF2_BASE: &str = "0x27000";
const UF2_FAMILY: &str = "0xADA52840";

fn workspace_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn run(cmd: &str, args: &[&str], dir: &Path) {
    let status = Command::new(cmd)
        .args(args)
        .current_dir(dir)
        .status()
        .unwrap_or_else(|e| {
            eprintln!("Failed to run {} {:?}: {}", cmd, args, e);
            process::exit(1);
        });
    if !status.success() {
        process::exit(status.code().unwrap_or(1));
    }
}

fn flash(crate_name: &str) {
    let ws = workspace_dir();
    let crate_dir = ws.join(crate_name);

    if !crate_dir.exists() {
        eprintln!("Crate not found: {}", crate_dir.display());
        process::exit(1);
    }

    let target_dir = ws.join("target/thumbv7em-none-eabihf/release");
    let elf = target_dir.join(crate_name);
    let bin = target_dir.join(format!("{}.bin", crate_name));
    let uf2 = target_dir.join(format!("{}.uf2", crate_name));

    // Build from crate dir so its .cargo/config.toml is picked up
    eprintln!("Building {}...", crate_name);
    run("cargo", &["build", "--release"], &crate_dir);

    // ELF → binary
    eprintln!("Converting to binary...");
    run(
        "rust-objcopy",
        &["-O", "binary", elf.to_str().unwrap(), bin.to_str().unwrap()],
        &ws,
    );

    // Binary → UF2
    eprintln!("Creating UF2...");
    run(
        "uf2conv",
        &[
            bin.to_str().unwrap(),
            "--base",
            UF2_BASE,
            "--family",
            UF2_FAMILY,
            "--output",
            uf2.to_str().unwrap(),
        ],
        &ws,
    );

    // Try to copy to mounted XIAO
    let mount = find_xiao_mount();
    if let Some(mount) = mount {
        eprintln!("Flashing to {}...", mount.display());
        fs::copy(&uf2, mount.join(format!("{}.uf2", crate_name))).unwrap_or_else(|e| {
            eprintln!("Copy failed: {} (double-tap reset first?)", e);
            process::exit(1);
        });
        eprintln!("Done — board will reset automatically.");
    } else {
        eprintln!("UF2 ready: {}", uf2.display());
        eprintln!("Double-tap reset on the XIAO, then run again to flash.");
    }
}

fn find_xiao_mount() -> Option<PathBuf> {
    // Check common mount points
    for name in &["XIAO-SENSE", "XIAO BLE"] {
        let output = Command::new("findmnt")
            .args(["-rn", "-o", "TARGET", "-S", &format!("LABEL={}", name)])
            .output()
            .ok()?;
        let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
        if !path.is_empty() {
            return Some(PathBuf::from(path));
        }
    }
    None
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();

    match args.first().map(|s| s.as_str()) {
        Some("flash") => {
            let crate_name = args.get(1).unwrap_or_else(|| {
                eprintln!("Usage: cargo xtask flash <crate>");
                process::exit(1);
            });
            flash(crate_name);
        }
        _ => {
            eprintln!("Usage:");
            eprintln!("  cargo xtask flash <crate>    Build + UF2 + flash");
            eprintln!("");
            eprintln!("Shortcuts:");
            eprintln!("  cargo xiao-sleep             Flash deep-sleep firmware");
            eprintln!("  cargo xiao-flash             Flash IMU+BLE firmware");
            process::exit(1);
        }
    }
}
