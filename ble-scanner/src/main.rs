use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::Manager;
use std::time::Duration;
use tokio::time;

/// Decode manufacturer data: 4-byte BE timestamp + 4× BE i16 Q14 quaternion.
/// Returns (timestamp_ms, [w, x, y, z]).
fn decode_payload(data: &[u8]) -> Option<(u32, [f32; 4])> {
    if data.len() < 12 {
        return None;
    }
    let ts = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
    let mut q = [0.0f32; 4];
    for i in 0..4 {
        let raw = i16::from_be_bytes([data[4 + i * 2], data[4 + i * 2 + 1]]);
        q[i] = raw as f32 / 16384.0;
    }
    Some((ts, q))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next().ok_or("No Bluetooth adapter found")?;

    println!("Scanning for BLE devices...\n");
    adapter.start_scan(ScanFilter::default()).await?;

    let mut last_ts: u32 = u32::MAX;

    loop {
        time::sleep(Duration::from_millis(1)).await;

        let peripherals = adapter.peripherals().await?;
        for p in &peripherals {
            let Some(props) = p.properties().await? else { continue };
            let name = props.local_name.as_deref().unwrap_or("(unknown)");

            if !name.contains("XIAO") {
                continue;
            }

            for (company_id, data) in &props.manufacturer_data {
                if *company_id == 0xFFFF {
                    if let Some((ts, [w, x, y, z])) = decode_payload(data) {
                        if ts == last_ts {
                            continue; // skip duplicate
                        }
                        last_ts = ts;
                        let norm = (w * w + x * x + y * y + z * z).sqrt();
                        let secs = ts / 1000;
                        let ms = ts % 1000;
                        println!("{}.{:03}  w={:.4} x={:.4} y={:.4} z={:.4}  norm={:.4}  RSSI:{:?}",
                            secs, ms, w, x, y, z, norm, props.rssi);
                    }
                }
            }
        }
    }
}
