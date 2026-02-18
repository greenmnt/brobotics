use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::Manager;
use std::time::Duration;
use tokio::time;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next().ok_or("No Bluetooth adapter found")?;

    println!("Scanning for BLE devices...\n");
    adapter.start_scan(ScanFilter::default()).await?;

    loop {
        time::sleep(Duration::from_secs(1)).await;

        let peripherals = adapter.peripherals().await?;
        for p in &peripherals {
            let Some(props) = p.properties().await? else { continue };
            let name = props.local_name.as_deref().unwrap_or("(unknown)");

            if !name.contains("XIAO") {
                continue;
            }

            println!("--- {} [{}] RSSI: {:?} ---",
                name,
                props.address,
                props.rssi,
            );

            // Print manufacturer data
            for (company_id, data) in &props.manufacturer_data {
                println!("  Manufacturer 0x{:04X}: {:02X?}", company_id, data);
            }

            // Print service data
            for (uuid, data) in &props.service_data {
                println!("  Service {}: {:02X?}", uuid, data);
            }

            // Print service UUIDs
            if !props.services.is_empty() {
                println!("  Services: {:?}", props.services);
            }

            // Print TX power
            if let Some(tx) = props.tx_power_level {
                println!("  TX Power: {} dBm", tx);
            }

            println!();
        }
    }
}
