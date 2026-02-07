use once_cell::sync::Lazy;
use regex::Regex;

#[derive(Clone, Copy, Debug)]
pub struct RawImuData {
    timestamp: f32, // seconds (float) or u64 milliseconds
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
}

#[derive(Clone, Copy)]
struct ScaledImuData {
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
}

#[derive(Clone, Copy, Default)]
struct Orientation {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

enum TiltCalculationMethod {
    //******* Accel only ********
    //  pitch = atan2(Ax, sqrt(Ay² + Az²))
    //  roll = atan2(Ay, sqrt(Ax² + Az²))
    Fundamental,
    // tilt = asin(Ax / g)
    // tilt = acos(Az / g)
    SingleAxis,
    // tilt = acos(Az / sqrt(Ax² + Ay² + Az²))
    TotalInclination,
    //******* Gyro only ********
    // θ(t)=θ(t−1)+ω⋅dt
    GyroOnly,
    //******* Mixed ********
    // angle=α⋅(angle+ω⋅dt)+(1−α)⋅acc_angle
    Complementary { alpha: f32 }, // alpha: gyro weight
    KalmanFilter,                 //advanced TODO
}

pub struct ImuFilter {
    pub orientation: Orientation,
    pub method: TiltCalculationMethod,
}

impl ImuFilter {
    pub fn new(method: TiltCalculationMethod) -> Self {
        Self {
            orientation: Orientation::default(),
            method,
        }
    }

    pub fn update(&mut self, data: ScaledImuData, dt: f32) {
        match self.method {
            TiltCalculationMethod::Fundamental => {
                let roll = (data.ay).atan2(data.az);
                let pitch = (-data.ax).atan2((data.ay * data.ay + data.az * data.az).sqrt());
                self.orientation.roll = roll;
                self.orientation.pitch = pitch;
                // yaw cannot be estimated from accel alone
            }
            TiltCalculationMethod::SingleAxis => {
                let roll = data.ay.asin();
                let pitch = (-data.ax).asin();
                self.orientation.roll = roll;
                self.orientation.pitch = pitch;
            }
            TiltCalculationMethod::TotalInclination => {
                let norm = (data.ax * data.ax + data.ay * data.ay + data.az * data.az).sqrt();
                let tilt_x = (data.ax / norm).acos();
                let tilt_y = (data.ay / norm).acos();
                let tilt_z = (data.az / norm).acos();
                self.orientation.roll = tilt_x;
                self.orientation.pitch = tilt_y;
                self.orientation.yaw = tilt_z; // approximate total tilt
            }
            TiltCalculationMethod::GyroOnly => {
                // integrate angular velocities
                self.orientation.roll += data.gx * dt;
                self.orientation.pitch += data.gy * dt;
                self.orientation.yaw += data.gz * dt;
            }
            TiltCalculationMethod::Complementary { alpha } => {
                // compute accel-based angles
                let roll_acc = (data.ay).atan2(data.az);
                let pitch_acc = (-data.ax).atan2((data.ay * data.ay + data.az * data.az).sqrt());

                // integrate gyro
                let roll_gyro = self.orientation.roll + data.gx * dt;
                let pitch_gyro = self.orientation.pitch + data.gy * dt;
                let yaw_gyro = self.orientation.yaw + data.gz * dt;

                // complementary filter: combine
                self.orientation.roll = alpha * roll_gyro + (1.0 - alpha) * roll_acc;
                self.orientation.pitch = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc;
                self.orientation.yaw = yaw_gyro; // yaw cannot be corrected with accel
            }
            TiltCalculationMethod::KalmanFilter => todo!(),
        }
    }

    pub fn get_orientation(&self) -> &Orientation {
        &self.orientation
    }

    pub fn roll_deg(&self) -> f32 {
        self.orientation.roll.to_degrees()
    }
    pub fn pitch_deg(&self) -> f32 {
        self.orientation.pitch.to_degrees()
    }
    pub fn yaw_deg(&self) -> f32 {
        self.orientation.yaw.to_degrees()
    }
}

static IMU_LINE_RE: Lazy<Regex> = Lazy::new(|| {
    Regex::new(r"^(\d{2}:\d{2}:\d{2}\.\d+):\s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)$")
        .unwrap()
});

pub fn parse_imu_line(line: &str) -> Option<RawImu> {
    /*
    line is of the form "12:15:20.432: 15996,5528,3340,-233,-127,-110"
    */
    if let Some(caps) = IMU_LINE_RE.captures(line) {
        let timestamp = &caps[1];
        let ax: f32 = caps[2].parse().unwrap_or(0.0);
        let ay: f32 = caps[3].parse().unwrap_or(0.0);
        let az: f32 = caps[4].parse().unwrap_or(0.0);
        let gx: f32 = caps[5].parse().unwrap_or(0.0);
        let gy: f32 = caps[6].parse().unwrap_or(0.0);
        let gz: f32 = caps[7].parse().unwrap_or(0.0);

        println!("timestamp={}", timestamp);
        println!(
            "ax={}, ay={}, az={}, gx={}, gy={}, gz={}",
            ax, ay, az, gx, gy, gz
        );
        let timestamp = parse_hhmmss(timestamp);
        return Some(RawImu {
            timestamp,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
        });
    }
    None
}

fn parse_hhmmss(s: &str) -> f32 {
    // Expects "HH:MM:SS.mmm"
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 3 {
        return 0.0;
    }

    let hours: f32 = parts[0].parse().unwrap_or(0.0);
    let minutes: f32 = parts[1].parse().unwrap_or(0.0);

    let seconds: f32 = parts[2].parse().unwrap_or(0.0);

    hours * 3600.0 + minutes * 60.0 + seconds
}

pub fn clean_raw_readings(
    raw: RawImuData,
    gyro_bias: Option<&GyroBias>,
    accel_bias: Option<&AccelBias>,
    gyro_sens: f32,
    accel_sens: f32,
) -> ScaledImu {
    /*
     scaled = (raw - bias) / sens
    */
    let mut scaled = raw.into();
    if let Some(gyro_bias) = gyro_bias.as_ref() {
        scaled.gx -= gyro_bias.x;
        scaled.gy -= gyro_bias.y;
        scaled.gz -= gyro_bias.z;
    }
    if let Some(accel_bias) = accel_bias.as_ref() {
        unimplemented!(); //TODO
    }
    scaled.gx /= gyro_sens;
    scaled.gy /= gyro_sens;
    scaled.gz /= gyro_sens;

    scaled.ax /= accel_sens;
    scaled.ay /= accel_sens;
    scaled.az /= accel_sens;

    scaled
}

impl From<RawImuData> for ScaledImuData {
    fn from(value: RawImuData) -> Self {
        Self {
            ax: value.ax,
            ay: value.ay,
            az: value.az,
            gx: value.gx,
            gy: value.gy,
            gz: value.gz,
        }
    }
}

//fn apply_sensitivity(raw: RawImuData, accel_sens: f32, gyro_sens: f32) -> ScaledImu {
//ScaledImu {
//ax: raw.ax / accel_sens,
//ay: raw.ay / accel_sens,
//az: raw.az / accel_sens,
//gx: (raw.gx / gyro_sens).to_radians(),
//gy: (raw.gy / gyro_sens).to_radians(),
//gz: (raw.gz / gyro_sens).to_radians(),
//}
//}

//fn apply_calibration(
//scaled: ScaledImuData,
//bias_accel: &RawImuData,
//bias_gyro: &RawImuData,
//) -> ScaledImu {
//ScaledImu {
//ax: scaled.ax - bias_accel.ax,
//ay: scaled.ay - bias_accel.ay,
//az: scaled.az - bias_accel.az,
//gx: scaled.gx - bias_gyro.gx,
//gy: scaled.gy - bias_gyro.gy,
//gz: scaled.gz - bias_gyro.gz,
//}
//}

//fn apply_calibration(
//scaled: ScaledImuData,
//bias_accel: Option<&AccelBias>,
//bias_gyro: Option<&GyroBias>,
//) -> ScaledImu {
//ScaledImu {
//ax: scaled.ax - bias_accel.ax,
//ay: scaled.ay - bias_accel.ay,
//az: scaled.az - bias_accel.az,
//gx: scaled.gx - bias_gyro.gx,
//gy: scaled.gy - bias_gyro.gy,
//gz: scaled.gz - bias_gyro.gz,
//}
//}

fn integrate(filter: &mut ImuFilter, imu: ScaledImu, dt: f32) -> Orientation {
    filter.update(
        ImuData {
            ax: imu.ax,
            ay: imu.ay,
            az: imu.az,
            gx: imu.gx,
            gy: imu.gy,
            gz: imu.gz,
        },
        dt,
    );
    filter.orientation
}

pub fn collect_sample(reader: &BufReader, num_samples: usize) -> Vec<RawImuData> {
    let mut sample = Vec::new();
    for line_result in reader.lines().take(num_samples) {
        if let Err(e) = line {
            eprintln!("Read error: {:?}", e);
            continue;
        }
        let line: &str = line.unwrap();
        if let Some(raw_imu) = parse_imu_line(line) {
            sample.push(raw_imu);
        }
    }
    sample
}

/// Gyro bias is the offset of the sensors raw input when it is stationary
#[derive(Default, Clone, Copy)]
pub struct GyroBias {
    x: f32,
    y: f32,
    z: f32,
}

pub fn calculate_gyro_bias_from_sample(data_points: Vec<RawImuData>) -> GyroBias {
    let mut bias = GyroBias::default();
    let num_samples = data_points.len() as f32;
    for data_point in data_points {
        bias.x += data_point.gx;
        bias.y += data_point.gy;
        bias.z += data_point.gz;
    }
    bias.x /= num_samples;
    bias.y /= num_samples;
    bias.z /= num_samples;

    bias
}

pub fn calculate_gyro_bias(imu: &SharedImu, num_samples: usize) -> GyroBias {
    let mut sum = GyroBias::default();

    for _ in 0..num_samples {
        let data = imu.0.lock().unwrap(); // read current gyro values
        sum.x += data.gx;
        sum.y += data.gy;
        sum.z += data.gz;

        // small delay between samples to avoid reading too fast
        std::thread::sleep(std::time::Duration::from_millis(5));
    }

    GyroBias {
        x: sum.x / num_samples as f32,
        y: sum.y / num_samples as f32,
        z: sum.z / num_samples as f32,
    }
}

pub struct AccelBias {}

#[test]
fn test_line_parse() {
    let line = "12:15:20.432: 15996,5528,3340,-233,-127,-110";
    let result = parse_imu_line(line);
    println!("{:#?}", result);
}
