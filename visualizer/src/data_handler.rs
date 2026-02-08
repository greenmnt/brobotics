use once_cell::sync::Lazy;
use regex::Regex;
use std::io::{BufRead, BufReader};

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

impl RawImuData {
    pub fn timestamp(&self) -> f32 {
        self.timestamp
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ScaledImuData {
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
}

#[derive(Clone, Copy, Default)]
pub struct Orientation {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Clone, Copy)]
pub enum TiltFromAccelerationMethod {
    //******* Accel only ********
    //  pitch = atan2(Ax, sqrt(Ay² + Az²))
    //  roll = atan2(Ay, sqrt(Ax² + Az²))
    Fundamental,
    // tilt = asin(Ax / g)
    // tilt = acos(Az / g)
    SingleAxis,
    // tilt = acos(Az / sqrt(Ax² + Ay² + Az²))
    TotalInclination,
    // Tilt-compensated roll: roll = atan2(Ay*cos(pitch) - Az*sin(pitch), Az*cos(pitch) + Ay*sin(pitch))
    TiltCompensatedRoll,
}

impl TiltFromAccelerationMethod {
    fn pry(&self, data: &ScaledImuData) -> [f32; 3] {
        match self {
            Self::Fundamental => {
                let pitch = (-data.ax).atan2((data.ay * data.ay + data.az * data.az).sqrt());
                let roll = (data.ay).atan2(data.az);
                [pitch, roll, 0.0]
            }
            Self::SingleAxis => {
                let roll = data.ay.asin();
                let pitch = (-data.ax).asin();
                [pitch, roll, 0.0]
            }
            Self::TotalInclination => {
                let norm = (data.ax * data.ax + data.ay * data.ay + data.az * data.az).sqrt();
                let tilt_x = (data.ax / norm).acos();
                let tilt_y = (data.ay / norm).acos();
                let tilt_z = (data.az / norm).acos();
                [tilt_x, tilt_y, tilt_z]
            }
            Self::TiltCompensatedRoll => {
                let pitch = (-data.ax).atan2((data.ay * data.ay + data.az * data.az).sqrt());
                let roll = (data.ay * pitch.cos() - data.ax * pitch.sin())
                    .atan2(data.az * pitch.cos() + data.ax * pitch.sin());
                [pitch, roll, 0.0]
            }
        }
    }
}

pub enum TiltCalculationMethod {
    TiltFromAccelerationMethod(TiltFromAccelerationMethod),
    //******* Gyro only ********
    // θ(t)=θ(t−1)+ω⋅dt
    GyroOnly,
    //******* Mixed ********
    // angle=α⋅(angle+ω⋅dt)+(1−α)⋅acc_angle
    Complementary {
        alpha: f32,
        acceleration_method: TiltFromAccelerationMethod,
    }, // alpha: gyro weight
    KalmanFilter, //advanced TODO
}

pub struct ImuFilter {
    pub orientation: Orientation,
    pub gyro_orientation: Orientation,
    pub method: TiltCalculationMethod,
}

impl ImuFilter {
    pub fn new(method: TiltCalculationMethod) -> Self {
        Self {
            orientation: Orientation::default(),
            gyro_orientation: Orientation::default(),
            method,
        }
    }

    pub fn update(&mut self, data: ScaledImuData, dt: f32) {
        match self.method {
            TiltCalculationMethod::TiltFromAccelerationMethod(method) => {
                let [pitch, roll, yaw] = method.pry(&data);
                self.orientation.roll = roll;
                self.orientation.pitch = pitch;
                self.orientation.yaw = yaw;
            }
            TiltCalculationMethod::GyroOnly => {
                // integrate gyro
                self._update_raw_gyro(&data, dt);
                self.orientation = self.gyro_orientation;
            }
            TiltCalculationMethod::Complementary {
                alpha,
                acceleration_method,
            } => {
                // integrate gyro
                self._update_raw_gyro(&data, dt);

                // compute accel-based angles
                let [pitch_acc, roll_acc, _] = acceleration_method.pry(&data);

                // complementary filter: combine
                self.orientation.roll =
                    alpha * self.gyro_orientation.roll + (1.0 - alpha) * roll_acc;
                self.orientation.pitch =
                    alpha * self.gyro_orientation.pitch + (1.0 - alpha) * pitch_acc;
                self.orientation.yaw = self.gyro_orientation.yaw; // yaw cannot be corrected with accel
            }
            TiltCalculationMethod::KalmanFilter => todo!(),
        }
    }
    fn _update_raw_gyro(&mut self, data: &ScaledImuData, dt: f32) {
        self.gyro_orientation.roll = self.gyro_orientation.roll + data.gx * dt;
        self.gyro_orientation.pitch = self.gyro_orientation.pitch + data.gy * dt;
        self.gyro_orientation.yaw = self.gyro_orientation.yaw + data.gz * dt;
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

pub fn parse_imu_line(line: &str) -> Option<RawImuData> {
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

        //println!("timestamp={}", timestamp);
        //println!(
        //"ax={}, ay={}, az={}, gx={}, gy={}, gz={}",
        //ax, ay, az, gx, gy, gz
        //);
        let timestamp = parse_hhmmss(timestamp);
        return Some(RawImuData {
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
    gyro_bias: &Option<GyroBias>,
    accel_bias: &Option<AccelBias>,
    gyro_sens: f32,
    accel_sens: f32,
) -> ScaledImuData {
    /*
     scaled = (raw - bias) / sens
    */
    let mut scaled: ScaledImuData = raw.into();
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

    //now convert the gyro readings to radians (trig in rust is radian based)
    let deg_to_rad = std::f32::consts::PI / 180.0;

    scaled.gx *= deg_to_rad;
    scaled.gy *= deg_to_rad;
    scaled.gz *= deg_to_rad;

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

fn integrate(filter: &mut ImuFilter, imu: ScaledImuData, dt: f32) -> Orientation {
    filter.update(imu, dt);
    filter.orientation
}

pub fn collect_sample(
    reader: &mut BufReader<std::fs::File>,
    num_samples: usize,
) -> Vec<RawImuData> {
    let mut sample = Vec::new();
    let mut prev_time: f32 = 0.0;

    for (i, line_result) in reader.lines().take(num_samples).enumerate() {
        if i % 100 == 0 {
            tracing::info!(sample_num = i, total = num_samples);
        }
        if let Err(e) = line_result {
            eprintln!("Read error: {:?}", e);
            continue;
        }
        let line = line_result.unwrap();
        if let Some(raw_imu) = parse_imu_line(&line) {
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

pub struct AccelBias {}

#[test]
fn test_line_parse() {
    let line = "12:15:20.432: 15996,5528,3340,-233,-127,-110";
    let result = parse_imu_line(line);
    println!("{:#?}", result);
}
