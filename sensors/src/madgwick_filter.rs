#[derive(Clone, Copy, Debug)]
pub struct Madgwick {
    pub q0: f32,
    pub q1: f32,
    pub q2: f32,
    pub q3: f32,
    beta: f32,
    inv_sample_freq: f32,
}

impl Madgwick {
    /// beta ≈ 0.05–0.2
    /// sample_freq in Hz (e.g. 200.0)
    pub fn new(beta: f32, sample_freq: f32) -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            beta,
            inv_sample_freq: 1.0 / sample_freq,
        }
    }

    pub fn update_imu(
        &mut self,
        gx: f32,
        gy: f32,
        gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
    ) {
        let mut q0 = self.q0;
        let mut q1 = self.q1;
        let mut q2 = self.q2;
        let mut q3 = self.q3;

        // Normalize accelerometer
        let norm = inv_sqrt(ax * ax + ay * ay + az * az);
        if norm == 0.0 {
            return;
        }
        let ax = ax * norm;
        let ay = ay * norm;
        let az = az * norm;

        // Gradient descent corrective step
        let f1 = 2.0 * (q1 * q3 - q0 * q2) - ax;
        let f2 = 2.0 * (q0 * q1 + q2 * q3) - ay;
        let f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az;

        let s0 = -2.0 * q2 * f1 + 2.0 * q1 * f2;
        let s1 =  2.0 * q3 * f1 + 2.0 * q0 * f2 - 4.0 * q1 * f3;
        let s2 = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 4.0 * q2 * f3;
        let s3 =  2.0 * q1 * f1 + 2.0 * q2 * f2;

        let norm_s = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        let s0 = s0 * norm_s;
        let s1 = s1 * norm_s;
        let s2 = s2 * norm_s;
        let s3 = s3 * norm_s;

        // Quaternion derivative from gyro
        let q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0;
        let q_dot1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy) - self.beta * s1;
        let q_dot2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx) - self.beta * s2;
        let q_dot3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx) - self.beta * s3;

        // Integrate
        q0 += q_dot0 * self.inv_sample_freq;
        q1 += q_dot1 * self.inv_sample_freq;
        q2 += q_dot2 * self.inv_sample_freq;
        q3 += q_dot3 * self.inv_sample_freq;

        // Normalize quaternion
        let norm_q = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        self.q0 = q0 * norm_q;
        self.q1 = q1 * norm_q;
        self.q2 = q2 * norm_q;
        self.q3 = q3 * norm_q;
    }

    pub fn quaternion(&self) -> (f32, f32, f32, f32) {
        (self.q0, self.q1, self.q2, self.q3)
    }
}

#[inline(always)]
fn inv_sqrt(x: f32) -> f32 {
    1.0 / x.sqrt()
}
