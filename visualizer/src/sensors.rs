pub mod mpu_6050 {
    /*
    LSB per unit = (2^(ADC_bits - 1)) / full_scale

    ADC resolution is fixed (16-bit here)

    */
    pub mod gyro {
        //sensitivity factors, LSB(º/s)
        /*
        Least Significant Bit
        ie: raw_gyro = 131  →  1 °/s

        we can set the sensor to any of the sensitivities below
        */

        pub const FS_SEL_0: f32 = 131.0;
        pub const FS_SEL_1: f32 = 65.5;
        pub const FS_SEL_2: f32 = 32.8;
        pub const FS_SEL_3: f32 = 16.4;

        //Zero rate output
        pub const ABS_ZRO_PER_SECOND: f64 = 20.0;
    }
    pub mod accel {

        pub const ABS_ZRO_X_BIAS_MG: u32 = 50;
        pub const ABS_ZRO_Y_BIAS_MG: u32 = 50;
        pub const ABS_ZRO_Z_BIAS_MG: u32 = 80;

        /*
        Sensitivity factors
        Least Significant Bit

            raw_accel = 16384  →  1 g
            raw_accel = 8192   →  0.5 g

        */

        pub struct AccelConfig {
            pub afs_sel: u32,
            pub fs_g: f32,
            pub lsb_per_g: f32,
        }

        pub const ACCEL_TABLE: [AccelConfig; 4] = [
            AccelConfig {
                afs_sel: 0,
                fs_g: 2.0,
                lsb_per_g: 16384.0,
            },
            AccelConfig {
                afs_sel: 1,
                fs_g: 4.0,
                lsb_per_g: 8192.0,
            },
            AccelConfig {
                afs_sel: 2,
                fs_g: 8.0,
                lsb_per_g: 4096.0,
            },
            AccelConfig {
                afs_sel: 3,
                fs_g: 16.0,
                lsb_per_g: 2048.0,
            },
        ];
    }
}
