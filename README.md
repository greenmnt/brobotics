socat -d -d \
  PTY,raw,echo=0,link=/tmp/imu_in \
  PTY,raw,echo=0,link=/tmp/imu_out


  MPU6050 githubs
   - https://github.com/ZHomeSlice/Simple_MPU6050
   - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino
