socat -d -d \
  PTY,raw,echo=0,link=/tmp/imu_in \
  PTY,raw,echo=0,link=/tmp/imu_out
