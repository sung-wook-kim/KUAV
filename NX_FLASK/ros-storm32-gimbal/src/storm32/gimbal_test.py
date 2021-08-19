import storm32

gimbal = storm32.Storm32(port='/dev/ttyACM0')
while True:
  print(gimbal.get_imu1_angles())


