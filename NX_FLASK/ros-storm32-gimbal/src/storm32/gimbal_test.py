import storm32

gimbal = storm32.Storm32(port='COM5')
gimbal.set_angles(0,20,0)
while True:
  print(gimbal.get_imu1_angles())


