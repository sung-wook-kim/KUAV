import storm32
import os
os.system('sudo chmod 666 /dev/ttyACM0')

gimbal = storm32.Storm32(port='/dev/ttyACM0')

gimbal.set_angles(0,0,0)
while True:
  print(gimbal.get_imu1_angles())


