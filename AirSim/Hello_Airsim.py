import airsim
import time

# drone 객체 생성 후 이륙
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
# client.reset() # 실행 시 뒤에 코드가 무시될 수 있어 필요할 때만 주석 해제
client.takeoffAsync().join()

# drone value
motor_1 = 0.5001
motor_2 = 0.5001
motor_3 = 0.5
motor_4 = 0.5

duration = 0.001

while 1:

    client.moveByMotorPWMsAsync(motor_4, motor_2, motor_1, motor_3, duration).join()

    # Barometer
    barometer_data = client.getBarometerData(barometer_name='Barometer', vehicle_name='Drone1')
    temp = barometer_data.pressure
    altitude = barometer_data.altitude

    # GPS
    gps_data = client.getGpsData('Gps','Drone1')

    # IMU
    imu_data = client.getImuData('Imu','Drone1')

    print(f'altitude : {altitude}')