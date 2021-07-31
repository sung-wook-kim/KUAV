import airsim
from airsim import MultirotorClient

# drone 객체 생성 후 이륙
class MyClient(MultirotorClient):

    def __init__(self,**kwargs):
        super(MyClient, self).__init__(**kwargs)
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        # client.reset() # 실행 시 뒤에 코드가 무시될 수 있어 필요할 때만 주석 해제
        self.client.takeoffAsync().join()

        # Drone Value
        self.angle = {'pitch': None, 'roll': None, 'yaw': None}
        self.rate = {'pitch': None, 'roll': None, 'yaw': None}
        self.altitude = None
        self.longitude = None
        self.latitude = None
        self.motor = {1:0.60001,2:0.60001,3:0.6,4:0.6,'duration':1}

    def run(self):
        while 1:
            # Read Sensor Data
            # IMU
            self.read_angle()
            self.read_rate()
            # Barometer
            self.read_altitude()
            # GPS
            self.read_gps()

            # Write Motors
            self.client.moveByMotorPWMsAsync(self.motor[4], self.motor[2],\
                                             self.motor[1], self.motor[3], self.motor['duration']).join()

            # print('roll : ',self.angle['roll'],' pitch : ', self.angle['pitch'], ' yaw : ', self.angle['yaw'])

    def read_angle(self):
        self.angle['pitch'], self.angle['roll'], self.angle['yaw'] = \
        airsim.to_eularian_angles(self.client.getImuData('Imu', 'Drone1').orientation)

    def read_rate(self):
        self.rate['roll'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.x_val
        self.rate['pitch'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.y_val
        self.rate['yaw'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.z_val

    def read_altitude(self):
        self.altitude = self.client.getBarometerData(barometer_name='Barometer', vehicle_name='Drone1').altitude

    def read_gps(self):
        self.longitude = self.client.getGpsData('Gps', 'Drone1').gnss.geo_point.longitude
        self.latitude = self.client.getGpsData('Gps', 'Drone1').gnss.geo_point.latitude

if __name__ == "__main__":

    kuav = MyClient()

    kuav.run()