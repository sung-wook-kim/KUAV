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

        # Motor Value
        self.motor = {1:0.5,2:0.5,3:0.5,4:0.5,'duration':1}

    def run(self):
        while 1:
            self.client.moveByMotorPWMsAsync(self.motor[4], self.motor[2],\
                                             self.motor[1], self.motor[3], self.motor['duration']).join()

            # Barometer
            barometer_data = self.client.getBarometerData(barometer_name='Barometer', vehicle_name='Drone1')
            temp = barometer_data.pressure
            altitude = barometer_data.altitude

            # GPS
            gps_data = self.client.getGpsData('Gps', 'Drone1')

            # IMU
            imu_data = self.client.getImuData('Imu', 'Drone1')

            print(f'altitude : {altitude}')

if __name__ == "__main__":

    kuav = MyClient()

    kuav.run()