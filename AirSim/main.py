import airsim
from airsim import MultirotorClient

import time

class PIDSingle():
    def __init__(self):
        self.kp = None
        self.ki = None
        self.kd = None
        self.reference = None
        self.meas_value = None
        self.meas_value_prev = 0
        self.error = None
        self.error_prev = 0
        self.error_sum = 0
        self.error_deriv = None
        self.error_deriv_filt = 0
        self.p_result = None
        self.i_result = None
        self.d_result = None
        self.pid_result = None

class PIDDouble():
    def __init__(self):
        self.IN = PIDSingle()
        self.OUT = PIDSingle()

# drone 객체 생성 후 이륙
class MyClient(MultirotorClient):

    def __init__(self, **kwargs):
        super(MyClient, self).__init__(**kwargs)
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        #self.client.reset() # 실행 시 뒤에 코드가 무시될 수 있어 필요할 때만 주석 해제
        self.client.takeoffAsync().join()

        self.roll = PIDDouble()
        self.pitch = PIDDouble()
        # Set PID Gain
        self.set_pid_gain()

        # Drone Value
        self.angle = {'pitch': None, 'roll': None, 'yaw': None}
        self.rate = {'pitch': None, 'roll': None, 'yaw': None}
        self.altitude = None
        self.longitude = None
        self.latitude = None
        self.DT = 0.001

        self.ccr = {1 : None, 2 : None, 3 : None, 4 : None, 'duration': 1}

    def run(self):
        while 1:
            start = time.time()
            # Read Sensor Data
            # IMU
            self.read_angle()
            self.read_rate()
            # Barometer
            self.read_altitude()
            # GPS
            self.read_gps()

            # PID Control
            self.Double_Roll_Pitch_PID_Calculation(self.pitch, 0, self.angle['pitch'], self.rate['pitch'], self.DT)
            self.Double_Roll_Pitch_PID_Calculation(self.roll, 0, self.angle['roll'], self.rate['roll'], self.DT)

            # PWM
            self.ccr[1] = 84000 + 600 * 84 - self.pitch.IN.pid_result + self.roll.IN.pid_result
            self.ccr[2] = 84000 + 600 * 84 + self.pitch.IN.pid_result + self.roll.IN.pid_result
            self.ccr[3] = 84000 + 600 * 84 + self.pitch.IN.pid_result - self.roll.IN.pid_result
            self.ccr[4] = 84000 + 600 * 84 - self.pitch.IN.pid_result - self.roll.IN.pid_result

            for i in range(4):
                if self.ccr[i+1] < 84000: self.ccr[i+1] = 84000
                elif self.ccr[i+1] > 168000: self.ccr[i+1] = 168000

                self.ccr[i+1] = (self.ccr[i+1] - 84000) / 84000

            # Write Motors
            # print(self.ccr[1],self.ccr[2],self.ccr[3],self.ccr[4])
            self.client.moveByMotorPWMsAsync(self.ccr[4], self.ccr[2],\
                                             self.ccr[1], self.ccr[3], self.ccr['duration'])

            # print('roll : ',self.angle['roll'],' pitch : ', self.angle['pitch'], ' yaw : ', self.angle['yaw'])

            self.DT = start - time.time()

    def read_angle(self):
        self.angle['pitch'], self.angle['roll'], self.angle['yaw'] = \
        airsim.to_eularian_angles(self.client.getImuData('Imu', 'Drone1').orientation)

        self.angle['pitch'] *= -1

    def read_rate(self):
        self.rate['roll'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.x_val
        self.rate['pitch'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.y_val
        self.rate['yaw'] = self.client.getImuData('Imu', 'Drone1').angular_velocity.z_val

        self.rate['pitch'] *= -1

    def read_altitude(self):
        self.altitude = self.client.getBarometerData(barometer_name='Barometer', vehicle_name='Drone1').altitude

    def read_gps(self):
        self.longitude = self.client.getGpsData('Gps', 'Drone1').gnss.geo_point.longitude
        self.latitude = self.client.getGpsData('Gps', 'Drone1').gnss.geo_point.latitude

    def Double_Roll_Pitch_PID_Calculation(self, axis : PIDDouble, set_point_angle: float, angle: float, rate: float, DT:float):
        # *********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************#
        # P calculation
        axis.OUT.reference = set_point_angle
        axis.OUT.meas_value = angle
        axis.OUT.error = axis.OUT.reference - axis.OUT.meas_value
        axis.OUT.p_result = axis.OUT.error * axis.OUT.kp

        # I calculation, DT need to be given in every loop
        axis.OUT.error_sum = axis.OUT.error_sum + axis.OUT.error * DT  # Define summation of outer loop
        OUT_ERR_SUM_MAX = 2000
        OUT_ERR_SUM_MIN = -OUT_ERR_SUM_MAX
        if (axis.OUT.error_sum > OUT_ERR_SUM_MAX): axis.OUT.error_sum = OUT_ERR_SUM_MAX
        elif (axis.OUT.error_sum < OUT_ERR_SUM_MIN): axis.OUT.error_sum = OUT_ERR_SUM_MIN
        axis.OUT.i_result = axis.OUT.error_sum * axis.OUT.ki  # Calculate I result of outer loop

        # D calculation
        OUTER_DERIV_FILT_ENABLE = True
        axis.OUT.error_deriv = -rate  # Define derivative of outer loop (rate = ICM-20602 Angular Rate)
        if (not OUTER_DERIV_FILT_ENABLE) :
            axis.OUT.d_result = axis.OUT.error_deriv * axis.OUT.kd  # Calculate D result of outer loop
        else :
            axis.OUT.error_deriv_filt = axis.OUT.error_deriv_filt * 0.4 + axis.OUT.error_deriv * 0.6 # filter for derivative
            axis.OUT.d_result = axis.OUT.error_deriv_filt * axis.OUT.kd  # Calculate D result of inner loop

        axis.OUT.pid_result = axis.OUT.p_result + axis.OUT.i_result + axis.OUT.d_result  # Calculate PID result of outer loop
        # ****************************************************************************************#

        # ************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************#
        axis.IN.reference = axis.OUT.pid_result  # Set point of inner PID control is the PID result of outer loop (for double PID control)
        axis.IN.meas_value = rate  # ICM-20602 angular rate

        axis.IN.error = axis.IN.reference - axis.IN.meas_value  # Define error of inner loop
        axis.IN.p_result = axis.IN.error * axis.IN.kp  # Calculate P result of inner loop

        axis.IN.error_sum = axis.IN.error_sum + axis.IN.error * DT  # Define summation of inner loop
        IN_ERR_SUM_MAX = 500
        IN_ERR_SUM_MIN = -IN_ERR_SUM_MAX
        if (axis.OUT.error_sum > IN_ERR_SUM_MAX): axis.OUT.error_sum = IN_ERR_SUM_MAX
        elif (axis.OUT.error_sum < IN_ERR_SUM_MIN):  axis.OUT.error_sum = IN_ERR_SUM_MIN
        axis.IN.i_result = axis.IN.error_sum * axis.IN.ki  # Calculate I result of inner loop

        axis.IN.error_deriv = -(axis.IN.meas_value - axis.IN.meas_value_prev) / DT  # Define derivative of inner loop
        axis.IN.meas_value_prev = axis.IN.meas_value  # Refresh value_prev to the latest value

        INNER_DERIV_FILT_ENABLE = True
        if (not INNER_DERIV_FILT_ENABLE):
            axis.IN.d_result = axis.IN.error_deriv * axis.IN.kd;  # Calculate D result of inner loop
        else:
            axis.IN.error_deriv_filt = axis.IN.error_deriv_filt * 0.5 + axis.IN.error_deriv * 0.5 # filter for derivative
            axis.IN.d_result = axis.IN.error_deriv_filt * axis.IN.kd  # Calculate D result of inner loop

        axis.IN.pid_result = axis.IN.p_result + axis.IN.i_result + axis.IN.d_result  # Calculate PID result of inner loop
        # ****************************************************************************************#
    def set_pid_gain(self):
        self.pitch.IN.kp = 104
        self.pitch.IN.ki = 0
        self.pitch.IN.kd = 7
        self.pitch.OUT.kp = 14
        self.pitch.OUT.ki = 0
        self.pitch.OUT.kd = 3

        self.roll.IN.kp = 104
        self.roll.IN.ki = 0
        self.roll.IN.kd = 7
        self.roll.OUT.kp = 14
        self.roll.OUT.ki = 0
        self.roll.OUT.kd = 3

if __name__ == "__main__":

    kuav = MyClient()
    kuav.run()
