import numpy as np
import serial

serSTM = serial.Serial('/dev/ttyUSB0',115200)
serSTM.flush()
while True:
        countSTM = serSTM.in_waiting
        if countSTM > 34:
            recvSTM = serSTM.read(35)
            serSTM.reset_input_buffer()
            if recvSTM[0] == 0x88 and recvSTM[1] == 0x18:
                mode_echo = np.int16(recvSTM[2])

                lat_drone = np.int32(
                    np.int32(recvSTM[3] << 24) + np.int32(recvSTM[4] << 16) + np.int32(recvSTM[5] << 8) + recvSTM[
                        6]) / 10 ** 7
                lon_drone = np.int32(
                    np.int32(recvSTM[7] << 24) + np.int32(recvSTM[8] << 16) + np.int32(recvSTM[9] << 8) + recvSTM[
                        10]) / 10 ** 7
                gps_time = np.int32(
                    np.int32(recvSTM[11] << 24) + np.int32(recvSTM[12] << 16) + np.int32(recvSTM[13] << 8) +
                    recvSTM[14])

                roll = np.int32(
                    np.int32(recvSTM[15] << 24) + np.int32(recvSTM[16] << 16) + np.int32(recvSTM[17] << 8) +
                    recvSTM[18])
                pitch = np.int32(
                    np.int32(recvSTM[19] << 24) + np.int32(recvSTM[20] << 16) + np.int32(recvSTM[21] << 8) +
                    recvSTM[22])
                heading_angle = np.int32(
                    np.int32(recvSTM[23] << 24) + np.int32(recvSTM[24] << 16) + np.int32(recvSTM[25] << 8) +
                    recvSTM[26])

                altitude = np.int32(
                    np.int32(recvSTM[27] << 24) + np.int32(recvSTM[28] << 16) + np.int32(recvSTM[29] << 8) +
                    recvSTM[30])
                voltage = np.int32(
                    np.int32(recvSTM[31] << 24) + np.int32(recvSTM[32] << 16) + np.int32(recvSTM[33] << 8) +
                    recvSTM[34])
                print("From STM : " , mode_echo , lat_drone , lon_drone , gps_time , roll , pitch , heading_angle , altitude , voltage)

                serSTM.reset_input_buffer()
    