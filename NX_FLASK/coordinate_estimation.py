import cv2
import time
import numpy as np
from storm32 import Storm32
from util import *

count = 0
currTime = time.time()
gimbal = Storm32(port='COM5')
cap = cv2.VideoCapture(0)
circles = []
prm1 = 940
prm2 = 10
prmch = False
prmch2 = False
prmch3 = False
Hup = 255
Hunder = 200
lower_bound = np.array([135, 78, 50])
higher_bound = np.array([200, 200, 255])
fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
thickness = 1
i = 0
setroll, setpitch, setyaw = 0, 45, 0

K = np.array([[2.015446587076636433e+03, 0, 9.765539697951869584e+02],
              [0, 2.017362842708620747e+03, 4.778646481084207380e+02], [0, 0, 1]])
K_inv = np.linalg.inv(K)
T = np.array([[0], [0], [-1]])
R_tran = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

while (True):
    gimbal.set_angles(setpitch, setroll, setyaw)
    src = cap.read()[1]
    try:
        frame_gau_blur = cv2.GaussianBlur(src, (3, 3), 0)
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        blue_range = cv2.inRange(hsv, lower_bound, higher_bound)

        blue_s_gray = blue_range[::1]
        cv2.imshow("color filter", blue_s_gray)
        # print(blue_s_gray)
        circles = cv2.HoughCircles(blue_s_gray, cv2.HOUGH_GRADIENT, 1, 100, param1=prm1, param2=prm2, minRadius=5,
                                   maxRadius=50)[0]
        roll, pitch, yaw = gimbal.get_imu1_angles()
        roll = math.pi/180*roll
        pitch = -math.pi/180*pitch
        yaw = -math.pi/180*yaw


        OR_G = euler_rotation_matrix(roll, pitch, yaw)
        OR_C = np.hstack((OR_G, T))
        OR_C = np.vstack((OR_C, np.array([[0, 0, 0, 1]])))
        OR_C = np.matmul(OR_C, R_tran)
        CR_O = np.linalg.inv(OR_C)
        A = np.dstack((CR_O[0:3, 0], CR_O[0:3, 1], CR_O[0:3, 3]))

        for i in circles:
            cv2.circle(src, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 1)
            A = np.linalg.inv(A)
            xy_est = np.matmul(np.matmul(A, K_inv), np.array([i[0], i[1], 1]))
            xy_est = xy_est[0]
            xy_est = xy_est / xy_est[2]
            H_x, H_y = (xy_est[0], xy_est[1])
            cv2.putText(src, str((H_x, H_y)), (int(i[0]), int(i[1]) - 2), fontFace, fontScale,
                        (255, 0, 255), thickness)
    except:
        pass
    cv2.imshow("dst", src)
    key = cv2.waitKey(1)
    if key == 27:  # esc Key
        break
    # time.sleep(0.1)
    # elif key == ord('q'):
    #     prm1 += 1
    #     prmch = True
    # elif key == ord('w'):
    #     prm1 -= 1
    #     prmch = True
    # elif key == ord('e'):
    #     prm2 += 1
    #     prmch = True
    # elif key == ord('r'):
    #     prm2 -= 1
    #     prmch = True
    # elif key == ord('a'):
    #     Hup += 1
    #     Hup = min(255, max(Hup, Hunder))
    #     prmch2 = True
    # elif key == ord('s'):
    #     Hup -= 1
    #     Hup = min(255, max(Hup, Hunder))
    #     prmch2 = True
    # elif key == ord('d'):
    #     Hunder += 1
    #     Hunder = max(0, min(Hup, Hunder))
    #     prmch2 = True
    # elif key == ord('f'):
    #     Hunder -= 1
    #     Hunder = max(0, min(Hup, Hunder))
    #     prmch2 = True
    elif key == ord('4'):
        setyaw += 1
        prmch3 = True
    elif key == ord('6'):
        setyaw -= 1
        prmch3 = True
    elif key == ord('8'):
        setpitch += 1
        prmch3 = True
    elif key == ord('5'):
        setpitch -= 1
        prmch3 = True

    if prmch == True:
        prmch = False
        print(f"param1 = {prm1}, param2 = {prm2}")
    elif prmch2 == True:
        prmch2 = False
        print(f"Hunder = {Hunder}, Hup = {Hup}")
    elif prmch3 == True:
        prmch3 = False
        print(f"setyaw = {setyaw}, setpitch = {setpitch}")
cap.release()
cv2.destroyAllWindows()
