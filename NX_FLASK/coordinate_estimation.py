import cv2
import time
import numpy as np
from storm32 import Storm32
from util import *
import os

os.system('sudo systemctl restart nvargus-daemon')
os.system('sudo chmod 666 /dev/ttyACM0')

count = 0
currTime = time.time()
gimbal = Storm32(port='/dev/ttyACM0')

cap = cv2.VideoCapture(
    gstreamer_pipeline(flip_method=2, capture_width=1920, capture_height=1080, display_width=960, display_height=540,
                       framerate=30), cv2.CAP_GSTREAMER)
circles = []
prm1 = 940
prm2 = 5
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


# 1920,1080 NX ver
# [[980.02319542   0.         453.44606334]
#  [  0.         979.3227789  276.15629447]
#  [  0.           0.           1.        ]]


class CoordEstimator:
    K = np.array([[980.02319542, 0, 453.44606334],
                  [0, 979.3227789, 276.15629447], [0, 0, 1]])
    R_tran = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    # remove distortion
    dist = np.array([-4.92456940e-01, 2.68723186e-01, -1.15903527e-04, -6.44152979e-04, -2.90173099e-02])
    h, w = 540, 960
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    x, y, w, h = roi
    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(K, dist, None, newcameramtx, (w, h), 5)
    mapx = np.array(mapx)
    mapy = np.array(mapy)
    mapx_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
    mapy_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
    for i in range(mapx_test.shape[0]):
        mapx_test[i, :] = [x for x in range(mapx_test.shape[1])]
    for j in range(mapy_test.shape[1]):
        mapy_test[:, j] = [y for y in range(mapy_test.shape[0])]

    mapx_ = mapx_test - mapx
    mapy_ = mapy_test - mapy
    K_inv = np.linalg.inv(newcameramtx)

    def est(self, pitch, roll, yaw, foot):
        # stm front ->x, right -> y, down -> z

        T = np.array([[0], [0], [-1.22]])
        OR_G = euler_rotation_matrix_YRP(roll, pitch, yaw)
        OR_C = np.hstack((OR_G, T))
        OR_C = np.vstack((OR_C, np.array([[0, 0, 0, 1]])))
        OR_C = np.matmul(OR_C, self.R_tran)
        CR_O = np.linalg.inv(OR_C)
        A = np.dstack((CR_O[0:3, 0], CR_O[0:3, 1], CR_O[0:3, 3]))
        A = np.linalg.inv(A)
        newx = foot[0] + self.mapx_[min(int(foot[1]), self.h - 1), min(int(foot[0]), self.w - 1)]
        newy = foot[1] + self.mapy_[min(int(foot[1]), self.h - 1), min(int(foot[0]), self.w - 1)]
        xy_est = np.matmul(np.matmul(A, self.K_inv),
                           np.array([min(newx + self.x, self.w), min(newy + self.y, self.h), 1]))
        xy_est = xy_est[0]
        xy_est = xy_est / xy_est[2]
        H_x, H_y = (xy_est[0], xy_est[1])
        return [H_x, H_y]


coord_estor = CoordEstimator()
j = 0
prev_H_x, prev_H_y = 1,0
min_len = 0.1
target_i = None
while (True):
    j += 1

    # read image and sensor date together
    src = cap.read()[1]
    pitch, roll, yaw = gimbal.get_imu1_angles()
    if (j % 5 == 0):
        print("roll,pitch, yaw raw", roll, pitch, yaw, end=' ')

    # image process
    frame_gau_blur = cv2.GaussianBlur(src, (3, 3), 0)
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    blue_range = cv2.inRange(hsv, lower_bound, higher_bound)
    blue_s_gray = blue_range[::1]
    # cv2.imshow("color filter", blue_s_gray)
    # print(blue_s_gray)
    try:
        circles = cv2.HoughCircles(blue_s_gray, cv2.HOUGH_GRADIENT, 1, 100, param1=prm1, param2=prm2, minRadius=5,
                                   maxRadius=50)[0]
    except:
        pass

    # tranformation process
    roll = math.pi / 180 * roll
    pitch = -math.pi / 180 * pitch
    yaw = -math.pi / 180 * yaw
    # roll = 0
    # pitch = -math.pi/180*45
    # yaw = 0
    if (j % 5 == 0):
        print("roll,pitch, yaw", roll, pitch, yaw)
    detect = False

    for i in circles:
        cv2.circle(src, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 1)
        H_x_, H_y_ = coord_estor.est(pitch, roll, yaw, i)
        foot_len = math.sqrt((H_x_ - prev_H_x) ** 2 + (H_y_ - prev_H_y) ** 2)
        if min_len > foot_len:
            min_len = foot_len
            H_x, H_y = H_x_, H_y_
            detect = True
        if detect is True:
            cv2.putText(src, str(f"{H_x:.3f}, {H_y:.3f}"), (int(i[0] - 70), int(i[1]) - 2), fontFace, fontScale,
                        (0, 0, 0), thickness)
            prev_H_x = H_x
            prev_H_y = H_y

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
    # elif key == ord('a'):-
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
        gimbal.set_angles(setpitch, setroll, setyaw)
        print(f"setyaw = {setyaw}, setpitch = {setpitch}")

cap.release()
cv2.destroyAllWindows()
