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

cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2, capture_width=3840, capture_height=2160, display_width=956, display_height=540, framerate=21), cv2.CAP_GSTREAMER)
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

# 1.019148736205558748e+03,0.000000000000000000e+00,4.543907475076335913e+02
# 0.000000000000000000e+00,1.025519691143287901e+03,2.894037118747232284e+02
# 0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00

K = np.array([[1.019148736205558748e+03, 0, 4.543907475076335913e+02],
              [0, 1.025519691143287901e+03, 2.894037118747232284e+02], [0, 0, 1]])

T = np.array([[0], [0], [-1.22]])
R_tran = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
j = 1
gimbal.set_angles(setpitch, setroll, setyaw)

# remove distortion
dist = np.array([-4.971605177084769123e-01,1.874575939087049781e-01,-2.658143561773393827e-03,-1.691380333517990709e-03,3.543487326723160913e-01])
h,  w = 540, 956
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))
x,y,w,h = roi
# undistort
mapx,mapy = cv2.initUndistortRectifyMap(K,dist,None,newcameramtx,(w,h),5)
mapx = np.array(mapx)
mapy = np.array(mapy)
mapx_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
mapy_test = np.zeros((mapx.shape[0], mapx.shape[1]), dtype=np.float32)
for i in range(mapx_test.shape[0]):
    mapx_test[i,:] = [x for x in range(mapx_test.shape[1])]

for j in range(mapy_test.shape[1]):
    mapy_test[:,j] = [y for y in range(mapy_test.shape[0])]


mapx_ = mapx_test-mapx
mapy_ = mapy_test-mapy
K_inv = np.linalg.inv(newcameramtx)

while (True):
    j+=1
    
    #read image and sensor date together
    src = cap.read()[1]
    pitch, roll, yaw = gimbal.get_imu1_angles()

    # image process
    frame_gau_blur = cv2.GaussianBlur(src, (3, 3), 0)
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    blue_range = cv2.inRange(hsv, lower_bound, higher_bound)
    blue_s_gray = blue_range[::1]
    # cv2.imshow("color filter", blue_s_gray)
    # print(blue_s_gray)
    try : 
        circles = cv2.HoughCircles(blue_s_gray, cv2.HOUGH_GRADIENT, 1, 100, param1=prm1, param2=prm2, minRadius=5,
                                    maxRadius=50)[0]
    except :
        pass

    # tranformation process
    roll = math.pi/180*roll
    pitch = -math.pi/180*pitch
    yaw = -math.pi/180*setyaw
    # roll = 0
    # pitch = -math.pi/180*45
    # yaw = 0
    if (j%5 == 0) : 
        print("roll,pitch, yaw",roll,pitch, yaw)
    OR_G = euler_rotation_matrix(roll, pitch, yaw)
    OR_C = np.hstack((OR_G, T))
    OR_C = np.vstack((OR_C, np.array([[0, 0, 0, 1]])))
    OR_C = np.matmul(OR_C, R_tran)
    CR_O = np.linalg.inv(OR_C)
    A = np.dstack((CR_O[0:3, 0], CR_O[0:3, 1], CR_O[0:3, 3]))
    A = np.linalg.inv(A)
    
    for i in circles:
        cv2.circle(src, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 1)

        newx = i[0] + mapx_[int(i[1]), int(i[0])]
        newy = i[1] + mapy_[int(i[1]), int(i[0])]
        xy_est = np.matmul(np.matmul(A, K_inv), np.array([newx-x, newy-y, 1]))
        xy_est = xy_est[0]
        xy_est = xy_est / xy_est[2]
        # xy_est2 = np.matmul(np.matmul(A, K_inv), np.array([i[0], i[1], 1]))
        # xy_est2 = xy_est[0]
        # xy_est2 = xy_est / xy_est[2]
        H_x, H_y = (xy_est[0], xy_est[1])
        cv2.putText(src, str(f"{H_x:.3f}, {H_y:.3f}"), (int(i[0]-70), int(i[1]) - 2), fontFace, fontScale,
                    (0, 0, 0), thickness)


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
