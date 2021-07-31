import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time
import pandas as pd

# 네가지 값 프린트 , 전체변경으로 value 값을 원하는 변수로 변경 하세요
# alt , target , error global declaration + value
global alt , target , error ,value
alt = 0
target = 0
error = 0
value = 0


ser = serial.Serial('COM7', 115200)
ser.flush()

# you have to receive 18bytes
def connect():
    global alt , target , error , value
    # for save
    i = 0
    now = time.localtime()
    timevar = time.strftime('%Y%m%d%H%M%S', now)
    # list
    alt_li = []
    target_li = []
    error_li = []
    value_li = []
    while True:
        i+=1
        # 100개의 데이터 마다 저장
        if i%100 == 0:
            df = pd.DataFrame()
            df['alt'] = alt_li
            df['target'] = target_li
            df['error'] = error_li
            df['pid_result'] = value_li
            df.to_csv(f"{timevar}_alt_data.csv")

        a = int(ser.read(1).hex(), 16)
        if a == 0x88:
            b = int(ser.read(1).hex(), 16)
            if b == 0x18:
                alt_1 = int(ser.read(1).hex(), 16) & 0xff
                alt_2 = int(ser.read(1).hex(), 16) & 0xff
                alt_3 = int(ser.read(1).hex(), 16) & 0xff
                alt_4 = int(ser.read(1).hex(), 16) & 0xff
                

                target_1 = int(ser.read(1).hex(), 16) & 0xff
                target_2 = int(ser.read(1).hex(), 16) & 0xff
                target_3 = int(ser.read(1).hex(), 16) & 0xff
                target_4 = int(ser.read(1).hex(), 16) & 0xff
                
                error_1 = int(ser.read(1).hex(), 16) & 0xff
                error_2 = int(ser.read(1).hex(), 16) & 0xff
                error_3 = int(ser.read(1).hex(), 16) & 0xff
                error_4 = int(ser.read(1).hex(), 16) & 0xff
                
                value_1 = int(ser.read(1).hex(), 16) & 0xff
                value_2 = int(ser.read(1).hex(), 16) & 0xff
                value_3 = int(ser.read(1).hex(), 16) & 0xff
                value_4 = int(ser.read(1).hex(), 16) & 0xff
                
                alt_sign = alt_1 >> 7
                target_sign = target_1 >> 7
                error_sign = error_1 >> 7
                value_sign = value_1 >> 7

                alt = alt_1 << 24 | alt_2 << 16 | alt_3 << 8 | alt_4
                target = target_1 << 24 | target_2 << 16 | target_3 << 8 | target_4
                error = error_1 << 24 | error_2 << 16 | error_3 << 8 | error_4
                value = value_1 << 24 | value_2 << 16 | value_3 << 8 | value_4

                if alt_sign == 1: alt = (alt & 0x7fffffff) - 2 ** 31
                if target_sign == 1 : target = (target & 0x7fffffff) - 2 ** 31
                if error_sign == 1: error = (error & 0x7fffffff) - 2 ** 31
                if value_sign == 1: value = (value & 0x7fffffff) - 2 ** 31
                
                print(alt , target , error , value)
                alt_li.append(alt)
                target_li.append(target)
                error_li.append(error)
                value_li.append(value)

fig = plt.figure()
# h , w 간격 조절
fig.subplots_adjust(hspace=0.4, wspace=0.2)

ax = plt.subplot(221, xlim=(0, 50), ylim=(-1000, 1000))
ax.set_title("altitude")
ax_2 = plt.subplot(222, xlim=(0, 50), ylim=(-1000, 1000))
ax_2.set_title("target")
ax_3 = plt.subplot(223, xlim=(0, 50), ylim=(-1000, 1000))
ax_3.set_title("error")
ax_4 = plt.subplot(224, xlim=(0, 50), ylim=(-2100, 15000))
ax_4.set_title("something you want")

# 화면에 보여질 점의 개수 ( 길이 )
max_points = 50

line, = ax.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=2)

line_2, = ax_2.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)

line_3, = ax_3.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)

line_4, = ax_4.plot(np.arange(max_points), 
                np.ones(max_points, dtype=np.float)*np.nan, lw=1,ms=1)
def init():
    return line,

def init_2():
    return line_2

def init_3():
    return line_3

def init_4():
    return line_4

def animate(i):
    global alt
    old_y = line.get_ydata()
    new_y = np.r_[old_y[1:], alt]
    line.set_ydata(new_y)
    return line,

def animate_2(i):
    global target
    old_y_2 = line_2.get_ydata()
    new_y_2 = np.r_[old_y_2[1:], target]
    line_2.set_ydata(new_y_2)

    return line_2

def animate_3(i):
    global error
    y_3 = random.randint(0,512)
    old_y_3 = line_3.get_ydata()
    new_y_3 = np.r_[old_y_3[1:], error]
    line_3.set_ydata(new_y_3)
    return line_3

# value를 다른 값으로 변경하여 사용
def animate_4(i):
    global value
    y_4 = random.randint(0,512)
    old_y_4 = line_4.get_ydata()
    new_y_4 = np.r_[old_y_4[1:], value]
    line_4.set_ydata(new_y_4)
    return line_4

thread1 = threading.Thread(target = connect)
thread1.start()


# frame -> 반복문에 들어가는 i의 값 , 의미없음 / interval = 실행단위 100ms
anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=100, blit=False)
anim_2 = animation.FuncAnimation(fig, animate_2  , init_func= init_2 ,frames=200, interval=100, blit=False)
anim_3 = animation.FuncAnimation(fig, animate_3  , init_func= init_3 ,frames=200, interval=100, blit=False)
anim_4 = animation.FuncAnimation(fig, animate_4  , init_func= init_4 ,frames=200, interval=100, blit=False)
plt.show()

