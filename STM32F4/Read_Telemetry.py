import serial
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import threading
import random
import time

ser = serial.Serial('COM7', 115200)

import pandas as pd

global alt, target, error
alt = 0
target = 0
error = 0


def connect():
    global alt, target, error
    i = 0

    alt_li = []
    target_li = []
    error_li = []
    while True:
        i += 1
        if i % 100 == 0:
            df = pd.DataFrame()
            df['alt'] = alt_li
            df['target'] = target_li
            df['error'] = error_li
            df.to_csv("data.csv")
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

                alt_sign = alt_1 >> 7
                target_sign = target_1 >> 7
                error_sign = error_1 >> 7

                alt = alt_1 << 24 | alt_2 << 16 | alt_3 << 8 | alt_4
                target = target_1 << 24 | target_2 << 16 | target_3 << 8 | target_4
                error = error_1 << 24 | error_2 << 16 | error_3 << 8 | error_4

                if alt_sign == 1: alt = (alt & 0x7fffffff) - 2 ** 31
                if target_sign == 1: target = (target & 0x7fffffff) - 2 ** 31
                if error_sign == 1: error = (error & 0x7fffffff) - 2 ** 31
                alt_li.append(alt)
                target_li.append(target)
                error_li.append(error)


fig = plt.figure()

ax = plt.subplot(311, xlim=(0, 50), ylim=(-100, 100))
ax_2 = plt.subplot(312, xlim=(0, 50), ylim=(-100, 100))
ax_3 = plt.subplot(313, xlim=(0, 50), ylim=(-100, 100))

max_points = 50
max_points_2 = 50
max_points_3 = 50
line, = ax.plot(np.arange(max_points),
                np.ones(max_points, dtype=np.float) * np.nan, lw=2)

line_2, = ax_2.plot(np.arange(max_points_2),
                    np.ones(max_points, dtype=np.float) * np.nan, lw=1, ms=1)

line_3, = ax_3.plot(np.arange(max_points_3),
                    np.ones(max_points, dtype=np.float) * np.nan, lw=1, ms=1)


def init():
    return line,


def init_2():
    return line_2


def init_3():
    return line_3


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
    y_3 = random.randint(0, 512)
    old_y_3 = line_3.get_ydata()
    new_y_3 = np.r_[old_y_3[1:], error]
    line_3.set_ydata(new_y_3)
    return line_3


thread1 = threading.Thread(target=connect)
thread1.start()

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=False)
anim_2 = animation.FuncAnimation(fig, animate_2, init_func=init_2, frames=200, interval=10, blit=False)
anim_3 = animation.FuncAnimation(fig, animate_3, init_func=init_3, frames=200, interval=10, blit=False)
plt.show()
