import airsim
import threading
from threading import Timer
import time

import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

def print_state(client):
    #print(client.getMultirotorState().kinematics_estimated.position.x_val, time.time())
    # threading.Timer(0.001, print_state, args=[client]).start()

    print(time.time())
    threading.Timer(0.0001, print_state, args=[client]).start()

def nextfunction():
  print(f'{time.time()}')

t = Timer(0.5, nextfunction)
t.start()

while 1:
    pass