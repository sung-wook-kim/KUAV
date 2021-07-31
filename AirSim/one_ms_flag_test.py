import airsim
import threading
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
    threading.Timer(0.05, print_state, args=[client]).start()

print_state(client)

