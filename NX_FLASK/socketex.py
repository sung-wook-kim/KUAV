from gimbalcmd import *
import keyboard
crc = '3334'
def dectohex(value):
    hexnum = hex(value)[2:]
    length = (len(hexnum))
    while length <= 3:
        hexnum = '0' + hexnum
        length = (len(hexnum))
    hexflip = fliphex(hexnum)
    return (hexflip)

ser = serialinit(ser_port='COM5')
safemode = responsetest(ser)
paramstore(safemode)
sleepmultipliercalc()
intervalcalc()

pitch = 1500
roll = 1500
yaw = 1500
key_cng = False
while True:
    pitch_hex = dectohex(pitch)
    roll_hex = dectohex(roll)
    yaw_hex = dectohex(yaw)
    cmd = bytes.fromhex('FA0612' + pitch_hex + roll_hex + yaw_hex + '3334')
    cmdexecute(cmd, sleep)
    key = keyboard.read_key()
    if key == 'a':
        yaw+=5 ; yaw = min(2000,max(yaw,1000)) ; key_cng = True
    elif key == 'd':
        yaw -= 5 ; yaw = min(2000, max(yaw, 1000)) ; key_cng = True
    elif key == 'w':
        pitch += 5 ; pitch = min(2000, max(pitch, 1000)) ; key_cng = True
    elif key == 's':
        pitch -= 5 ; pitch = min(2000, max(pitch, 1000)) ; key_cng = True
    if key_cng ==True:
        print(f"pitch, {pitch}, roll, {roll}, yaw, {yaw}")


