#--
# This is can_control_basic but tripped down to just serial and CAN
#--
import numpy as np

import time

import can
from time import sleep
import serial
import json


def initialize():
    bus1 = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)
    bus0 = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

    return (bus0, bus1)


def initializeSerial():
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    return ser

def translate_remote_controls(ser):
    speed = 0
    steering = 0
    braking=0
    mode = 0

    ser.write(str.encode(chr(0)))

    if (ser.in_waiting > 0):
        inputJson = ser.readline() # Read the newest output from the Arduino
        if inputJson != None and inputJson[0] == 123: #123 = '{' ascii code
            jsonEncoded = json.loads(str(inputJson.decode('utf-8')))
            #print(jsonEncoded)
            speed = jsonEncoded["Throttle"]
            steering = jsonEncoded["Steering"]
            braking =jsonEncoded["Braking"]
            mode = jsonEncoded["Mode"]

    sleep(.005)

    return (speed, steering, braking, mode)

def user_control_loop(speed, steering,braking, control, direction, 
		      radio_control, serial_connection):
    mode = 0
    axis0 = 0  # Left / Right on left joystick
    # axis1 = 0  # Up / Dpwn on left joystick
    axis2 = 0  # R2 / L2 L2 is positive, R2 is negative.
    axis5 = 0
    up = False
    down = False
    left = False
    right = False

    if control is True:
        if radio_control is True:
            (speed, steering,braking, mode) = translate_remote_controls(serial_connection)
    

    return (speed, steering, braking, control, direction, mode)

def sendCanAcceleratorMSG(bus, maxSpeed, speed, steering, enable):
    #print("----------Throttle message------------------------------")
    ACCEL_ID = 0x90
    #ACCEL_ID = 0x144
    if speed < -0:
        speed = 0

    msg = can.Message(arbitration_id=ACCEL_ID,
                      data=[enable, maxSpeed, speed],
                      is_extended_id=False)
    #print(msg.data)
    try:
        bus.send(msg)
        #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def sendCanSteeringMSG(bus, steering, enable):
    #print("----------Steering message------------------------------")
    STEER_ID = 0x33
    test = hex(STEER_ID)
    if steering < -0:
       steering = 0

    msg = can.Message(arbitration_id=STEER_ID,
                      data=[enable, steering],
                      is_extended_id=False)
    #print(msg.data)
    try:
        bus.send(msg)
        #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def sendCanBrakingMSG(bus, braking, enable):
    #print("----------Steering message------------------------------")
    BRAKE_ID = 0x35
    test = hex(BRAKE_ID)
    if braking < -0:
       braking = 0

    msg = can.Message(arbitration_id=BRAKE_ID,
                      data=[enable, braking],
                      is_extended_id=False)
    #print(msg.data)
    try:
        bus.send(msg)
        #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def initEstop(bus):
    #print("----------Steering message------------------------------")
    ESTOP_ID = 0x00
    test = hex(ESTOP_ID)
    enable = 0x01
    node_id = 0x00

    msg = can.Message(arbitration_id=ESTOP_ID,
                      data=[enable, node_id],
                      is_extended_id=False)
    #print(msg.data)
    try:
        bus.send(msg)
        #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def main():

    joystick_enable = False
    control = True
    mode = 0;
    direction = "0"
    speed = 0
    max_speed = 0
    steering = 0
    braking=0
    accel_enable = False
    radio_control = True;   
    serial_connection = initializeSerial()

    (bus0, bus1) = initialize()
    print("Initializing..")
    startTick = 0
    reverseTick = 0
    initEstop(bus1)
    while True:
        startTick = startTick + 1
        start_time = time.time()
        (speed,
         steering,
	 braking,
         control,
         direction, mode) = user_control_loop(
                                        speed=speed,
                                        steering=steering,
					braking=braking,
                                        control=control,
                                        direction=direction,
					radio_control=radio_control,
  					serial_connection=serial_connection)

        #print("The speed is " + str(speed))
        #print("The max speed is " + str(max_speed))
        #print("Is Accel Enabled? " + str(accel_enable))
        print("Speed: " + str(speed) + "     Steering: " + str(steering) +  "     Braking: " + str(braking) +"    Mode: " + str(mode))

        max_speed = 127
        #sendCanAcceleratorMSG(bus=bus0, maxSpeed=max_speed, speed=speed, steering=steering, enable=mode)#enable_acel
        sendCanSteeringMSG(bus=bus1, steering=steering, enable=mode)
        sendCanBrakingMSG(bus=bus1, braking=braking, enable=mode)
        time.sleep(0.005)

# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
