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
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
    return bus


def initializeSerial():
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    return ser

def translate_remote_controls(ser):
    speed = 0
    steering = 0
    mode = 0

    ser.write(str.encode(chr(0)))

    if (ser.in_waiting > 0):
        inputJson = ser.readline() # Read the newest output from the Arduino
        if inputJson != None and inputJson[0] == 123: #123 = '{' ascii code
            jsonEncoded = json.loads(str(inputJson.decode('utf-8')))
            #print(jsonEncoded)
            speed = jsonEncoded["Throttle"]
            steering = jsonEncoded["Steering"]
            mode = jsonEncoded["Mode"]
    #sleep(.05) #temporary
    sleep(.005)

    return (speed, steering, mode)

def user_control_loop(speed, steering, control, direction, 
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
            (speed, steering, mode) = translate_remote_controls(serial_connection)
    

    return (speed, steering, control, direction, mode)

def sendCanMSG(bus, maxSpeed, speed, steering, enable):
    print("----------Throttle message------------------------------")
    ACCEL_ID = 0x144
    if speed < -255:
        speed = -255

    msg = can.Message(arbitration_id=ACCEL_ID,
                      data=[enable, maxSpeed, speed],
                      is_extended_id=False)
    print(msg.data)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def sendCanSteeringMSG(bus, steering, enable):
    print("----------Steering message------------------------------")
    STEER_ID = 0x33
    test = hex(STEER_ID)
    if steering < -255:
       steering = -255

    msg = can.Message(arbitration_id=STEER_ID,
                      data=[enable, steering],
                      is_extended_id=False)
    print(msg.data)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
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
    accel_enable = False
    radio_control = True;
    if(radio_control is True):
        serial_connection = initializeSerial()

    bus = initialize()
    print("Initializing..")
    startTick = 0;
    reverseTick = 0;
    while True:
        startTick = startTick + 1
        start_time = time.time()
        (speed,
         steering,
         control,
         direction, mode) = user_control_loop(
                                        speed=speed,
                                        steering=steering,
                                        control=control,
                                        direction=direction,
					radio_control=radio_control,
  					serial_connection=serial_connection)

        #print("The speed is " + str(speed))
        #print("The max speed is " + str(max_speed))
        #print("Is Accel Enabled? " + str(accel_enable))
        print("Speed: " + str(speed) + "     Steering: " + str(steering) + "    Mode: " + str(mode))

        
        sendCanMSG(bus=bus, maxSpeed=max_speed, speed=speed, steering=steering, enable=accel_enable)
        sendCanSteeringMSG(bus=bus, steering=steering, enable=mode)

# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
