#--
# This is can_control_basic but tripped down to just serial and CAN
#--
#TODO:
'''
1. modify initialize to ros creating publishers and subscribers
2. modify control message from PYTHONCAN to ros can_msgs
3. change bus.send() to ros publishers publish
'''
import numpy as np
import time

import can
from time import sleep
import serial
import json

import rospy
from can_msgs.msg import *

def initialize_publisher():
    pub_can0 = rospy.Publisher('can0_', Frame, queue_size=10)
    pub_can1 = rospy.Publisher('can1_', Frame, queue_size=10)

    return (can0, can1)


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

def sendCanAcceleratorMSG(pub, maxSpeed, speed, steering, enable):
    #print("----------Throttle message------------------------------")
    ACCEL_ID = 0x90
    #ACCEL_ID = 0x144
    if speed < -0:
        speed = 0

    msg = Frame(id=ACCEL_ID,
                is_rtr=False,
                is_extended_id=False,
                dlc=3,
                data=[enable, maxSpeed, speed,0,0,0,0,0])

    pub.publish(msg)

    return

def sendCanSteeringMSG(pub, steering, enable):
    #print("----------Steering message------------------------------")
    STEER_ID = 0x33
    test = hex(STEER_ID)
    if steering < -0:
       steering = 0

    msg = Frame(id=STEER_ID,
                is_rtr=False,
                is_extended_id=False,
                dlc=3,
                data=[enable, steering,0,0,0,0,0,0])

    pub.publish(msg)

    return

def sendCanBrakingMSG(pub, braking, enable):
    #print("----------Steering message------------------------------")
    BRAKE_ID = 0x35
    test = hex(BRAKE_ID)
    if braking < -0:
       braking = 0

    msg = Frame(id=BRAKE_ID,
                is_rtr=False,
                is_extended_id=False,
                dlc=3,
                data=[enable, braking,0,0,0,0,0,0])

    pub.publish(msg)

    return

def initEstop(pub):
    #print("----------Steering message------------------------------")
    ESTOP_ID = 0x00
    test = hex(ESTOP_ID)
    enable = 0x01
    node_id = 0x00

    msg = Frame(id=ESTOP_ID,
                is_rtr=False,
                is_extended_id=False,
                dlc=3,
                data=[enable, node_id,0,0,0,0,0,0])

    pub.publish(msg)

    return

def main():

    #-------------------Initialise variable-------------------#
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

    print("Initializing..")
    #-------------------Initialise PYTON SERIAL-------------------#
    serial_connection = initializeSerial()

    #-------------------ROS initialisation-------------------#
    rospy.init_node('Remote RC control node', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    (pub_can0, pub_can1) = initialize_publisher()

    # required initialisation for REMOTE EMERGENCY SWITCH BOX
    initEstop(can1)

    while not rospy.is_shutdown():
        (speed,
         steering,
         braking,
         control,
         direction,
         mode) = user_control_loop( speed=speed,
                                    steering=steering,
					                braking=braking,
                                    control=control,
                                    direction=direction,
					                radio_control=radio_control,
  					                serial_connection=serial_connection)

        #-------------------DEBUG-------------------#                            
        #print("The speed is " + str(speed))
        #print("The max speed is " + str(max_speed))
        #print("Is Accel Enabled? " + str(accel_enable))
        print("Speed: " + str(speed) + "     Steering: " + str(steering) +  "     Braking: " + str(braking) +"    Mode: " + str(mode))

        max_speed = 127
        #-------------------CAN 0 control-------------------#
        #sendCanAcceleratorMSG(bus=bus0, maxSpeed=max_speed, speed=speed, steering=steering, enable=mode)#enable_acel

        #-------------------CAN 1 control-------------------#
        sendCanSteeringMSG(pub=pub_can1, steering=steering, enable=mode)
        sendCanBrakingMSG(pub=pub_can1, braking=braking, enable=mode)
        rate.sleep()

# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
