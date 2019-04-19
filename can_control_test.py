
import numpy as np
# import imutils
# import Camera
import cv2

import time
import interface
import can
from time import sleep
import serial
import json


def initialize():

    main_interface = interface.interface()
    main_interface.start_screen()
    main_interface.init_joystick()
    joystick_count = main_interface.get_joystick_count()
    main_interface.create_menu()
    bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)

    return (main_interface, joystick_count, bus)


def initializeSerial():
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    return ser
    
def translate_key_controls(control, up, down, left, right, speed, steering, direction):
    if(control):
        if(up):
            speed = 56
            direction = "1"
        elif(down):
            speed = 60
            direction = "2"
        else:
            speed = 0
            direction = "0"
        if(left):
            steering = "35"
        elif (right):
            steering = "145"
        else:
            steering = "90"

    return (speed, steering, direction)


def translate_stick_controls(axis0, axis1, axis2,
                             axis3, axis4, axis5, speed, steering):
    #
    # if(abs(axis0) > 0.1):
    #     steering = str(90 + 40 * axis0)
    # if(abs(axis5 + 1) > 0.1):
    #     speed = 1430 - 20 * (axis5 + 1)
    # elif(abs(axis2 + 1) > 0.1):
    #     speed = 1570 + 20 * (axis2 + 1)

    return (speed, steering)




def translate_remote_controls(ser):
    speed = 0
    steering = 0
    ser.write(str.encode(chr(0)))
    if ser.in_waiting > 0:
        inputJson = ser.readline() # Read the newest output from the Arduino
        if inputJson != None and inputJson[0] == 123: #123 = '{' ascii code
            jsonEncoded = json.loads(str(inputJson.decode('utf-8')))
            #print(jsonEncoded)
            speed = jsonEncoded["Throttle"]
            steering = jsonEncoded["Steering"]
    sleep(.005)
    return (speed, steering)

def user_control_loop(window, joystick_enable, speed, steering, control, direction, 
		      radio_control, serial_connection):
    axis0 = 0  # Left / Right on left joystick
    # axis1 = 0  # Up / Dpwn on left joystick
    axis2 = 0  # R2 / L2 L2 is positive, R2 is negative.
    axis5 = 0
    up = False
    down = False
    left = False
    right = False

    joystick_count = window.get_joystick_count()


    if control is True:
        (up, down, left, right, control) = window.get_key_input()

        if radio_control is True:
	    (spped, steering) = translate_remote_controls(serial_connection)
	
        elif joystick_count is not 0 and joystick_enable is True:
            (axis0,
             axis1,
             axis2,
             axis3,
             axis4,
             axis5) = window.get_joystick_input(joystick_num=0)
            (speed, steering) = translate_stick_controls(axis0=axis0,
                                                          axis1=axis1,
                                                          axis2=axis2,
                                                          axis3=axis3,
                                                          axis4=axis4,
                                                          axis5=axis5,
                                                          speed=speed,
                                                          steering=steering)

        elif joystick_count is 0 or joystick_enable is False:
            (speed,
             steering,
             direction) = translate_key_controls(control=control,
                                                 up=up,
                                                 down=down,
                                                 left=left,
                                                 right=right,
                                                 speed=speed,
                                                 steering=steering,
                                                 direction=direction)

    return (speed, steering, control, direction)

def sendCanMSG(bus, maxSpeed, speed, steering, enable):
    ACCEL_ID = 0x144
    msg = can.Message(arbitration_id=ACCEL_ID,
                      data=[enable, maxSpeed, speed],
                      is_extended_id=True)
    print(msg.data)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return


def main():

    joystick_enable = False
    control = False
    direction = "0"
    speed = 0
    max_speed = 0
    steering = 0
    accel_enable = False
    # buttonA = 0 # A Button
    # buttonY = 0 # Y Button
    main_interface = None
    display = cv2.imread('doge.jpg')
    radio_control = True;
    if(radio_control is True):
        serial_connection = initializeSerial()

    (main_interface,
     joystick_count, bus) = initialize()

    print("Initializing..")
    startTick = 0;
    reverseTick = 0;
    while True:
        startTick = startTick + 1
        start_time = time.time()

        accel_enable = main_interface.get_accel_enable()
        speed = main_interface.get_accel_value()
        max_speed = main_interface.get_maxaccel_value()


        (speed,
         steering,
         control,
         direction) = user_control_loop(window=main_interface,
                                        joystick_enable=joystick_enable,
                                        speed=speed,
                                        steering=steering,
                                        control=control,
                                        direction=direction,
					radio_control=radio_control,
					serial_connection=serial_connection)

        print("The speed is " + str(speed))
        print("The max speed is " + str(max_speed))
        print("Is Accel Enabled? " + str(accel_enable))

        main_interface.update_frame(display)

        if main_interface.exit_check():
            speed = 0
            steering = "180"
            message = ("{\"Mode\" : \"Drive\"," +
                       "\"Throttle\" : \"" + str(speed) + "\"," +
                       "\"Direction\" : \"0\"," +
                       "\"Steering\" : \"" + str(steering) + "\"}")
            exit()

        main_interface.process_events()
        sendCanMSG(bus=bus, maxSpeed=max_speed, speed=speed, steering=steering, enable=accel_enable)


# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
