#--
# This is can_control_basic but tripped down to just serial and CAN
#--
import numpy as np

import time

import can
from time import sleep
import serial
import json
import pandas as pd
import datetime

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2



def initialize():
    bus1 = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)
    bus0 = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

    return (bus0, bus1)

def initializeLogging():
    dataframe = pd.DataFrame()
    return dataframe

def log(speed, steering, control, direction,
		      radio_control, serial_connection, dataframe):

    time = datetime.datetime.utcnow()

    data = [[speed, steering, control, direction,
             radio_control, serial_connection,time]]

    df_temp = pd.DataFrame(data, columns=["Speed", "Steering",
              "Control Enabled","Direction", "Radio Enabled",
              "Serial Active","Time Stamp"])

    dataframe = dataframe.append(df_temp)
    return dataframe

def saveLog(dataframe, filename):
    dataframe.to_excel(filename, sheet_name='Autonomous Testing')
    # dataframe.to_csv("testing_log.csv", encoding='utf-8', index=False)

def initializeCanLogging():
    dataframe = pd.DataFrame()
    return dataframe

def logCan(bus, dataframe):

    time = datetime.datetime.utcnow()
    try:
        msg = bus.recv(timeout=0.001)
        if msg is not None:
            data = [[msg.arbitration_id, msg.data, msg.dlc,time]]
            df_temp = pd.DataFrame(data, columns=["Arbitration ID", "Data","DLC", "Time Stamp"])
            dataframe = dataframe.append(df_temp)
    except KeyboardInterrupt:
        exit()

    return dataframe

def saveCanLog(dataframe, filename):
    dataframe.to_excel(filename, sheet_name='Autonomous Testing - Can Bus')
    # dataframe.to_csv("testing_log.csv", encoding='utf-8', index=False)

def estop(bus, estop_flag):
    
    try:
        msg = bus.recv(timeout=0.01)
        if msg is not None:
            print("The arbitration id is " + str(msg.arbitration_id))
            print("The message is " + str(msg.data))
            print("The message length is " + str(len(msg)))

            if msg.arbitration_id is 4:
                print("What is Can 4?")
                print("The Estop variable at 4 is: " + str(estop_flag))
                return estop_flag

            if msg.arbitration_id is 136: 
                if msg.data[0] is 0 and msg.dlc is 4:
                    estop_flag = True
                elif msg.data[0] is 1 and msg.dlc is 4:
                    print("E-stop off")
                    estop_flag = False
            

    except KeyboardInterrupt:
        exit()
    
    return estop_flag
    

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
            print(jsonEncoded)
            speed = jsonEncoded["Throttle"]
            steering = jsonEncoded["Steering"]
            mode = jsonEncoded["Mode"]

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
    #ACCEL_ID = 0x90
    ACCEL_ID = 0x144
    if speed < -0 or speed < 10:
        speed = 0

    msg = can.Message(arbitration_id=ACCEL_ID,
                      data=[enable, maxSpeed, speed],
                      is_extended_id=False)
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
    if steering < -0:
       steering = 0

    msg = can.Message(arbitration_id=STEER_ID,
                      data=[enable, steering],
                      is_extended_id=False)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")

    return

def main():
    filename = "log_data/testing_log" + str(datetime.datetime.now()) + ".xls"
    filename_can = "log_data/can_testing_log" + str(datetime.datetime.now()) + ".xls"
    joystick_enable = False
    control = True
    mode = 0;
    direction = "0"
    speed = 0
    max_speed = 0
    steering = 0
    accel_enable = False
    log_enable = False
    can_log_enable = True
    radio_control = True
    log_data = None
    can_log_data = None
    global estop_flag
    estop_flag = False
    if(radio_control is True):
        serial_connection = initializeSerial()

    (bus1, bus0) = initialize()


    if log_enable is True:
        log_data = initializeLogging()
    if can_log_enable is True:
        can_log_data = initializeCanLogging()


    print("Initializing..")

    startTick = 0;
    reverseTick = 0;
    try:
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

            estop_flag = estop(bus0, estop_flag) 

            if estop_flag is True:
                print("E-stop on")
                mode = 0
                speed = 0

            print("Speed: " + str(speed) + "     Steering: " + str(steering) + "    Mode: " + str(mode))

            if log_enable is True:
                log_data = log(speed, steering, control, direction,radio_control, serial_connection, log_data)

            if can_log_enable is True:
                can_log_data = logCan(bus1, can_log_data)

            max_speed = 64

            sendCanMSG(bus=bus1, maxSpeed=max_speed, speed=speed, steering=steering, enable=mode)#enable_acel
            sendCanSteeringMSG(bus=bus0, steering=steering, enable=mode)

            time.sleep(0.01)

    except KeyboardInterrupt:
        if log_enable is True:
            saveLog(log_data, filename)

        if can_log_enable is True:
            saveCanLog(can_log_data, filename_can)

        exit()


# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
