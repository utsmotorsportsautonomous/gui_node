from time import sleep
import serial
import json
#ser = serial.Serial('/dev/tty.usbmodem1d11', 9600) # Establish the connection on a specific port
#ser = serial.Serial('COM12', 9600) # Establish the connection on a specific port
ser = serial.Serial('/dev/ttyUSB0', 9600)
counter = 32 # Below 32 everything in ASCII is gibberish
print("Starting Serial Communication")
while True:

    counter +=1
    ser.write(str.encode(chr(0))) 
    if ser.in_waiting > 0:
        inputJson = ser.readline() # Read the newest output from the Arduino
        if inputJson != None and inputJson[0] == 123: #123 = '{' ascii code
            jsonEncoded = json.loads(str(inputJson.decode('utf-8')))
            print(jsonEncoded)
            #print(jsonEncoded["Throttle"])

    sleep(.005)
    if (counter == 255):
        counter = 32

