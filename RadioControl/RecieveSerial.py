from time import sleep
import serial
import json
#ser = serial.Serial('/dev/tty.usbmodem1d11', 9600) # Establish the connection on a specific port
#ser = serial.Serial('COM12', 9600) # Establish the connection on a specific port
ser = serial.Serial('/dev/ttyUSB0', 9600)
counter = 32 # Below 32 everything in ASCII is gibberish
while True:
     print("Starting Serial Communication")
     counter +=1
     ser.write(str.encode(chr(0))) 
     print(ser.readline());
     print("test")
     if ser.in_waiting > 0:
         inputJson = ser.readline() # Read the newest output from the Arduino
	 print(inputJson);
         if inputJson != None and inputJson[0] == 123:
             jsonEncoded = json.loads(str(inputJson.decode('utf-8')))
             print(jsonEncoded)

             #print(jsonEncoded["Throttle"])

     sleep(.005)
     if (counter == 255):
         counter = 32


#import serial.tools.list_ports as ls;
#print (list(ls.comports()));


# import sys
# import glob
# import serial
#
#
# def serial_ports():
#     """ Lists serial port names
#
#         :raises EnvironmentError:
#             On unsupported or unknown platforms
#         :returns:
#             A list of the serial ports available on the system
#     """
#     if sys.platform.startswith('win'):
#         ports = ['COM%s' % (i + 1) for i in range(256)]
#     elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
#         # this excludes your current terminal "/dev/tty"
#         ports = glob.glob('/dev/tty[A-Za-z]*')
#     elif sys.platform.startswith('darwin'):
#         ports = glob.glob('/dev/tty.*')
#     else:
#         raise EnvironmentError('Unsupported platform')
#
#     result = []
#     for port in ports:
#         try:
#             s = serial.Serial(port)
#             s.close()
#             result.append(port)
#         except (OSError, serial.SerialException):
#             pass
#     return result
#
#
# if __name__ == '__main__':
#     print(serial_ports())
