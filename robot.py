#Petr Busov
#This is the master script
#testing serial communication

import time
import sys
import RPi.GPIO as GPIO
import serial

if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    ser.flush()

while True:
    if ser.in_waiting > 0:
        line=ser.readline().decode('utf-8').rstrip()
        print(line)
GPIO.cleanup()
print("GPIO cleaned up")
