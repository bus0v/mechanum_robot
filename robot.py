#Petr Busov
#This script governs the motion of the robot
import time
import sys
import tkinter as tk
import RPi.GPIO as GPIO
mode=GPIO.getmode()
print(str(mode))
#setup motor pins
#left
Lenb=2
Lin4=3
Lin3=4
Lin2=17
Lin1=27
Lena=22

Rena=11
Rin1=5
Rin2=6
Rin3=13
Rin4=19
Renb=26

GPIO.setmode(GPIO.BCM)
GPIO.setup(Lin1,GPIO.OUT)
GPIO.setup(Lin2,GPIO.OUT)
GPIO.setup(Lin3,GPIO.OUT)
GPIO.setup(Lin4,GPIO.OUT)
#GPIO.setup(Lena,GPIO.OUT)
#GPIO.setup(Lenb,GPIO.OUT)
GPIO.setup(Rin1,GPIO.OUT)
GPIO.setup(Rin2,GPIO.OUT)
GPIO.setup(Rin3,GPIO.OUT)
GPIO.setup(Rin4,GPIO.OUT)
#GPIO.setup(Rena,GPIO.OUT)
#GPIO.setup(Renb,GPIO.OUT)

GPIO.output(Lin1,GPIO.LOW)
GPIO.output(Lin2,GPIO.LOW)
GPIO.output(Lin3,GPIO.LOW)
GPIO.output(Lin4,GPIO.LOW)
GPIO.output(Rin1,GPIO.LOW)
GPIO.output(Rin2,GPIO.LOW)
GPIO.output(Rin3,GPIO.LOW)
GPIO.output(Rin4,GPIO.LOW)

# p=GPIO.PWM(Lena,1000)
# p.start(100)
# o=GPIO.PWM(Lenb,1000)
# o.start(100)
# b=GPIO.PWM(Renb,1000)
# b.start(100)
# c=GPIO.PWM(Rena,1000)
# c.start(100)

print("/n")
print("default speed and dir is low and forward")

run=True
delay=0.001
	
def turnon(pin):
    GPIO.output(pin,GPIO.HIGH)
    return
def turnoff(pin):
    GPIO.output(pin,GPIO.LOW)
    return
def motor(position,direction):
    #f for forward
    # b for bcakward
    if position==bl:
        first=Lin1
        second=Lin2
    elif position == fl:
        first=Lin3
        second=Lin4
    elif position == fr:
        first=Rin1
        second=Rin2
    elif position == br:
        first=Rin3
        second=Rin4
    else:
        print("invalid motor")

    if direction=="f":
        turnon(first)
        turnoff(second)
    elif direction=="b"
        turnon(second)
        turnoff(first)  
    else:
        print("badinput")
    return

GPIO.cleanup()
ptiny("GPIO cleaned up")