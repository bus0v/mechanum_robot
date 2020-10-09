#Petr Busov
#This script governs the motion of the robot
import time
import sys
import tkinter as tk
import RPi.GPIO as GPIO
mode=GPIO.getmode()
print(str(mode))
en=5
in3=16
in4=20
temp1=1
enb=21
in1=6
in2=13
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p=GPIO.PWM(en,1000)
p.start(50)
o=GPIO.PWM(enb,1000)
o.start(50)
print("/n")
print("default speed and dir is low and forward")

run=True
delay=0.001

def forward(delay):

	GPIO.output(in1,GPIO.LOW)
	GPIO.output(in2,GPIO.HIGH)
	GPIO.output(in3,GPIO.HIGH)
	GPIO.output(in4,GPIO.LOW)
	time.sleep(delay)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.LOW)
	GPIO.output(in1,GPIO.LOW)
	print("forward")

def backward(delay):
	GPIO.output(in1,GPIO.HIGH)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.LOW)
	GPIO.output(in1,GPIO.LOW)
	print("retreat")
	
def right(delay):
	GPIO.output(in1,GPIO.HIGH)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.HIGH)
	GPIO.output(in4,GPIO.LOW)
	time.sleep(delay)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.LOW)
	GPIO.output(in1,GPIO.LOW)
	print("right")
	 
def left(delay):

	GPIO.output(in1,GPIO.LOW)
	GPIO.output(in2,GPIO.HIGH)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3,GPIO.LOW)
	GPIO.output(in4,GPIO.LOW)
	GPIO.output(in1,GPIO.LOW)
	print("left")
	
while(run):
	 try:
	  k=key.char
	 except:
	  print("please use a wasdq characters")
	 if k=="w":
	 if k=="s":
	 if k=="q":
	 if k=="d":
	 if k=="a":
	  print("stop")
	  run=False
	 #GPIO.output(in1,GPIO.LOW)
	 #GPIO.output(in2,GPIO.LOW)
	 #GPIO.output(in3,GPIO.LOW)
	 #GPIO.output(in4,GPIO.LOW)	
GPIO.cleanup()
