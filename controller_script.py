#!/usr/bin/env python3

"""
CronTab:https://www.makeuseof.com/how-to-run-a-raspberry-pi-program-script-at-startup/
Make HC06 work: https://dev.to/ivanmoreno/how-to-connect-raspberry-pi-with-hc-05-bluetooth-module-arduino-programm-3h7a
"""

import pygame
import time
import RPi.GPIO as GPIO
# Communicate with serial ports on Raspberry Pi.
import serial
import random
# Stuff for the LCD display.
from RPLCD.i2c import CharLCD
# import rf24 libraries
import sys
import time
import datetime
import logging
import os
from evdev import InputDevice, categorize, ecodes
import threading

GPIO.setwarnings(False)

# Button mapping for controller.
aBtn = 304
bBtn = 305
xBtn = 307
yBtn = 308

# Mapping for back-side buttons
l1Btn = 310
r1Btn = 311
l2Trig = 10
r2Trig = 9

# Mapping for Left Axis
lhaxis = 0
lvaxis = 1

# Mapping for right Axis
rhaxis = 2
rvaxis = 5

# Mapping for D-pad
dpadHorizontal = 16
dpadVertical = 17

dpadUp = "dpup"
dpadDown = "dpdn"
dpadLeft = "dplt"
dpadRight = "dprt"

# Click mode for l2 and r2 trig, which needs to be ignored.
clickL2Trig = 312
clickR2Trig = 313

# Define LCD screen
I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

lcd = CharLCD('PCF8574', I2C_ADDR, cols=I2C_NUM_COLS, rows=I2C_NUM_ROWS)

lcd.write_string("     Pairing\n\r    Device...")

con1ena = 10
con1in1 = 4
con1in2 = 14
con1in3 = 8
con1in4 = 11
con1enb = 25

con2ena = 13
con2in1 = 16
con2in2 = 19
con2in3 = 20
con2in4 = 26
con2enb = 21


GPIO.setmode(GPIO.BCM)

GPIO.setup(con1in1,GPIO.OUT)
GPIO.setup(con1in2,GPIO.OUT)
GPIO.setup(con1ena,GPIO.OUT)

GPIO.setup(con1in3,GPIO.OUT)
GPIO.setup(con1in4,GPIO.OUT)
GPIO.setup(con1enb,GPIO.OUT)

GPIO.setup(con2in1,GPIO.OUT)
GPIO.setup(con2in2,GPIO.OUT)
GPIO.setup(con2ena,GPIO.OUT)
# 
GPIO.setup(con2in3,GPIO.OUT)
GPIO.setup(con2in4,GPIO.OUT)
GPIO.setup(con2enb,GPIO.OUT)
# 
GPIO.output(con1in1,GPIO.LOW)
GPIO.output(con1in2,GPIO.LOW)
GPIO.output(con1ena,GPIO.LOW)

GPIO.output(con1in3,GPIO.LOW)
GPIO.output(con1in4,GPIO.LOW)
GPIO.output(con1enb,GPIO.LOW)

GPIO.output(con2in1,GPIO.LOW)
GPIO.output(con2in2,GPIO.LOW)
GPIO.output(con2ena,GPIO.LOW)

GPIO.output(con2in3,GPIO.LOW)
GPIO.output(con2in4,GPIO.LOW)
GPIO.output(con2enb,GPIO.LOW)

front_right_pwm = GPIO.PWM(con1ena, 1000)
front_right_pwm.start(0)
front_left_pwm = GPIO.PWM(con1enb, 1000)
front_left_pwm.start(0)
back_left_pwm = GPIO.PWM(con2ena, 1000)
back_left_pwm.start(0)
back_right_pwm = GPIO.PWM(con2enb, 1000)
back_right_pwm.start(0)

movement_status = "Hello World"

def forward():
    global movement_status
    
    #front right
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.HIGH)
    
    #front left
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.HIGH)
    
    #back left
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.HIGH)
    
    #back right
    GPIO.output(con2in3,GPIO.HIGH)
    GPIO.output(con2in4,GPIO.LOW)
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Move Forward"
    
def backward():
    global movement_status
    
    GPIO.output(con1in1,GPIO.HIGH)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.HIGH)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.HIGH)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.HIGH)
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Move Backward"
    
def strafe_left():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.HIGH)
   
    GPIO.output(con1in3,GPIO.HIGH)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.HIGH)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.HIGH)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Move Left"

def strafe_right():
    global movement_status
    
    GPIO.output(con1in1,GPIO.HIGH)
    GPIO.output(con1in2,GPIO.LOW)
   
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.HIGH)
    
    GPIO.output(con2in1,GPIO.HIGH)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.HIGH)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Move Right"
    
def turn_right():
    global movement_status
    
    GPIO.output(con1in1,GPIO.HIGH)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.HIGH)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.HIGH)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.HIGH)
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Turn Right"
    
    
def turn_left():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.HIGH)
    
    GPIO.output(con1in3,GPIO.HIGH)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.HIGH)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.HIGH)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Turn Left"
    
    
def forward_left():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.HIGH)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.HIGH)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Steer FL"
    

def forward_right():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.HIGH)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.HIGH)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Steer FR"
    
    
# def backward_left():
#     global movement_status
#     
#     GPIO.output(con1in1,GPIO.HIGH)
#     GPIO.output(con1in2,GPIO.LOW)
#     
#     GPIO.output(con1in3,GPIO.LOW)
#     GPIO.output(con1in4,GPIO.LOW)
#     
#     GPIO.output(con2in1,GPIO.LOW)
#     GPIO.output(con2in2,GPIO.LOW)
#     
#     GPIO.output(con2in3,GPIO.LOW)
#     GPIO.output(con2in4,GPIO.HIGH)
#     
#     
#     front_right_pwm.ChangeDutyCycle(100)
#     front_left_pwm.ChangeDutyCycle(100)
#     back_left_pwm.ChangeDutyCycle(100)
#     back_right_pwm.ChangeDutyCycle(100)
#     
#     movement_status = "Steer BL"
#     
#     
# def backward_right():
#     global movement_status
#     
#     GPIO.output(con1in1,GPIO.LOW)
#     GPIO.output(con1in2,GPIO.LOW)
#     
#     GPIO.output(con1in3,GPIO.HIGH)
#     GPIO.output(con1in4,GPIO.LOW)
#     
#     GPIO.output(con2in1,GPIO.HIGH)
#     GPIO.output(con2in2,GPIO.LOW)
#     
#     GPIO.output(con2in3,GPIO.LOW)
#     GPIO.output(con2in4,GPIO.LOW)
#     
#     
#     front_right_pwm.ChangeDutyCycle(100)
#     front_left_pwm.ChangeDutyCycle(100)
#     back_left_pwm.ChangeDutyCycle(100)
#     back_right_pwm.ChangeDutyCycle(100)
#     
#     movement_status = "Steer BR"
    

def diagonal_up_left():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.HIGH)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.HIGH)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Diagonal"
    
    
def diagonal_up_right():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.HIGH)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.HIGH)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Diagonal"
    

def diagonal_back_left():
    global movement_status
    
    GPIO.output(con1in1,GPIO.LOW)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.HIGH)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.LOW)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.HIGH)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Diagonal"
    
    
def diagonal_back_right():
    global movement_status
    
    GPIO.output(con1in1,GPIO.HIGH)
    GPIO.output(con1in2,GPIO.LOW)
    
    GPIO.output(con1in3,GPIO.LOW)
    GPIO.output(con1in4,GPIO.LOW)
    
    GPIO.output(con2in1,GPIO.HIGH)
    GPIO.output(con2in2,GPIO.LOW)
    
    GPIO.output(con2in3,GPIO.LOW)
    GPIO.output(con2in4,GPIO.LOW)
    
    
    front_right_pwm.ChangeDutyCycle(100)
    front_left_pwm.ChangeDutyCycle(100)
    back_left_pwm.ChangeDutyCycle(100)
    back_right_pwm.ChangeDutyCycle(100)
    
    movement_status = "Diagonal"
    
    
def stop():
    global movement_status
    
    front_right_pwm.ChangeDutyCycle(0)
    front_left_pwm.ChangeDutyCycle(0)
    back_left_pwm.ChangeDutyCycle(0)
    back_right_pwm.ChangeDutyCycle(0)
    
    movement_status = "Hello World"


# function to translate analog axis inputs to motor speeds
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


# Function to initialize joystick
print("Going to initialize controller ....")
gamepad = None
while gamepad is None:
    try:
        gamepad = InputDevice('/dev/input/event6')
    except Exception as ex:
        print("Waiting for 2 seconds ....")
        time.sleep(2)
        
deadzone = 0.02

def handle_movement():
    global movement_status
    
    #track state of each directional key
    direction_pressed = {
        yBtn: False,
        bBtn: False,
        xBtn: False,
        aBtn: False,
        
        l1Btn: False,
        r1Btn: False,
        
        dpadUp: False,
        dpadDown: False,
        dpadLeft: False,
        dpadRight: False
    }
    
    try:
        # An event loop in evdev is where the majority of the program happens.
        # This is where we take key inputs and make the robot do things.
        while True:
            for event in gamepad.read_loop():
                if gamepad is not None:
                    # If event is a button event:
                    if event.type == ecodes.EV_KEY:
                        # If a button is pressed down
                        if event.value == 1:
                            direction_pressed[event.code] = True
                            if any(direction_pressed.values()):
                                if direction_pressed[yBtn]:
                                    print("Y Button | Forward")
                                    #forward()
                                elif direction_pressed[aBtn]:
                                    print("A Button | Backward")
                                    #backward()
                                elif direction_pressed[xBtn]:
                                    print("X Button | Left")
                                    #strafe_left()
                                elif direction_pressed[bBtn]:
                                    print("B Button | Right")
                                    #strafe_right()
                                elif direction_pressed[l1Btn]:
                                    print("L Button | Turn Left")
                                    turn_left()
                                elif direction_pressed[r1Btn]:
                                    print("R Button | Turn Right")
                                    turn_right()
                        # If a button is released
                        elif event.value == 0:
                            direction_pressed[event.code] = False
                            if not any(direction_pressed.values()):
                                stop()
                                
                    # D-Pad and stick events
                    elif event.type == ecodes.EV_ABS:
                        print(event)                        
                        
                        #D-Pad Eeents
                        if event.code == dpadHorizontal:
                            if event.value < 0:
                                direction_pressed[dpadLeft] = True
                                if direction_pressed[dpadLeft]:
                                    print("DPAD Left | Left")
                                    strafe_left()
                            elif event.value > 0:
                                direction_pressed[dpadRight] = True
                                if direction_pressed[dpadRight]:
                                    print("DPAD Right | Right")
                                    strafe_right()
                            else:
                                direction_pressed[dpadLeft] = False
                                direction_pressed[dpadRight] = False
                                if not any (direction_pressed.values()):
                                    stop()
                                    
                        elif event.code == dpadVertical:
                            if event.value < 0:
                                direction_pressed[dpadUp] = True
                                if direction_pressed[dpadUp]:
                                    print("DPAD Up | Forward")
                                    forward()
                            elif event.value > 0:
                                direction_pressed[dpadDown] = True
                                if direction_pressed[dpadDown]:
                                    print("DPAD Down | Backward")
                                    backward()
                            else:
                                direction_pressed[dpadUp] = False
                                direction_pressed[dpadDown] = False
                                if not any (direction_pressed.values()):
                                    stop()
                                    
                        if direction_pressed[dpadUp] and direction_pressed[dpadRight]:
                            print("DPAD Up + DPAD Right | Forward Right")
                            diagonal_up_right()
                            
                        elif direction_pressed[dpadUp] and direction_pressed[dpadLeft]:
                            print("DPAD Up + DPAD Left | Forward Left")
                            diagonal_up_left()
                            
                        elif direction_pressed[dpadDown] and direction_pressed[dpadRight]:
                            print("DPAD Down + DPAD Right | Back Right")
                            diagonal_back_right()
                            
                        elif direction_pressed[dpadDown] and direction_pressed[dpadLeft]:
                            print("DPAD Down + DPAD Left | Back Left")
                            diagonal_back_left()
                        
                        
                        #Stick logic
                        elif event.code == lhaxis:
                            if abs(event.value - 128) > deadzone:
                                if event.value < 128:
                                    print("LEFT STICK Left | Left")
                                    strafe_left()
                                else:
                                    print("LEFT STICK Right | Right")
                                    strafe_right()
                            else:
                                stop()

                        elif event.code == lvaxis:
                            if abs(event.value - 128) > deadzone:
                                if event.value < 128:
                                    print("LEFT STICK Up | Up")
                                    forward()
                                else:
                                    print("LEFT STICK Down | Down")
                                    backward()
                            else:
                                stop()

                        elif event.code == rhaxis:
                            if abs(event.value - 128) > deadzone:
                                if event.value < 128:
                                    print("RIGHT STICK Left | Turn Left")
                                    turn_left()
                                else:
                                    print("RIGHT STICK Right | Turn Right")
                                    turn_right()
                            else:
                                stop()
                                
                                
                        # Trigger logic
                        if event.code == l2Trig:
                            if event.value > 128:
                                print("ZL Trigger | Steer Left")
                                forward_left()
                        elif event.code == r2Trig:
                            if event.value > 128:
                                print("ZR Trigger | Steer Right")
                                forward_right()

                    elif event.code == rvaxis:
                        pass
                    else:
                        pass
                else:
                    print("Gamepad is not connected ....")
                    logging.info("Gamepad is not connected ....")
    except Exception as ex:
        print(ex)
        quit()
        
def update_lcd():
    global movement_status
    prev_status = ""
    
    while True:
        if movement_status != prev_status:
            lcd.clear()
            if movement_status == "Hello World":
                lcd.write_string("     Hello\n\r     World!")
            elif movement_status == "Move Forward":
                lcd.write_string("     Moving\n\r    Forward!")
            elif movement_status == "Move Backward":
                lcd.write_string("     Moving\n\r    Backward!")
            elif movement_status == "Move Left":
                lcd.write_string("     Moving\n\r     Left!")
            elif movement_status == "Move Right":
                lcd.write_string("     Moving\n\r     Right!")
            elif movement_status == "Turn Left":
                lcd.write_string("    Turning\n\r     Left!")
            elif movement_status == "Turn Right":
                lcd.write_string("    Turning\n\r    Right!")
            elif movement_status == "Steer FR":
                lcd.write_string("    Steering\n\r     Right!")
            elif movement_status == "Steer FL":
                lcd.write_string("    Steering\n\r     Left!")
            elif movement_status == "Steer BR":
                lcd.write_string("   Reversing\n\r    Right!")
            elif movement_status == "Steer BL":
                lcd.write_string("   Reversing\n\r     Left!")
            elif movement_status == "Diagonal":
                lcd.write_string("     Moving\n\r   Diagonally!")
            prev_status = movement_status
            
            
movement_thread = threading.Thread(target=handle_movement)
movement_thread.start()


lcd_thread = threading.Thread(target=update_lcd)
lcd_thread.start()

# make sure threads finish before cleanup
movement_thread.join()
lcd_thread.join()

GPIO.cleanup()