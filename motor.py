import time
import evdev
from evdev import InputDevice, ecodes
from rpi_lcd import LCD
import RPi.GPIO as GPIO

GPIO.setwarnings(False)

lcd = LCD()

con1ena = 13
con1in1 = 16
con1in2 = 19
con1in3 = 20
con1in4 = 26
con1enb = 21

con2ena = 10
con2in1 = 4
con2in2 = 14
con2in3 = 8
con2in4 = 11
con2enb = 25

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

GPIO.setup(con2in3,GPIO.OUT)
GPIO.setup(con2in4,GPIO.OUT)
GPIO.setup(con2enb,GPIO.OUT)

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

def forward():
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
   
   lcd.text("     Moving", 1)
   lcd.text("    Forward!", 2)
   
def backward():
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
   
   lcd.text("     Moving", 1)
   lcd.text("    Backward!", 2)
   
def strafe_left():
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
   
   lcd.text("     Moving", 1)
   lcd.text("     Left!", 2)

def strafe_right():
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
   
   lcd.text("     Moving", 1)
   lcd.text("     Right!", 2)
   
def turn_right():
   print("Turn right")
   
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
   
   lcd.text("    Turning", 1)
   lcd.text("     Right!", 2)
   
   
def turn_left():
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
   
   lcd.text("    Turning", 1)
   lcd.text("    Left!", 2)
   
def stop():
   front_right_pwm.ChangeDutyCycle(0)
   front_left_pwm.ChangeDutyCycle(0)
   back_left_pwm.ChangeDutyCycle(0)
   back_right_pwm.ChangeDutyCycle(0)
   
   lcd.text("     Hello", 1)
   lcd.text("     World", 2)
   
#device = InputDevice('/dev/input/event1')
   
device = InputDevice('/dev/input/event0')

#track state of each directional key
direction_pressed = {
   ecodes.KEY_W: False,
   ecodes.KEY_S: False,
   ecodes.KEY_A: False,
   ecodes.KEY_D: False,
   ecodes.KEY_LEFT: False,
   ecodes.KEY_RIGHT: False
}

#check if any directional key is pressed
def any_direction_pressed():
   return any(direction_pressed.values())

lcd.text("     Hello", 1)
lcd.text("     World", 2)

for event in device.read_loop():
   if event.type == ecodes.EV_KEY:
       if event.code in direction_pressed:
           if event.value == 1:
               direction_pressed[event.code] = True
               if any_direction_pressed():
                   if direction_pressed[ecodes.KEY_W]:
                       forward()
                   elif direction_pressed[ecodes.KEY_S]:
                       backward()
                   elif direction_pressed[ecodes.KEY_A]:
                       strafe_left()
                   elif direction_pressed[ecodes.KEY_D]:
                       strafe_right()
                   elif direction_pressed[ecodes.KEY_LEFT]:
                       turn_left()
                   elif direction_pressed[ecodes.KEY_RIGHT]:
                       turn_right()
               else:
                   stop()
           elif event.value == 0:
               direction_pressed[event.code] = False
               if not any_direction_pressed():
                   stop()

GPIO.cleanup()