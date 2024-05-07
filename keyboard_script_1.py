import time
import evdev
from evdev import InputDevice, ecodes
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import threading

GPIO.setwarnings(False)

LCD_COLUMNS = 16
LCD_ROWS = 2

lcd = CharLCD('PCF8574', 0x27, cols=LCD_COLUMNS, rows=LCD_ROWS)

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
    
    movement_status = "Moving Forward"
    
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
    
    movement_status = "Moving Backward"
    
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
    
    movement_status = "Moving Left"

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
    
    movement_status = "Moving Right"
    
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
    
    movement_status = "Turning Right"
    
    
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
    
    movement_status = "Turning Left"
    
def stop():
    global movement_status
    
    front_right_pwm.ChangeDutyCycle(0)
    front_left_pwm.ChangeDutyCycle(0)
    back_left_pwm.ChangeDutyCycle(0)
    back_right_pwm.ChangeDutyCycle(0)
    
    movement_status = "Hello World"
    
#device = InputDevice('/dev/input/event3')
    
device = InputDevice('/dev/input/event0')

def handle_movement():
    #track state of each directional key
    direction_pressed = {
        ecodes.KEY_W: False,
        ecodes.KEY_S: False,
        ecodes.KEY_A: False,
        ecodes.KEY_D: False,
        ecodes.KEY_LEFT: False,
        ecodes.KEY_RIGHT: False
    }
    
    for event in device.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.code in direction_pressed:
                if event.value == 1:
                    direction_pressed[event.code] = True
                    if any(direction_pressed.values()):
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
                    if not any(direction_pressed.values()):
                        stop()

    
def update_lcd():
    global movement_status
    prev_status = ""
    
    while True:
        if movement_status != prev_status:
            lcd.clear()
            if movement_status == "Hello World":
                lcd.write_string("     Hello\n\r     World!")
            elif movement_status == "Moving Forward":
                lcd.write_string("     Moving\n\r    Forward!")
            elif movement_status == "Moving Backward":
                lcd.write_string("     Moving\n\r    Backward!")
            elif movement_status == "Moving Left":
                lcd.write_string("     Moving\n\r     Left!")
            elif movement_status == "Moving Right":
                lcd.write_string("     Moving\n\r     Right!")
            elif movement_status == "Turning Left":
                lcd.write_string("    Turning\n\r     Left!")
            elif movement_status == "Turning Right":
                lcd.write_string("    Turning\n\r    Right!")
            prev_status = movement_status
        
        
movement_thread = threading.Thread(target=handle_movement)
movement_thread.start()


lcd_thread = threading.Thread(target=update_lcd)
lcd_thread.start()

# make sure threads finish before cleanup
movement_thread.join()
lcd_thread.join()

GPIO.cleanup()