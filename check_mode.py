import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
import time
import subprocess

# GPIO setup
GPIO.setmode(GPIO.BCM)

switch = 5

GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Define LCD screen
I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

# LCD setup
lcd = CharLCD('PCF8574', I2C_ADDR, cols=I2C_NUM_COLS, rows=I2C_NUM_ROWS)

lcd.clear()
lcd.write_string("  NSCC Virtual\n\r Presence Robot")

time.sleep(2)


# Function to start controller script
def start_script():
    return subprocess.Popen(["/home/rodney/Desktop/virtual_presence/motor3.py"])


# Function to terminate controller script
def terminate_script(process):
    if process:
        process.terminate()
        

# Variable to determine if script is running
script_process = None

# Variable to determine if this is the first check
startup_check = True

try:
    while True:
        switch_state = GPIO.input(switch)
        
        # If switch is toggled on
        if switch_state == GPIO.HIGH:
            # If script is not running, start script
            if script_process is None:
                print("Starting Script...")
                script_process = start_script()
                
                lcd.clear()
                lcd.write_string("   Controller\n\r      Mode")

                time.sleep(0.95)
        # If switch is toggled off
        else:
            # If script is running, terminate script
            if script_process is not None:
                print("Terminating Script...")
                terminate_script(script_process)
                script_process = None
                
                lcd.clear()
                lcd.write_string("     Web App\n\r      Mode")

                time.sleep(0.95)
            else:
                if startup_check:
                    lcd.clear()
                    lcd.write_string("     Web App\n\r      Mode")

                    time.sleep(1.45)
                    
                    startup_check = False
                
        
        time.sleep(0.05) # Delay to debounce switch

# If Ctrl+C is pressed, terminate without error
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    if script_process is not None:
        terminate_script(script_process)