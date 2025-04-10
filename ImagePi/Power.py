import RPi.GPIO as GPIO
import time
import subprocess
import os
import signal
import sys

# Configuration
BUTTON_GPIO = 4  # GPIO 4 = Pin 7
HOLD_TIME_SECONDS = 3  # Button must be held this long
DEBOUNCE_TIME_MS = 200  # For GPIO event
CHECK_PROCESS_NAME = "my_important_process"  # Set to None if not needed

# Helper Function

def shutdown(channel):
    print("Button press detected. Waiting for hold confirmation...")

    start_time = time.time()
    while GPIO.input(BUTTON_GPIO) == GPIO.LOW:
        if time.time() - start_time >= HOLD_TIME_SECONDS:
            print("Button held for required time.")

            print("Shutting down system now...")
            os.system("sudo shutdown now")
            return

        time.sleep(0.1)

    print("Button released too early. Shutdown canceled.")

def cleanup(signum=None, frame=None):
    print("Cleaning up GPIO and exiting.")
    GPIO.cleanup()
    sys.exit(0)

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=shutdown, bouncetime=DEBOUNCE_TIME_MS)

# Handle signals for clean exit
signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

print("Shutdown button monitoring started.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    cleanup()
