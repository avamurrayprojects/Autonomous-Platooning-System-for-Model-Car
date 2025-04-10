import threading
import time
import pigpio
import ControlSystem0204
import sys
import os
from datetime import datetime

SWITCH_PIN = 17  
pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

pi.set_mode(SWITCH_PIN, pigpio.INPUT)
pi.set_pull_up_down(SWITCH_PIN, pigpio.PUD_UP)

# Reference to the follower instance
follower = None
follower_thread = None
log_file = None
original_stdout = sys.stdout

def start_follower():
    global follower, follower_thread, log_file, original_stdout
    if follower is None:
        print("Switch ON - Starting Control System...")
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = f"follower_log_{timestamp}.log"
        log_path = os.path.join("logs", log_filename)
        os.makedirs("logs", exist_ok=True)
        log_file = open(log_path, "w")
        sys.stdout = log_file
        
        print(f"[{timestamp}] Switch ON - Starting Control System...")
        
        follower = ControlSystem0204.PlatoonFollower(pigpio_instance=pi)
        follower_thread = threading.Thread(target=follower.run, daemon=True)
        follower_thread.start()
    else:
        print("Follower already running")

def stop_follower():
    global follower, follower_thread, log_file, original_stdout
    if follower is not None:
        print("Switch OFF - Stopping Control System...")
        follower.stop()
        follower_thread.join()  # Ensure the thread fully stops
        follower = None
        follower_thread = None
        time.sleep(0.5)
        
        #close log and restore stdout
        sys.stdout = original_stdout
        if log_file:
            log_file.close()
            log_file = None
    else:
        print("No follower to stop")

try:
    last_state = pi.read(SWITCH_PIN)  # Track last switch state
    debounce_time = 0.2  # 100ms debounce delay

    while True:
        current_state = pi.read(SWITCH_PIN)

        if current_state != last_state:  # Only trigger on state change
            time.sleep(debounce_time)  # Debounce delay
            current_state = pi.read(SWITCH_PIN)  # Read again to confirm
            print("Switch state changed")

            if current_state == 0 and follower is None:  # Switch turned ON
                print("Switch ON detected.")
                start_follower()
            elif current_state == 1 and follower is not None:  # Switch turned OFF
                print("Switch OFF detected.")
                stop_follower()

            last_state = current_state  # Update last state

        time.sleep(0.1)  # Small delay to reduce CPU usage

except KeyboardInterrupt:
    print("Exiting...")
    stop_follower()
    pi.stop()
    print("Cleanup complete.")
