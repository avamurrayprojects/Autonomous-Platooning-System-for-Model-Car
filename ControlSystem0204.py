import threading
import time
import serial  
import pigpio
from collections import deque

    
                             
class PIDController:
   
       
    def __init__(self, kp, ki, kd, setpoint=160.0, sample_time=0.01):
        self.kp = kp #proportional
        self.ki = ki #Integral
        self.kd = kd #Derivative
        self.setpoint = setpoint #target value
        self.sample_time = sample_time #interval of computations in seconds
       
        self.last_marker_time =time.time()#timestamp for when marker was last seen
       

        # internal variables
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    #compute is used to compute output
    def compute(self, current_value):
       
        now = time.time()
        dt = now - self._last_time
        if dt < self.sample_time:
            # Not enough time has passed to compute
            return None

        error = self.setpoint - current_value
        self._integral += error * dt
        derivative = 0.0
        if dt > 0:
            derivative = (error - self._last_error) / dt

        # PID output
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        # Update for next round
        self._last_error = error
        self._last_time = now

        return output


#follower class
                             
class PlatoonFollower:
    #states are used by state machine to determine the mode of operation
    STATE_NOT_PLATOONING = "NOT_PLATOONING"
    STATE_SEARCHING = "SEARCHING"
    STATE_LOCKED_ON = "LOCKED_ON"
   
    def __init__(self,
                 uart_port='/dev/serial0', 
                 baud_rate=115200,
                 steering_pin=18,
                 trigger_pin=23,
                 echo_pin=24,
                 motor_pwm_pin=25,
                 motor_in1_pin=27,
                 motor_in2_pin=22,
                 pigpio_instance = None,
                 toggle_button_pin = 6,
                 GREEN_LED_PIN = 19,
                 YELLOW_LED_PIN = 26,
                 RED_LED_PIN= 13,
                 locked_on_time = None
                ):  
   

        # Configuration
        self.uart_port = uart_port #Serial port for communication   
        self.baud_rate = baud_rate #baud rate of data
        self.steering_pin = steering_pin #pin for servo
        self.trigger_pin = trigger_pin # pin for ultrasonic sensor trigger
        self.echo_pin = echo_pin #pin for ultrasonic echo
        self.motor_pwm_pin = motor_pwm_pin
        self.motor_in1_pin = motor_in1_pin
        self.motor_in2_pin = motor_in2_pin
        self.use_constant_speed = False
        self.constant_speed=30
        self.use_constant_steering = False
        self.override_steering = False
        self.searching_distance_start = None
        
        self.center_threshold = 70



        #following are for switching states
        self.last_button_state = 1 #starts unpressed and pulled high
        self.last_toggle_time = 0
        self.toggle_debounce = 0.25
        self.toggle_button_pin = toggle_button_pin

        self.servo_pulse =1500
        self.servo_dir = 1

        self.platooning_pin = self.toggle_button_pin

        self.state = self.STATE_NOT_PLATOONING #setting state to not platooning automatically
        self.sweeping = False

        self.marker_seen_buffer = deque(maxlen=10)  # buffer of last 10 frames

        self.steering_pid = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=160.0)
       
        

        # Shared state variables
        self.x_position = 0.0          # from serial
        self.distance_measured = 999.0 # from HC-SR04
        self.running = True            # for thread control
       
        self.previous_distance = None
        
        #PID setup for speed control
        self.distance_setpoint = 45.0
        self.speed_pid = PIDController(kp=0.1, ki= 0.0, kd = 0.11, setpoint=self.distance_setpoint, sample_time = 0.1)
        self.previous_speed = 0 
       
        #Pigpiod instance
        self.pi = pigpio_instance or pigpio.pi()
        self._own_pigpio = pigpio_instance is None #stop only if we created it
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon")

        # Initialize hardware
        self.setup_gpio()
        self._setup_port()
        self.median_buffer = deque(maxlen=6)
        self.median_usage_count = 0
        self.invalid_count = 0
        self.last_valid_distance = None
        
        #leds
        self.RED_LED =RED_LED_PIN
        self.YELLOW_LED =YELLOW_LED_PIN
        self.GREEN_LED =GREEN_LED_PIN
        
        self.pi.set_mode(self.RED_LED, pigpio.OUTPUT)
        self.pi.set_mode(self.YELLOW_LED, pigpio.OUTPUT)
        self.pi.set_mode(self.GREEN_LED, pigpio.OUTPUT)
        self.pi.write(self.RED_LED, 1)                     

         # Threads
        self.uart_thread = threading.Thread(target=self.uart_listener, daemon=True)
        self.steering_thread = threading.Thread(target=self.steering_control, daemon=True)
        self.speed_thread = threading.Thread(target=self.speed_control, daemon=True)
        self.state_thread = threading.Thread(target=self.state_machine_loop, daemon = True)


    # Hardware Setup                      
    def setup_gpio(self):
        #setting input and output pins
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon")
           
        # Set modes for servo and motor pins using pigpio
        self.pi.set_mode(self.steering_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.motor_pwm_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.motor_in1_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.motor_in2_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.trigger_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.echo_pin, pigpio.INPUT)

        # platooning toggling
        self.pi.set_mode(self.platooning_pin, pigpio.INPUT)
        self.pi.set_mode(self.toggle_button_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.toggle_button_pin, pigpio.PUD_UP)


        # For motor PWM
        self.pi.set_PWM_frequency(self.motor_pwm_pin, 100)
        
    def update_leds(self,state):
        self.pi.write(self.RED_LED, 0)
        self.pi.write(self.YELLOW_LED, 0)
        self.pi.write(self.GREEN_LED, 0)
    
        
        if state == self.STATE_NOT_PLATOONING:
                self.pi.write(self.RED_LED, 1)
        elif state == self.STATE_SEARCHING:
                self.pi.write(self.YELLOW_LED, 1)
        elif state == self.STATE_LOCKED_ON:
                self.pi.write(self.GREEN_LED, 1)


    def _setup_port(self):
    
        self.serial_conn = serial.serial_for_url(self.uart_port,
                                                 baudrate=self.baud_rate,
                                                 timeout=1)

    #UART listener                     
    def uart_listener(self):
        #print("uart listener called")
        #continously reading in data
        while self.running:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                #print("raw  line: {line}" )
                if line:
                    #print(f"UART REceived: {line}") #add back in after testing
                    # Expecting a string that represents a float for X position
                    try:
                        if line in ["NO_MARKER"]:
                            self.marker_visible = False
                        else:
                            self.x_position = float(line)
                            self.marker_visible = True
                            self.last_marker_time = time.time()

                        self.marker_seen_buffer.append(self.marker_visible)
                    except ValueError:
                         self.marker_visible = False

                    #print("Serial line: " + str(line))
            except Exception as e:
                                                   
                #print(f"[UART Listener] Exception: {e}")
                #sleep briefly before trying again
                time.sleep(0.1)
    def get_valid_median(self, max_age_sec=3.0):
        # Special case: if just entered LOCKED_ON, force fallback to 40
        if self.state == self.STATE_LOCKED_ON and self.locked_on_time is not None:
            if time.time() - self.locked_on_time < 1.0:
                return 40.0
            
        now = time.time()
        valid = [val for val, ts in self.median_buffer if now -ts <= max_age_sec]
        
        if not valid:
            return None
        
        valid.sort()
        mid = len(valid)//2
        if len(valid) % 2 == 0:
            return (valid[mid - 1] + valid[mid]) / 2
        else:
            return valid[mid]
        
    def handle_obstacle_during_searching(self, repeat_count=0):
        MAX_REPEATS = 3

        print("[SEARCHING] Obstacle too close for 2s — starting avoidance maneuver")
        self.override_steering = True
        self.pi.set_servo_pulsewidth(self.steering_pin, 1500)
        time.sleep(0.1)  # allow steering thread to pause

        # Reverse Phase
        print("[SEARCHING] Reversing...")
        self.pi.write(self.motor_in1_pin, 0)
        self.pi.write(self.motor_in2_pin, 1)
        reverse_duty = int(25 * 255 / 100)
        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, reverse_duty)

        reverse_start = time.time()
        while time.time() - reverse_start < 1:
            if not self.running or self.state != self.STATE_SEARCHING:
                break
            time.sleep(0.1)

        #Stop after reverse
        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, 0)
        self.pi.write(self.motor_in1_pin, 1)
        self.pi.write(self.motor_in2_pin, 1)
        time.sleep(0.1)
        self.pi.write(self.motor_in1_pin, 0)
        self.pi.write(self.motor_in2_pin, 0)

        #Check Before Forward 
        check_distance = self.distance_measured
        if check_distance < 30: 
            print(f"[SEARCHING] Aborting forward — obstacle still close ({check_distance:.2f} cm)")
            self.override_steering = False
            return

        #Forward Turn Phase
        print("[SEARCHING] Forward + steer right...")
        self.pi.set_servo_pulsewidth(self.steering_pin, 1700)  # steer left
        forward_duty = int(20 * 255 / 100)
        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, forward_duty)
        
        self.pi.write(self.motor_in1_pin, 1)
        self.pi.write(self.motor_in2_pin, 0)
        print(f"Forward duty: {forward_duty}, IN1=1, IN2=0")

        forward_start = time.time()
        while time.time() - forward_start < 1.5:
            if not self.running or self.state != self.STATE_SEARCHING:
                break

            current_distance = self.distance_measured
            if current_distance < 30:
                print(f"[SEARCHING] Obstacle during forward: {current_distance:.2f} cm — aborting")
                break

            time.sleep(0.1)

        # Stop motor and steering
        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, 0)
        self.pi.write(self.motor_in1_pin, 1)
        self.pi.write(self.motor_in2_pin, 1)
        time.sleep(0.1)
        self.pi.write(self.motor_in1_pin, 0)
        self.pi.write(self.motor_in2_pin, 0)
        self.pi.set_servo_pulsewidth(self.steering_pin, 1500)

        print("[SEARCHING] Maneuver complete — resuming normal operation")
        self.override_steering = False

        #Post-check: Repeat if still blocked
        post_distance = self.distance_measured
        print(f"[SEARCHING] Post-maneuver distance: {post_distance:.2f} cm")

        if post_distance < 30 and repeat_count < MAX_REPEATS and self.state == self.STATE_SEARCHING:
            print("[SEARCHING] Obstacle still close — repeating avoidance maneuver")
            self.handle_obstacle_during_searching(repeat_count + 1)
            



    #steering control
                                         
    #gets ran in its own seperate thread                    
    def steering_control(self):
        #1100us (left), 1500us (center), 1900us (right)
        center_pw = 1500  # Center pulse width
        min_pw = 1100     # Minimum pulse width
        max_pw = 1900     # Maximum pulse width
        max_pixel_error = 160  # Maximum pixel error (0 to 320 image, with center at 160)
        max_correction_us =200  # Maximum correction in microseconds (2% of 20ms period)
        #rate limter additions
        current_pw = center_pw
        max_servo_step = 20
       
        while self.running:
            try:
                if self.state == self.STATE_NOT_PLATOONING or self.override_steering:
                    time.sleep(0.1)
                    continue
                        
                       
                elif self.state == self.STATE_SEARCHING:
                    distance_measured = self.distance_measured
                    current_time = time.time()

                    # Trigger avoidance if object is too close
                    if not self.sweeping:
                        self.sweeping = True
                        self.servo_sweep()
                        self.sweeping = False
                    continue

                       
                elif self.state == self.STATE_LOCKED_ON:
                    if self.use_constant_steering:
                        self.pi.set_servo_pulsewidth(self.steering_pin, center_pw)
                        time.sleep(self.steering_pid.sample_time)
                        continue
                    
                    #test
                    if self.use_constant_steering:
                        self.pi.set_servo_pulsewidth(self.steering_pin, center_pw)
                        time.sleep(self.steering_pid.sample_time)
                        continue

                    # Only compute correction if marker is visible
                    if self.marker_visible and self.x_position is not None:
                        pid_output = self.steering_pid.compute(self.x_position)
                        if pid_output is not None:
                            correction = pid_output
                            scaled_correction = (correction / max_pixel_error) * max_correction_us
                            pulsewidth = center_pw + scaled_correction

                            # Clamp target pulse within min/max bounds
                            target_pw = max(min_pw, min(max_pw, pulsewidth))

                            # Initialize current_pw if not already
                            if not hasattr(self, 'current_pw'):
                                self.current_pw = center_pw

                            # Rate limiting
                            if target_pw > self.current_pw + max_servo_step:
                                self.current_pw += max_servo_step
                            elif target_pw < self.current_pw - max_servo_step:
                                self.current_pw -= max_servo_step
                            else:
                                self.current_pw = target_pw

                            self.last_steering_pw = self.current_pw
                            self.pi.set_servo_pulsewidth(self.steering_pin, self.current_pw)
                    else:
                        # Marker not visible — hold last known position
                        if hasattr(self, 'last_steering_pw'):
                            self.pi.set_servo_pulsewidth(self.steering_pin, self.last_steering_pw)

                    time.sleep(self.steering_pid.sample_time)
            except Exception as e:
                print(f"[Steering Control] Exception: {e}")
                time.sleep(0.1)

   

                                 
    def _measure_distance(self):
        #using sensor to measure distance
        # 10us pulse
        self.pi.write(self.trigger_pin, 1)
        time.sleep(0.00001)
        self.pi.write(self.trigger_pin, 0)

        timeout = 0.1
        start_time = time.time()

         # Wait for the echo to go high
        while self.pi.read(self.echo_pin) == 0:
            if time.time() - start_time > timeout or not self.running:
                print("Timeout waiting for echo start")
                return 0
            time.sleep(0.001)  # brief sleep to allow flag check

        echo_start = time.time()

        # Wait for the echo to go low
        while self.pi.read(self.echo_pin) == 1:
            if time.time() - echo_start > timeout or not self.running:
                print("Timeout waiting for echo end")
                return 0
            time.sleep(0.001)  # brief sleep to allow flag check

        echo_end = time.time()

        elapsed = echo_end - echo_start
        #34300 is speed of sound
        distance = (elapsed * 34300) / 2
        print("Measured distance: " + str(distance))
        return distance


    #speed control using ultrasonic sensor - ran on speed control thread
    def speed_control(self):
        #uses sensor to measure distance and adjusts speed accordingly
        while self.running:
            try:
                    if self.state == self.STATE_NOT_PLATOONING:
                        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, 0)
                        time.sleep(0.2)
                        continue
                    
                    distance_measured = self._measure_distance()
                    self.distance_measured = distance_measured
                           
                    if self.state == self.STATE_SEARCHING:
                        self.pi.write(self.motor_in1_pin, 1)
                        self.pi.write(self.motor_in2_pin, 1)
                        
                        current_time = time.time()
                        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, 30)
                        

                        if distance_measured < 30:
                            self.pi.write(self.motor_in1_pin, 1)
                            self.pi.write(self.motor_in2_pin, 1)
                            time.sleep(0.1)
                            self.pi.write(self.motor_in1_pin, 0)
                            self.pi.write(self.motor_in2_pin, 0)
                            
                            if self.searching_distance_start is None:
                                self.searching_distance_start = current_time
                            elif current_time - self.searching_distance_start >= 2:
                                self.handle_obstacle_during_searching()
                                self.searching_distance_start = None  # Reset after manoeuvre
                        elif distance_measured > 35:
                            self.pi.write(self.motor_in1_pin, 1)
                            self.pi.write(self.motor_in2_pin, 0)
                            self.searching_distance_start = None
                        
                        else: 
                            self.pi.write(self.motor_in1_pin, 0)
                            self.pi.write(self.motor_in2_pin, 0)    
                        

                        time.sleep(0.1)
                        continue
                           
                    elif self.state == self.STATE_LOCKED_ON:
                        distance_measured = self._measure_distance()
                       
                        #safely stop in case the distance is under 30
                        if distance_measured is not None and (distance_measured < 30):
                            self.pi.write(self.motor_in1_pin, 1)
                            self.pi.write(self.motor_in2_pin, 1)
                            time.sleep(0.1)
                            new_speed=0
                        elif distance_measured is not None and abs(self.x_position - 160) <= self.center_threshold:
                            #appending buffer if the marker isnt centered enough
                            self.median_buffer.append((distance_measured, time.time()))
                            
                            #computing pid output
                            raw_pid = self.speed_pid.compute(distance_measured)
                            if raw_pid is None:
                                new_speed = self.previous_speed
                            else:
                                pid_output = -raw_pid
                                base_speed = 20
                                new_speed = base_speed + pid_output
                                new_speed = max(0, min(new_speed, 60)) #can go back to 100
                                        
                        else:
                            fallback_distance = self.get_valid_median(max_age_sec=3.0)
                            if fallback_distance is not None:
                                print(f"[Fallback] Using arUco marker buffer median: {fallback_distance:.2f} cm")
                                
                                #computing the pid output
                                raw_pid = self.speed_pid.compute(fallback_distance)
                                if raw_pid is None:
                                    new_speed = self.previous_speed
                                else:
                                    pid_output = -raw_pid
                                    base_speed = 20
                                    new_speed = base_speed + pid_output
                                    new_speed = max(0, min(new_speed, 60)) #can go back to 100
                            else:
                                print("[Fallback] Buffer empty - holding previous speed")
                                new_speed = 30 #self.previous_speed
                                
                        #speed ramp limiting
                        speed_ramp_limit=7
                        if new_speed > self.previous_speed:
                            new_speed = min(new_speed, self.previous_speed + speed_ramp_limit)
                        
                     
                        motor_duty = int(new_speed * 255 / 100)
                        self.pi.write(self.motor_in1_pin, 1)
                        self.pi.write(self.motor_in2_pin, 0)
                        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, motor_duty)
                        
                        self.previous_speed = new_speed
                        time.sleep(0.2)
                        
            except Exception as e:
                print(f"[Speed Control] Exception: {e}")
                time.sleep(0.1)

    #runs in a seperate thread to manage states seperate of rest of program            
    def state_machine_loop(self):
        last_state = self.state

        while self.running:
            if self.user_wants_to_platoon():
                if self.state == self.STATE_NOT_PLATOONING:
                    self.state  = self.STATE_SEARCHING
                    print("[StateMachine] Transitioning to SEARCHING")
                else:
                    self.state = self.STATE_NOT_PLATOONING
                    print("[StateMachine] Transitioning to NOT_PLATOONING")
                           
            elif self.state == self.STATE_SEARCHING:
                    #marker stable detection test
                    if self.marker_is_detected_stably():
                        self.state = self.STATE_LOCKED_ON
                        self.locked_on_time = time.time() 
                        print("[StateMachine] Transitioning to LOCKED_ON")
            elif self.state == self.STATE_LOCKED_ON:
                    #transitions to searching if the marker is lost for too long
                    # Only consider marker as lost if we haven’t *seen* it recently
                if not self.marker_visible and (time.time() - self.last_marker_time) > 1.0: 
                    self.state = self.STATE_SEARCHING
                    print("[StateMachine] Marker lost, returning to SEARCHING")
            if self.state != last_state:
               print(f"[StateMachine] State changed: {last_state} -> {self.state}")
               self.update_leds(self.state)
               last_state = self.state
            time.sleep(0.1)
       
    def user_wants_to_platoon(self):
        current_time = time.time()
        button_state = self.pi.read(self.toggle_button_pin)

        if button_state == 0 and self.last_button_state == 1:
            # Button was just pressed
            if current_time - self.last_toggle_time > self.toggle_debounce:
                self.last_toggle_time = current_time
                self.last_button_state = button_state
            return True  #Toggle requested

        self.last_button_state = button_state
        return False

    def marker_is_detected_stably(self):
        # If we've seen a marker in the last N frames (buffer), return True
        recent_frames = list(self.marker_seen_buffer)
        if len(recent_frames) < 5: 
            return False
        return sum(recent_frames) >= 7

             
    def servo_sweep(self):
        # Typical values: 1100us (left), 1500us (center), 1900us (right)
        #servo sweeping implementation
        min_pw = 1300
        max_pw = 1700

        while self.running and self.state == self.STATE_SEARCHING and not self.override_steering:
            self.pi.set_servo_pulsewidth(self.steering_pin, self.servo_pulse)
            time.sleep(0.01)

            # Update pulse
            self.servo_pulse += 3 * self.servo_dir

            # Change direction at limits
            if self.servo_pulse >= max_pw:
                self.servo_pulse = max_pw
                self.servo_dir = -1
            elif self.servo_pulse <= min_pw:
                self.servo_pulse = min_pw
                self.servo_dir = 1

                                         
    #main run
                                 
    def run(self):
           
        self.start_time =time.time() #used in user_wants_to_platoon
           
       
        self.uart_thread.start()
        self.steering_thread.start()
        self.speed_thread.start()
        self.state_thread.start()

        print("[PlatoonFollower] Threads started. System running...")
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[PlatoonFollower] Stopping...")
            self.stop()
        print("run has fully exited")

                                 
             
                                 
    def stop(self):
        print("stop() called")
                                                     
       
        self.running = False
        print("runnig set to false")
        time.sleep(0.5)  # Allow threads to exit

        # Stop motor and servo outputs via pigpio
        print("stopping motor and servo")
        self.pi.set_PWM_dutycycle(self.motor_pwm_pin, 0)
        self.pi.set_servo_pulsewidth(self.steering_pin, 0)
        self.pi.write(self.motor_in1_pin, 0)
        self.pi.write(self.motor_in2_pin, 0)
        
        self.pi.write(self.RED_LED, 0)
        self.pi.write(self.YELLOW_LED, 0)
        self.pi.write(self.GREEN_LED, 0)

        # Close the serial connection
        print("closing seriel connection")
        self.serial_conn.close()

        # Stop pigpio connection
        if self._own_pigpio:                    
            print("stopping pigpio")
            self.pi.stop()

        print("[PlatoonFollower] Stopped and cleaned up.")      
            
                             
if __name__ == "__main__":
    follower = PlatoonFollower()
    follower.run()