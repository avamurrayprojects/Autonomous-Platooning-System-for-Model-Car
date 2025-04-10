# Model Vehicle Platooning System.  
This repository contains the code for a model car platoonig system developed as part of an academic project. The system enables a follower car to autonomously track and follow a lead car using computer vision and distance sensing. Key features include ArUco marker detection, PID-based speed and steering control, autonomous obstacle avoidance and searching behaviour when visual tracking is lost.

## System Overview  
The system is split across two Raspberry Pi units:  

**Pi 1: Image Processing**  
Responsible for capturing frames using a Raspberry Pi camera and detecting an ArUco marker placed on the rear of the lead car. It calculates the horizontal position of the marker in the frame and transmits this data via serial to the second Pi.  

**Pi 2: Control Pi**  
Handles all motion control functions including:  
- Reading distance measurements from the ultrasonic sensor.
- Running PID controllers for both steering and speed.
- Executing obstacle avoidance manoeuvres.
- Switching between oparational states.

This separating improves performance and allows the image processing to run at full frame rate without interfering with real-time control loops.

## Key Features  
**ArUco Marker Tracking** - Using OpenCV on the image Pi to track lead vehicle position.  
**PID Control** - Used for both smooth speed adjustments and accurate steering.  
**Obstacle Avoidance** - Detects and avoids any nearby obstacles with a defined manoeuvre.  
**State Machine Architecture** - Switches betweens *Not Platooning, Searching and Locked-On* modes.  
**Multi-Threaded Control** - INdependent threads for UART communication, steering, speed and state logic.  
**LED Indicators** - Show current state via LEDs.  

## Hardware Requirements  
- 2 x Raspberry Pi (testd with Raspberry Pi Model 4 b)
- Raspberry Pi Camera Module v2
- HC-SR04 ULtrasonic Sensor
- L298N Motor Driver
- DC Motor
- Standard Servo
- Battery Pack
- ArUco Marker printed and mounted to lead car
- *Optional: 3 x LED, 4 x push button, toggle switch*

## Licence
This project is intended for academic and educational use. Please cite appropriately if used in your own work.
