CompleteAutoNavi.py

What the code does?
A Python script designed to control a DJI Mini 3 Pro using the OpenDJI library. The script performs an autonomous task where the drone takes off, navigates to a predefined target location using GPS coordinates, stabilizes at a specific altitude, and then lands. Below is a detailed documentation of what the code does, broken down into sections:
________________________________________
Overview
•	Purpose: The script autonomously controls a drone to take off, stabilize at 2 meters altitude, rotate toward a target location, move to that location, descend to 1 meter, and then land.
•	Libraries Used: time, cv2 (though not used in this script), numpy, math, re, and OpenDJI (a custom library for DJI drone control).
•	Key Features: 
o	PID (Proportional-Integral-Derivative) control for altitude stabilization.
o	GPS-based navigation to a target latitude and longitude.
o	Yaw (compass heading) adjustment to face the target.
o	Error handling and timeouts for safety.
________________________________________
Constants and Global Variables
1.	DRONE_IP: The IP address of the drone (192.168.215.55).
2.	DEADZONE: A threshold (0.1) to prevent small oscillations in altitude control.
3.	TARGET_LAT and TARGET_LON: The target GPS coordinates (latitude: x, longitude: y).
4.	STOP_DISTANCE_THRESHOLD: A GPS distance threshold (~2 meters) to stop approaching the target (0.00002 in GPS units).
5.	LOCATION_PATTERN: A regular expression to parse location data (latitude, longitude, altitude) from the drone.
6.	current_data: A dictionary storing the drone’s current state (latitude, longitude, altitude, yaw).
7.	filtered_altitude: A global variable for a moving average of altitude to smooth readings.
________________________________________
Key Classes and Functions
1. PIDController Class
•	Purpose: Implements a PID controller to stabilize the drone’s altitude.
•	Parameters: 
o	Kp (Proportional gain): 0.2
o	Ki (Integral gain): 0.02
o	Kd (Derivative gain): 1.0
o	target: The desired altitude (e.g., 2 meters).
•	Functionality: 
o	Calculates the error between the target and current altitude.
o	Applies PID logic to compute a control output (du), limited to ±0.1 to avoid excessive vertical speed.
o	Uses a deadzone (DEADZONE) to ignore small errors and prevent jitter.
2. LocationListener Class
•	Purpose: Listens for and processes 3D location data (latitude, longitude, altitude) from the drone.
•	Behavior: 
o	Parses JSON-like strings using LOCATION_PATTERN.
o	Updates current_data with the latest values.
o	Applies a moving average filter to altitude (filtered_altitude) for smoother readings (98% old value + 2% new value).
3. YawListener Class
•	Purpose: Listens for and updates the drone’s yaw (compass heading) in current_data.
•	Behavior: Converts the received value to a float and stores it, with error handling for invalid data.
4. calculate_bearing Function
•	Purpose: Calculates the bearing (angle in degrees) from the drone’s current position to the target.
•	Input: Current latitude/longitude (lat1, lon1) and target latitude/longitude (lat2, lon2).
•	Output: A value between 0 and 360 degrees representing the direction to the target.
5. connect_and_takeoff Function
•	Purpose: Connects to the drone, enables control, sets up listeners, and initiates takeoff.
•	Steps: 
o	Establishes a connection to the drone at DRONE_IP.
o	Enables control and sets up listeners for location and yaw data.
o	Commands the drone to take off and waits 5 seconds.
•	Error Handling: Returns None if connection or takeoff fails.
6. altitude_control_with_pid Function
•	Purpose: Stabilizes the drone at a target altitude using PID control.
•	Input: The drone object and target altitude (e.g., 2 meters).
•	Behavior: 
o	Runs a PID loop until the altitude is within 0.05 meters of the target or a 30-second timeout occurs.
o	Adjusts vertical speed (du) every 0.03 seconds.
o	Adds extra stabilization iterations after reaching the target.
•	Output: Returns True if stabilized, False if failed.
7. rotate_to_target Function
•	Purpose: Rotates the drone to face the target bearing.
•	Input: The drone object and target yaw (from calculate_bearing).
•	Behavior: 
o	Calculates the yaw error and adjusts rotation speed (rcw ±0.1) until the error is less than 5 degrees.
o	Times out after 30 seconds if unsuccessful.
•	Output: Returns True if aligned, False if failed.
8. move_to_target Function
•	Purpose: Moves the drone to the target GPS coordinates while maintaining altitude.
•	Input: The drone object and target latitude/longitude.
•	Behavior: 
o	Uses a PID controller to maintain altitude at 2 meters.
o	Continuously calculates the distance to the target and stops when within STOP_DISTANCE_THRESHOLD (~2 meters).
o	Adjusts yaw dynamically and scales forward speed (0.02 to 0.1) based on distance.
o	Times out after 60 seconds.
•	Output: Returns True if the target is reached, False if failed.
9. main Function
•	Purpose: Orchestrates the entire mission.
•	Steps: 
1.	Takeoff: Connects and takes off using connect_and_takeoff.
2.	Stabilize at 2 meters: Uses altitude_control_with_pid to reach and hold 2 meters altitude.
3.	Rotate to Target: Calculates the bearing and aligns the drone using rotate_to_target.
4.	Move to Target: Navigates to TARGET_LAT and TARGET_LON using move_to_target.
5.	Descend to 1 meter: Lowers the drone to 1 meter using altitude_control_with_pid.
6.	Pause and Land: Waits 5 seconds at 1 meter, then lands and disconnects.
•	Error Handling: Lands and exits if any step fails.
________________________________________
Execution Flow
1.	Initialization: The script starts by connecting to the drone and taking off.
2.	Altitude Stabilization: The drone rises to 2 meters and stabilizes.
3.	Navigation: 
o	Calculates the direction to the target.
o	Rotates to face it.
o	Moves toward the target while maintaining altitude.
4.	Final Descent: Drops to 1 meter, pauses, and lands.
5.	Cleanup: Disconnects from the drone.
________________________________________
Key Notes
•	Timeouts: Each major operation (stabilization, rotation, movement) has a timeout to prevent infinite loops (30–60 seconds).
•	Dynamic Speed: Forward speed decreases as the drone approaches the target, ensuring a smooth stop.
•	Safety: The script includes error checks and lands the drone if anything fails.
•	Units: Distance is calculated in GPS coordinate units and roughly converted to meters for display (multiplied by 100,000).
________________________________________
Example Output
•	"🔗 Connected to drone successfully"
•	"🚀 Attempting takeoff... 🛫 Result: success"
•	"🛰 Current altitude: 1.95 meters"
•	"✅ Stabilized at 2 meters"
•	"📐 Bearing to target: 45.32 degrees"
•	"✅ Facing target"
•	"➡️ Moving toward target"
•	"Distance to target: 5.23 meters, Current altitude: 2.01 meters"
•	"✅ Stopped 1.98 meters from target"
•	"⬇️ Descending to 1 meter"
•	"🛬 Landing..."


This script provides a robust framework for autonomous drone navigation, with emphasis on stability, precision, and error handling.


How to use:
After installing the MSDK Remote application according to the guide https://github.com/Penkov-D/DJI-MSDK-to-PC
Download OpenDJI via this link and save it in the same folder with this code.We Connect the cell phone to the remote control, turn on HOTSPOT on a second cell phone and connect to it. An IP address will appear in the application, which we will enter in the global variable DRONE_IP in the code. Enter the GPS coordinates (you can get them using Google Maps, for example) of the location you want to fly the drone to. Copy the code to a Python compiler, turn on the drone and run the code(F5).
