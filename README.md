# Autonomous Drone Delivery

### Overview

This project enables autonomous flight of two drones carrying a rigid payload (simulating a pizza tray). The system integrates hardware and software to achieve coordinated, autonomous navigation, leveraging Python-based control, real-time sensor data, and a mobile interface to communicate with the drones.

### Prerequisites & Setup

Before using this project, you must follow the setup instructions detailed in the previous work that enables PC-to-drone communication: https://github.com/Penkov-D/DJI-MSDK-to-PC


#### Setup Instructions

These steps include:

*  Installing the DJI Mobile SDK on an Android device.
*  Setting up the PC-to-phone connection for drone control.
*  Running the necessary services to enable Python-based communication with the drone.


#### System Architecture
The system consists of the following components:

*  Drones (DJI Mini 3 Pro x2): Receives flight commands and provides sensor data.
*  Mobile Phone (Custom Android App): Acts as a bridge between the laptop and drones.
*  Laptop (Python Code): Executes autonomous flight control and processes sensor feedback.

#### Features
*  Autonomous Flight – Executes predefined flight plans.
*  Dual-Drone Coordination – Ensures stability while carrying a shared payload.
*  Real-time Sensor Data Processing – Utilizes GPS, IMU, and vision sensors for accurate navigation.
*  Python-Based Control – Uses a custom Python script to send commands and receive telemetry data.
*  PC-to-Drone Communication – Relies on an Android app to act as a communication relay.

#### Installation & Usage
##### 1. Setup Communication System
Follow the prerequisites setup guide linked above. Ensure that:
*  Your Android device has the necessary app installed.
*  The PC is connected to the phone and can communicate with the drones.
   
##### 2. Clone the Repository

```xml
git clone https://github.com/your-username/Autonomous_Drone_Delivery.git
cd Autonomous_Drone_Delivery
```

##### 3. Install Dependencies
Ensure you have Python installed, then install required dependencies:

```xml
pip install -r requirements.txt
```

##### 4. Run the Autonomous Flight Script
Execute the main Python script to start the autonomous flight:

```xml
python main.py
```


#### Technologies Used
*  Python – Main control logic and communication.
*  DJI Mobile SDK – Enables connection to the drones via Android.
*  OpenCV & NumPy – Used for sensor data processing (if applicable).


#### Contributing
If you'd like to contribute, feel free to fork the repository, submit pull requests, or report issues.

#### License
This project is licensed under the MIT License.

#### Acknowledgments
This project is inspired by previous work on PC-to-Drone communication: [DJI-MSDK-to-PC](https://github.com/Penkov-D/DJI-MSDK-to-PC). <br>






