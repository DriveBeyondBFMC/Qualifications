# Information
Team: DriveBeyond  
Date: 17/03/2024  
Description: This is a report article on the status of project qualification.

# 📋 Table of Contents

* [Task 1 Path planning control](#task-1-path-planning-control)  
* [Task 2: Enhancing the VL53L1X sensor driver, IMU sensor driver, vehicle dynamics, UWB module, and encoder data processing.](#task-2-enhancing-the-vl53l1x-sensor-driver-imu-sensor-driver-vehicle-dynamics-uwb-module-and-encoder-data-processing)

* [Task 3: Implement Vehicle Speed Display using Qt](#task-3-implement-vehicle-speed-display-using-qt)

* [Task 4: Enhance UWB performance for better positioning accuracy and stability.](#task-4-enhance-uwb-performance-for-better-positioning-accuracy-and-stability)

* [Task 5: Improved Lane Detection and Lane-Keeping Algorithm](#task-5-improved-lane-detection-and-lane-keeping-algorithm)


# Task 1 Path planning control
## 📂 Project Directory Structure
```bash
~/Qualification/src/Task_1
├── autopilot.py
├── controller.py
└── Simulation/
   ├── AckermanControl.slx
   ├── drawPath.m
   ├── params.m
   ├── AckermanSimulation.py
   └──
```

## 📝 Description
   Focuses on the simulation and implementation of an Ackermann steering control algorithm with path planning integration. The system models vehicle behavior, including motor and steering delays, noise interference from UWB sensors, and dynamic mode switching. The implementation is divided into MATLAB-based simulations and a Python-based execution environment.

**Simulation of Control algorithm**  
   - File: `params.m`, `AckermanControl.slx`, `drawPath.m`, `AckermanSimulation.py`
   - Functionality: 
      - `params.m`, `AckermanControl.slx`, `drawPath.m`: Simulation in matlab with accurate simulation of delayed reponse time in the motor and steering, with added noise from UWB
      - `AckermanSimulation.py`: A simplified version of the matlab counterpart but with more detailed output for analysis and simulation of when to switch mode


**Intergration of path planning**
   - File: `autopilot.py` and `controller.py`
   - Functionality: 
      - `autopilot.py`: This file sends command to Nucleo and execute tasks
      - `controller.py`: Calculates steering angle based on the deviation and heading of the vehicle, derived from `AckermanSimulation.py`
   - Changes: 
      - `autopilot.py`: Added path planning for a defined path and execute command based on each situation

# Task 2 Enhancing the VL53L1X sensor driver, IMU sensor driver, vehicle dynamics, UWB module, and encoder data processing.
## 📂 Project Directory Structure
```bash
~/Qualifications/ 
└── Task_2/Embedded_Platform
    ├── include
    │   ├── drivers
    │   │   ├── vl53l1x.hpp
    │   │   ├── speedingmonitor.hpp
    │   ├── periodics
    │   │   ├── distance.hpp
    │   │   ├── uwb.hpp
    │   │   ├── vehicledynamic.hpp
    └── source
        ├── drivers
        │   ├── vl53l1x.cpp
        │   ├── speedingmonitor.hpp
        ├── periodics
        │   ├── distance.hpp
        │   ├── uwb.cpp
        │   ├── vehicledynamic.cpp
```
## 📝 Description
   - Improved the VL53L1X sensor driver to enhance distance measurement accuracy.
   - Refined the IMU driver to provide more precise sensor data.
   - Integrated UWB for real-time vehicle positioning.
   - Implemented encoder pulse reading for accurate motion tracking.
   - Combined all sensor data into vehicle dynamics calculations to determine the X, Y position.


# Task 3: Implement Vehicle Speed Display using Qt
## 📂 Project Directory Structure
```bash
~/Qualifications/ 
└── src/Task_3
    ├── QT
```
## 📝 Description
   - Transmit vehicle speed data to the Raspberry Pi 4.
   - Display real-time speed information using a Qt-based application.

# Task 4: Enhance UWB performance for better positioning accuracy and stability.
```bash
~/Qualifications/ 
└── src/Task_4
    ├── dwm1000
```

## 📝 Description
   - Migrated UWB processing from Arduino to Raspberry Pi Zero for more stable and accurate data reading.
   - Improved overall positioning reliability by leveraging Raspberry Pi Zero's processing capabilities.

   

# Task 5: Improved Lane Detection and Lane-Keeping Algorithm
## 📂 Project Directory Structure
```bash
~/Qualification/src/Task_5
├── LaneDetection.py
└── LaneKeeping.py
└── config.ini
```
## 📝 Description
Task 2 focuses on improving the existing lane detection and lane keeping algorithms to make the vehicle's navigation smoother on both straight and curved roads. The solution uses advanced image processing techniques and control algorithms to achieve this goal.

**Lane Detection**

- File: ` LaneDetection.py` 
- Functionality:
The lane detection module detects lane markings from input images, identifies lane points, and computes polynomial coefficients to represent lane shapes. It supports dashed lanes and adapts to varying road conditions, such as changing lighting and curved roads.
**Lane Keeping**

- File: ` steering_calculator.py ` 
- Functionality:
The lane keeping module uses the lane data from LaneDetection.py to calculate steering angles and speeds, ensuring that the vehicle stays centered in the lane. It dynamically adjusts based on road curvature and provides visualizations for steering and speed.

## ⚙️ Configuration and Initialization 

Both files rely on `config.ini` for parameter settings (e.g., detection thresholds, curvature threshold). 
