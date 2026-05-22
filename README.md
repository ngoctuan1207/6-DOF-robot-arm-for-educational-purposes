# 6-DOF Educational Robotic Arm

## Overview
The goal of this project is to develop a cost-effective robotic arm system designed as an educational platform for student learning and hands-on training.
<img width="1920" height="1080" alt="RobotArm_Thumbnail" src="https://github.com/user-attachments/assets/ca39acf6-b188-479f-a3a9-6c0199982647" />

## Technical Scope
- Designed a 6-degree-of-freedom robotic arm structure using SolidWorks.
- Developed an STM32-based servo controller with I2C communication.
- Integrated six servo motors with ESP32 as the primary MCU.
- Implemented Forward/Inverse Kinematics with interpolation algorithms.
- Designed the electrical control panel in AutoCAD and developed the controller circuitry in Altium Designer.
- Developed an Android application with custom UI/UX for robotic arm control via TCP/IP communication.

## System Architecture
The robotic arm system architecture consists of:
- ESP32 as the primary communication and high-level control MCU
- STM32-based servo control modules for lower-load joints (J1, J5, and J6)
- Ezi-Plus R servo systems for joints J2, J3, and J4 due to higher torque and load requirements
- Android application for wireless robot operation via TCP/IP communication
- Custom-designed controller circuitry and electrical control panel for power distribution

## Software Features
- Forward/Inverse Kinematics calculation
- Motion interpolation for smooth trajectory control
- Real-time servo position control
- TCP/IP communication between Android application and robotic arm

## Design Iterations
### Version 1
- Initial prototype focused on basic robotic arm movement and servo integration
- Implemented basic angle control for all 6 robotic joints
- Tested STM32-based servo communication package
- Evaluated initial mechanical structure and motion performance on each joint
### Version 2
- Redesigned the linkage structure of joint 4, 5 and 6 to improve rigidity and stability
- Implemented Forward and Inverse Kinematics with interpolation algorithms
- Updated the PCB and electrical control panel design for improved system integration
- Implemented synchronized motion control for all 6 robotic joints
- Integrated a pneumatic gripper for handling objects weighing under 1 kg
- Integrated ESP32 wireless communication system
- Developed Android-based robotic arm control application
- 
<img width="6000" height="3375" alt="Robot arm comparision" src="https://github.com/user-attachments/assets/643d038c-a6c9-40a4-8858-c6df40547454" />



## Full Demonstration Video
[Watch on YouTube](https://youtu.be/WBqD1ZqwC8A?si=raEVwz-e9uBCVVol)

## Future Improvements
- Closed-loop feedback control
- Vision-based object detection
- Trajectory planning optimization
- ROS integration
- Industrial-grade actuator upgrade
- End-effector expansion support

