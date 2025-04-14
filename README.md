This repository contains the MATLAB implementation developed as part of my Master's thesis at the University of Michigan-Dearborn. The project focuses on evaluating cybersecurity threats in collaborative robots and proposing a real-time Intrusion Detection System (IDS) for ROS 1-based Niryo Ned2 robots.

## 🎯 Project Objective

To design and validate a lightweight IDS capable of detecting cyberattacks on ROS 1 middleware by monitoring key topics and nodes in a collaborative robot system.

---

## 📁 Repository Structure

```
├── Dataset/                     # Spectrogram or classification datasets
├── Final Codes/                # Attack scripts and IDS logic
├── Capture_Image_Environment.m # Vision environment capture
├── Gripper.m                   # Gripper control logic
├── PickandPlace.m              # Pick-and-place routine
├── Prediction.m                # Object classification using ML
├── Sorting_Task.m              # Full object sorting pipeline
├── Task_1.m / Task_2.m         # Manual task test routines
├── Training_Code.m             # ML model training for classification
├── captureImage.m              # Image capture from robot camera
├── controlGripper.m            # Gripper helper function
├── main.m                      # Master script to initiate control
├── moveRobot.m                 # IK-based robot motion control
├── runRobot1Task.m             # Robot 1 sorting pipeline
├── runRobot2Task.m             # Robot 2 classification pipeline
```

---

## 🚨 Features

- 14 attack implementations across 5 ROS topics:
  - Joint states, joint commands
  - Gripper status, gripper goal
  - Camera stream
- Real-time IDS for:
  - Node authentication
  - Topic anomaly detection
  - Attack-triggered pause and node termination
- Object classification from camera-captured images
- Collaborative task execution between Robot 1 and Robot 2
- Data logging for offline analysis

---

## 🧪 Experiment Phases

1. **Sorting Task** – Baseline robot behavior  
2. **14 Cyberattacks** – Demonstrated by injecting messages, spoofing, and modifying ROS topics  
3. **Real-Time IDS** – Detection and automated mitigation  
4. **Extra Scenery Injection Attack** – Vision-based threat shown as future work (currently undetected)

---

## 🖥️ Requirements

- MATLAB R2021a+
- Robotics System Toolbox
- ROS 1 (tested with Noetic)
- Niryo Ned2 robots connected to the same ROS master

---

## 🎓 Thesis Reference

This repository supports the thesis:  
**"Cybersecurity Risks in Collaborative Robots: Real-Time Intrusion Detection for ROS-Based Systems"**  
University of Michigan-Dearborn, 2025

---

## 📜 License

This project is for academic and research use. Contact for reuse or collaboration.

---

The Rasnet50 model is uploaded in Google Drive - https://drive.google.com/file/d/1lOqGVEXqqmYuCSFG0mhBGuMnHEgNlZtd/view?usp=drive_link

## Youtube Video
https://youtu.be/JfxX2bU3MgQ
