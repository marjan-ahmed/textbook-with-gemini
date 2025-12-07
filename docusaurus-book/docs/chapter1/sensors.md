---
sidebar_position: 4
---

# Sensor Systems for Humanoid Robots

## Overview

Humanoid robots rely on a diverse array of sensors to perceive and interact with their environment. This section covers the key sensor modalities you'll work with throughout this course.

## Vision Sensors

### RGB Cameras

**Purpose**: Color image capture for object recognition, scene understanding

**Specifications**:
- Resolution: 1080p to 4K
- Frame Rate: 30-60 fps
- Field of View: 90-120 degrees

**Applications**:
- Object detection and classification
- Facial recognition
- Color-based tracking
- Visual odometry

**Common Hardware**:
- Intel RealSense D435i
- Logitech C920/C930e
- Raspberry Pi Camera Module v3

### Depth Cameras

**Purpose**: Capture 3D structure of the environment

**Technologies**:
- **Structured Light**: Project pattern, measure distortion
- **Time-of-Flight (ToF)**: Measure light travel time
- **Stereo Vision**: Calculate depth from two cameras

**Specifications**:
- Depth Range: 0.3m - 10m (typical)
- Depth Accuracy: ±1-5mm at 1m
- Frame Rate: 30-90 fps

**Applications**:
- Obstacle detection
- 3D mapping and SLAM
- Grasp planning
- Human pose estimation

**Common Hardware**:
- Intel RealSense D435i/D455
- Microsoft Azure Kinect
- Orbbec Astra
- ZED 2/2i (stereo)

## LiDAR (Light Detection and Ranging)

**Purpose**: Accurate 3D environment mapping

**How it Works**: Laser pulses measure distance by time-of-flight

**Types**:
- **2D LiDAR**: Single scanning plane (e.g., SICK, Hokuyo)
- **3D LiDAR**: Multi-plane or spinning (e.g., Velodyne, Ouster)

**Specifications**:
- Range: 10m - 100m+
- Angular Resolution: 0.1-1 degree
- Scan Rate: 5-20 Hz

**Advantages**:
- High accuracy (cm-level)
- Works in various lighting conditions
- Long range

**Disadvantages**:
- Expensive
- Power hungry
- Cannot detect transparent surfaces

**Applications**:
- SLAM (Simultaneous Localization and Mapping)
- Obstacle avoidance
- Navigation
- 3D environment reconstruction

## Inertial Measurement Units (IMUs)

**Purpose**: Measure acceleration and rotational velocity

**Components**:
- **Accelerometer**: Linear acceleration (3-axis)
- **Gyroscope**: Angular velocity (3-axis)
- **Magnetometer**: Magnetic field orientation (optional)

**Specifications**:
- Update Rate: 100-1000 Hz
- Accuracy: Varies widely by cost/quality

**Applications**:
- Balance control for bipedal walking
- Orientation tracking
- Fall detection
- Motion profiling

**Common Hardware**:
- BMI088 (Bosch)
- MPU-6050/9250 (InvenSense)
- ADIS16505 (Analog Devices)
- VectorNav VN-100

**Key Concept - Sensor Fusion**: IMUs are often combined with cameras/GPS for accurate state estimation.

## Force/Torque Sensors

**Purpose**: Measure forces and torques at robot joints or end-effectors

**Types**:
- **6-Axis F/T Sensors**: Measure 3 forces + 3 torques
- **Joint Torque Sensors**: Built into actuators

**Applications**:
- Compliant grasping (don't crush objects)
- Force control for assembly tasks
- Detecting collisions
- Human-robot interaction (safe contact)

**Common Hardware**:
- ATI Industrial Automation (now OnRobot)
- Robotiq FT 300
- Weiss Robotics

## Proprioceptive Sensors

**Purpose**: Sense the robot's own state

### Joint Encoders
- **Function**: Measure joint angles
- **Types**: Absolute, incremental
- **Resolution**: 0.01-0.001 degrees

### Current/Voltage Sensors
- **Function**: Monitor motor currents
- **Use**: Detect jams, estimate torque

### Temperature Sensors
- **Function**: Monitor component temperatures
- **Use**: Prevent overheating, thermal management

## Microphones and Audio

**Purpose**: Capture sound for voice interaction

**Specifications**:
- **Array**: 2-7 microphones for beamforming
- **Sample Rate**: 16-48 kHz
- **Signal Processing**: Noise cancellation, echo suppression

**Applications**:
- Voice command recognition
- Speaker localization (turn toward sound)
- Environmental audio analysis

**Common Hardware**:
- ReSpeaker Mic Array
- Matrix Voice/Creator
- USB conference microphones

## Tactile Sensors

**Purpose**: Detect contact and pressure

**Types**:
- **Resistive**: Pressure changes resistance
- **Capacitive**: Touch detection
- **Optical**: Light-based pressure sensing

**Applications**:
- Grasp detection ("Did I pick it up?")
- Slip detection (object sliding in gripper)
- Safe human contact

**Research Area**: Soft robotics and skin-like sensor arrays are active research topics

## Sensor Fusion and Processing

Real-world robots **combine multiple sensors** for robust perception:

### Example: Visual-Inertial Odometry (VIO)
- Camera: Visual features
- IMU: High-frequency motion updates
- **Result**: Accurate, drift-resistant localization

### Example: Multi-Modal Object Detection
- RGB Camera: Color, texture, labels
- Depth Camera: 3D position and size
- **Result**: Robust object recognition and pose estimation

## Data Volumes and Bandwidth

**Challenge**: Humanoids generate massive data streams

| Sensor | Data Rate |
|--------|-----------|
| RGB Camera (1080p, 30fps) | ~60 MB/s |
| Depth Camera | ~30 MB/s |
| 3D LiDAR | ~10-50 MB/s |
| IMU (1000Hz) | ~0.01 MB/s |

**Solution Strategies**:
- On-board processing (edge computing)
- Data compression
- Selective streaming (only send when needed)
- Distributed computing architectures

## Sensors You'll Use in This Course

Throughout the course, you'll work primarily with:

✅ **RGB-D Cameras** (e.g., Intel RealSense)  
✅ **Simulated LiDAR** (in Gazebo/Isaac Sim)  
✅ **Simulated IMU** (for balance control)  
✅ **Virtual Force Sensors** (in manipulation tasks)

Most initial work will be in simulation, where sensors are "perfect." Later, you'll learn about real-world sensor noise, calibration, and fusion.

---

👉 **[Next: Hardware Requirements →](./hardware-requirements)**
