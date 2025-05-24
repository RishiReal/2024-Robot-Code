# FRC 2024 Robot Code – Team 1099


This repository encapsulates our efforts in developing a robust, autonomous-capable robot, integrating advanced vision systems and precise motion control.

---

## Demo Video

2024 Recap of robot:
[Watch on YouTube](https://www.youtube.com/watch?v=0a_ImXGhRR4)

---

## Project Overview

Our 2024 robot was engineered to:

- **Navigate autonomously** using pre-defined trajectories.
- **Detect and track game elements** utilizing dual vision systems.
- **Execute complex maneuvers** with precision through advanced control algorithms.

Key technologies and tools employed:

- **Java** with **WPILib** for robot control.
- **PathPlanner** for trajectory planning.
- **Limelight** for AprilTag and reflective tape detection.
- **YOLOv5** for real-time object detection.
- **PID and feedforward control** for motion profiling.
- **Sensor fusion** integrating encoders, IMU, and vision data.

---

## Features

### Autonomous Navigation

- Utilizes PathPlanner for generating and following trajectories.
- Implements PID and feedforward control for smooth motion.

### Dual Vision Systems

- **Limelight**: Provides rapid target acquisition and AprilTag alignment.
- **YOLOv5**: Custom-trained models enable detection of specific game elements.

### Operator Interface

- Driver-friendly controls with intuitive mappings.
- Real-time telemetry displayed via SmartDashboard and Shuffleboard.

---
