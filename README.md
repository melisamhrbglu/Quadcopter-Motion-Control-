# 🚁 Quadcopter Motion Control System

A real-time quadcopter motion control simulation implementing PID-based stabilization and nonlinear motion dynamics.

This project was developed as part of CMSE443 and focuses on feedback control systems, physics-based simulation, and modular software architecture.

---

## 📌 Project Overview

The objective of this project was to design and simulate a quadcopter stabilization system using feedback control principles.

The system computes motor thrust outputs based on positional and orientation errors using PID controllers. The simulation updates drone dynamics in real time and visualizes system behavior.

Core concepts applied:

- PID Control (Proportional–Integral–Derivative)
- Feedback control loops
- Nonlinear motion modeling
- Real-time simulation loop
- Physics-based state updates

---

## 🧠 System Architecture

The system consists of modular components responsible for control, simulation, and visualization.

Execution flow:

User Input → Controller → PID Computation → Physics Update → Visualization

---

## 🧩 Core Modules & Key Scripts

### DroneController.cs
Implements manual control and PID-based hover stabilization.

- Altitude controller computes thrust around hover baseline (m·g)
- Attitude PID controllers compute pitch, roll, and yaw moments
- Inspector panel exposes Kp/Ki/Kd gains and limit parameters for tuning

<img src="https://github.com/user-attachments/assets/39d8991e-dfc3-4402-80a3-f3287aee2926" width="400">

---

### SimulationManager.cs
Owns the real-time simulation loop:

- Reads user input
- Steps physics engine
- Applies updated state to drone transform
- Stores last N states for telemetry and visualization

<img src="https://github.com/user-attachments/assets/140c53d9-8e72-4327-a268-97a6d9b6ee02" width="700">

---

### Physics & Integration Module
Implements quadcopter nonlinear dynamics and numerical integration.

- State derivatives computed from current state + ControlInput
- Supports Euler and RK4 integration
- Pitch and roll clamped to maximum tilt angle to prevent unrealistic flips

---

### DroneCrashHandler.cs
Handles collision detection and recovery logic.

- Detects impact via collision / trigger logic
- Temporarily disables control
- Applies short “fall” effect
- Respawns drone at initial position
- Re-enables controls

---

### DroneCameraController.cs
Manages camera perspectives.

- Third-person view
- First-person (FPV) view
- Keyboard-triggered switching between modes

<img src="https://github.com/user-attachments/assets/2eca8c70-6c7f-4c9a-915e-8a787d30de2c" width="400">

---

## 🗂 Scene & UI Structure

Hierarchy view showing Canvas selection and UI structure used for simulation control and telemetry.

<img src="https://github.com/user-attachments/assets/7d1d2474-eef0-432a-9706-411b58f86384" width="300">

---

## 📷 Simulation Views

### 🛩 Third-Person View
<img src="https://github.com/user-attachments/assets/9954a910-4bd7-4f92-a2e0-3475435b008e" width="700">

Third-person camera showing spatial behavior and drone dynamics.

---

### 🎮 First-Person View (FPV)
<img src="https://github.com/user-attachments/assets/fc69e1da-115e-4a6f-ad2d-e054946c8305" width="700">

FPV mode allows observation of drone orientation and stabilization from the pilot perspective.

---

## ⚙️ Control Strategy

The controller minimizes position and orientation error by adjusting motor thrust values.

Each axis (roll, pitch, yaw, altitude) is stabilized using PID control.

The system operates under a fixed real-time update loop to simulate continuous drone dynamics.

---

## 👩‍💻 My Contribution

- Designed and implemented system testing strategy
- Verified PID stability and simulation behavior
- Conducted unit and integration validation
- Contributed to final technical documentation

---

## ⚠️ Note

This project was developed within a local simulation environment during the course period. Setup configuration files may not be fully included in this archive.
