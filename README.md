# 🌞 Autonomous Sundial

**Author:** Jānis Birznieks  
**Year:** 2025

This project is a physical, electromechanical sundial system that autonomously tracks the sun’s position to display accurate local solar time. It combines mechanical actuation, environmental sensing, and real-time data integration across three cooperating devices.

---

## 🔧 System Overview

The system runs across three hardware platforms, each with its own software component:

### 1. **ESP32 (Firmware: `main.cpp`)**
- Controls two stepper motors (azimuth + tilt)
- Reads sensor data (LDRs, AS5600 analog angle sensors, limit switches)
- Implements the full state machine (OFF, ON, PAUSED, RESET)  
- Executes sun-seeking and initialization subroutines

### 2. **Raspberry Pi (Script: `ESPSerialSender.py`)**
- Sends current time and location to the ESP32
- Reads serial telemetry from the ESP32
- Stores all received data in a local MySQL database

### 3. **Database/Visualization Machine (Script: `Sundial_Plots.py`)**
- Connects to the MySQL database
- Displays:
  - Polar chart for real-time azimuth
  - Polar chart for gnomon tilt
  - Smoothed time-series plots of LDR data
  - Live system state indicator

---

## 🧩 Included Components

| Type        | File                 | Description                                                |
|-------------|----------------------|------------------------------------------------------------|
| ESP32 Code  | `main.cpp`           | Core firmware controlling motors, state logic, and sensors |
| Pi Script   | `ESPSerialSender.py` | Syncs time/location and logs telemetry                     |
| Visualizer  | `Sundial_Plots.py`   | Live data visualization from the MySQL database            |

---

## 📄 License

This repository is distributed under a **custom Academic Use License**.

You are permitted to use, study, and modify this work **for non-commercial academic or research purposes only**. Commercial use is **strictly prohibited** without written permission from the author.

See [`LICENSE.txt`](LICENSE.txt) for full terms.

---

## 🛠️ Status

This is an academic project submitted as part of a final coursework requirement. The repository is currently **private** and shared for review purposes only.

