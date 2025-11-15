# 🚀 Projectile Reconnaissance Device (PRD)

### A Low-Cost, Disposable IoT Reconnaissance Capsule for Mapping and Sensing in Dangerous or Inaccessible Environments

![Status](https://img.shields.io/badge/status-Prototype%20Phase-blue)
![Hardware](https://img.shields.io/badge/hardware-IoT-green)
![Language](https://img.shields.io/badge/firmware-Embedded%20C-orange)
![Frontend](https://img.shields.io/badge/dashboard-Python%20-61DBFB)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

---

## 🧭 Overview

The **Projectile Reconnaissance Device (PRD)** is a **use-and-dump IoT sensor capsule** designed to be projected, dropped, or thrown into **hazardous, structurally unstable, or inaccessible locations**—such as collapsed buildings, mines, or deep caves—where human entry is impossible or dangerous.

The device autonomously collects environmental, spatial, and thermal data after deployment and transmits it wirelessly to a **remote Ground Control Station**.  
Its core innovation lies in **computational correction**, which digitally levels the captured LiDAR map even when the device lands at an unpredictable tilt.

---

## 🧩 Key Features

- **Disposable Design:** Low-cost build (~₹10,000) for one-time reconnaissance operations.
- **“Dumb Sensor / Smart Receiver” Architecture:** Lightweight device transmits raw data; the ground station performs heavy computation and visualization.
- **Multi-Sensor Data Fusion:**
  - LiDAR-based 2D/3D mapping
  - Orientation correction using IMU fusion
  - Environmental safety readings (temperature, humidity, pressure, air quality)
  - IR thermal imaging for search & rescue
- **Computational Leveling:** Corrects sensor data for tilt and pitch after landing.
- **Mission Dashboard:** Real-time visualization in a futuristic, dark-themed control interface (React-based).

---

## ⚙️ System Architecture

```
[ PRD Capsule ]
   ├── LIDAR-Lite v3  → Distance mapping (rotated 360° by servo)
   ├── BNO055 IMU     → Absolute orientation (pitch, roll, yaw)
   ├── BME680 Sensor  → Temp, humidity, pressure, gas resistance
   ├── AMG8833 IR Grid→ 8x8 thermal heatmap
   |—— Adafruit Ultimate GPS → GPS positioning (66 channels, 10 Hz updates)
   ├── nRF24L01       → Wireless transmission
   └── ATMEGA328P MCU → Core controller (low-power, Arduino-compatible)

          ↓ Wireless Link (2.4 GHz, nRF24L01)

[ Ground Control Station ]
   ├── Real-time data receiver
   ├── Computational correction for tilt (orientation-based)
   ├── 3D map generation from LiDAR data
   ├── Sensor data visualization dashboard (React)
   └── Mission logging and replay tools
```

---

## 🧠 Core Innovation: Computational Correction

When the capsule lands, it may rest at an **arbitrary tilt angle** on uneven terrain.  
Instead of using costly gyroscopic stabilizers or servo-leveling mechanisms, the PRD uses **orientation data from the BNO055 IMU** to digitally correct all incoming LiDAR data.

### Mathematical Concept:
Let:
- `θx`, `θy`, `θz` = Roll, Pitch, Yaw from IMU  
- `d` = LiDAR distance readings  

The **rotation matrix** derived from IMU orientation is applied to transform each LiDAR point `(x, y, z)` back to a global, level frame, thus generating a **flat, corrected 3D terrain map**.

---

## 🧰 Hardware Components

| Category | Component | Function |
|-----------|------------|-----------|
| Microcontroller | ATMEGA328P-PU | Central MCU for low-power control |
| Mapping | LiDAR-Lite v3 + FS90R Servo | Rotational 2D scanning |
| Orientation | BNO055 IMU | Provides absolute orientation with sensor fusion |
| Environmental | BME680 | Temp, humidity, pressure, and air quality |
| Thermal Vision | AMG8833 | 8×8 pixel IR thermal grid |
| Navigation | Adafruit Ultimate GPS Breakout v3 | GPS positioning with 66 channels, 10 Hz updates |
| Communication | nRF24L01 | Wireless transmission to base station |
| Power | Li-ion 3.7V Battery + Step-Up Regulator | Portable power supply |

---

## 🖥️ Ground Control Dashboard

The Ground Control Station (web app) is built using **React.js** and simulates a **dark, futuristic “Mission Control” interface**.

### Dashboard Features:
- Real-time IMU gauges (Pitch/Roll/Yaw)
- LiDAR sweep visualization with computational correction
- Thermal 8×8 grid heatmap
- Environmental charts for BME680 metrics
- Connection status indicators
- Mission log and replay support

---

## 🧪 Development Progress

| Milestone | Status | Description |
|------------|---------|-------------|
| Component Research & Selection | ✅ | Finalized cost-effective components |
| Circuit Simulation | ✅ | Verified using Circuit.io |
| 3D Modeling & Drop Simulation | ✅ | Designed and tested in Blender |
| Firmware Prototype | ⚙️ | Data collection and transmission routines under test |
| Dashboard UI | ✅ | Functional single-file React dashboard with mock data |
| Integration & Testing | 🔄 | In progress |

---

## 🧭 Use Case Scenarios

- **Search & Rescue Operations** – Deploy in collapsed buildings to locate heat signatures.
- **Cave & Mine Exploration** – Map unknown or unstable terrains.
- **Military Reconnaissance** – Gather environmental and spatial data without human risk.
- **Disaster Response** – Assess air quality, temperature, and structure safety post-event.

---

## 📡 Communication Protocol

- **Transmitter:** nRF24L01 module on PRD capsule
- **Receiver:** nRF24L01 module on Ground Station
- **Frequency:** 2.4 GHz ISM band
- **Data Rate:** 250 Kbps (optimized for reliability)
- **Encoding:** Binary packet with CRC checksum

---

## 📂 Repository Structure

```
PRD/
│
├── firmware/                # Arduino firmware for ATMEGA328P
│   ├── prd_main.ino
│   ├── sensors/
│   └── comm/
│
├── dashboard/               # React-based ground control UI
│   ├── src/
│   ├── public/
│   └── package.json
│
├── docs/                    # Technical and research documentation
│   ├── circuit_diagram.png
│   ├── 3d_model.blend
│   └── report.pdf
│
├── LICENSE
└── README.md
```

---

## 🧑‍🔧 Future Enhancements

- Add **LiDAR point-cloud stitching** for denser 3D mapping.
- Integrate **edge AI inference** for anomaly/heat detection.
- Implement **LoRa or Mesh Networking** for long-range communication.
- Enable **auto-triggered deployment** from UAVs or drones.
- Waterproof & ruggedized enclosure design.

---

## 📜 License

This project is licensed under the **MIT License** — you are free to use, modify, and distribute it with attribution.

---

## 🤝 Contributors

**Project Lead:** Shubham Singh  
**Guidance & Research Collaboration:** [Add mentors or institution names]  
**Visualization & Simulation:** Blender-based physics and dashboard mockups  

---

## 💬 Feedback & Collaboration

Interested in contributing or replicating the project?

- 🛠 Submit a PR with improvements or circuit refinements  
- 🧠 Share research ideas on sensor fusion or low-cost mapping  
- 📩 Contact: [Insert email or GitHub Discussions link]

---

> “Where humans can’t go, the PRD will go — see, sense, and send.”
