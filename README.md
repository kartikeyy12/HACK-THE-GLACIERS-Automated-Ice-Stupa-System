# 🧊 HACK THE GLACIERS: Automated Ice Stupa System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32-blue.svg)](https://www.espressif.com/)
[![Language](https://img.shields.io/badge/Language-C++%20|%20Python-orange.svg)]()

> **An open-source, edge-computed hardware and firmware ecosystem designed to automate and monitor artificial glaciers in high-altitude cold deserts.**

---

## 📖 The Backstory & Motivation

This repository contains my complete engineering solution for the **"Hack the Glaciers"** hackathon, an initiative focused on climate adaptation and building artificial glaciers (Ice Stupas) in the Trans-Himalayan region of Ladakh. 

As a 2nd-year Electronics and Communication Engineering student, I was highly motivated to solve these hardware challenges. However, the travel and logistics expenses to attend the physical competition in Ladakh were simply out of reach for my student budget. 

Instead of walking away, I decided to build, bench-test, and document the entire architecture as a comprehensive open-source portfolio project. This repository proves that robust embedded systems, autonomous control loops, and wireless mesh networks can be engineered and validated anywhere. Whether this gets officially acknowledged by the organizers or not, the goal is to contribute highly practical, deployable solutions to the climate tech community.

---

## 🌍 The Problem

Ice Stupas are constructed by siphoning meltwater through pipes and spraying it into sub-zero winter air, where it freezes into a massive conical ice reservoir. This stores winter water for the dry spring farming season. However, this manual process faces critical challenges:
1. **Pipe Freezing:** Water freezing mid-pipe during peak winter destroys the infrastructure.
2. **Microclimate Shifts:** Spraying water during warm shoulder-season days melts the existing ice structure.
3. **No Connectivity:** The steep, remote valleys lack cellular networks, making remote monitoring impossible.
4. **Volume Measurement:** Manually tracking the structural growth of the ice cone is inaccurate and dangerous.

---

## 🛠️ System Architecture & Sub-Projects

This repository is structured into four independent but highly integrated subsystems. Each directory contains its own dedicated firmware, Python scripts, hardware wiring guides, and Bills of Materials (BOM).

### [📂 01-mesh-communication](./01-mesh-communication)
**Off-Grid LoRaWAN/ESP-NOW Simulation**
A low-power, 3-node localized wireless network that operates with zero dependency on pre-existing infrastructure. It utilizes the ESP32's native ESP-NOW connectionless protocol to simulate a rugged sub-GHz mesh topology, featuring application-layer ACKs, dynamic TX power adjustments, and multi-hop routing.

### [📂 02-antifreeze-controller](./02-antifreeze-controller)
**Active Anti-Freeze Water Flow Controller**
An autonomous flow control station that combines thermal data (DS18B20) and kinetic data (YF-S201 flow sensor) to predict and prevent mid-pipe freezing. It drives a 12V motorized ball valve through a strict hardware-in-the-loop state machine to flush the system when a freeze risk is detected.

### [📂 03-microclimate-predictor](./03-microclimate-predictor)
**Predictive Microclimate Flow System**
An edge-computed environmental station utilizing a BME280 sensor and a Real-Time Clock. It runs a mathematical model based on the Wet-Bulb temperature approximation to calculate an "Ice Potential Index" (IPI) entirely offline. It actuates water flow only when the microclimate guarantees droplet freezing, protecting the ice structure from melting.

### [📂 04-spatial-volume-scanner](./04-spatial-volume-scanner)
**Spatial Ice Growth Measurement System**
A mechanized pan-tilt scanning system utilizing an ESP32 to sweep a micro-LiDAR module across a spherical grid. The firmware streams angular distance coordinates over UART to a Python visualizer, which renders an interactive 3D point cloud and calculates the enclosed volume of the Ice Stupa using a convex hull algorithm.

---

## ⚙️ Core Technology Stack

* **Microcontrollers:** ESP32 DevKit V1 (Xtensa LX6)
* **Firmware:** C/C++ (Arduino Framework, FreeRTOS concepts)
* **Protocols:** I2C, 1-Wire, UART, ESP-NOW, PWM (LEDC)
* **Software Integration:** Python (PySerial, NumPy, SciPy, Matplotlib)
* **Core Hardware:** DS18B20, BME280, YF-S201, TF-Luna LiDAR, IRF520 MOSFETs, 12V Actuators.

---

## 🚀 How to Navigate This Repository

If you want to replicate or review the systems, start by navigating into any of the four directories above. Each subsystem is fully self-contained with:
1. `main.cpp` / firmware files.
2. Complete ESP32 Pin Mapping Tables.
3. Point-to-point electrical netlists.
4. Mermaid.js system flow diagrams.

---

## 🤝 Contact & Contributions

Built by **Kartikey Tiwari**. 
If you are working on climate tech, embedded systems, or rural automation, feel free to fork this repository or reach out to discuss hardware optimizations. 

**License:** This project is licensed under the MIT License.
