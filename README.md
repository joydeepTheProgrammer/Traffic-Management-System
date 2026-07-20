# 🚦 Intelligent Traffic Management System v2.0

<p align="center">
  <img src="https://img.shields.io/badge/Target-ARM%20Cortex--M4-blue?logo=arm" />
  <img src="https://img.shields.io/badge/OS-FreeRTOS%20%2F%20POSIX-brightgreen" />
  <img src="https://img.shields.io/badge/Language-C99-A8B9CC?logo=c&logoColor=black" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

An advanced, real-time adaptive traffic control system designed for modern smart cities. The system utilizes multi-sensor fusion, dynamic adaptive green-light timing based on intersection density, and emergency vehicle preemption.

---

## 📖 System Architecture

The project contains two deployment targets:
1. **POSIX Simulator (`traffic_management_system.c`)**: A monolithic, multi-threaded POSIX C implementation for Linux/PC simulation.
2. **Embedded Firmware (Modular)**: A modular firmware designed for an ARM Cortex-M4 MCU running a cooperative RTOS scheduler, featuring a Hardware Abstraction Layer (`hal.h`), watchdog, and binary communication protocol.

### Core Features
- **Adaptive Timing Algorithm**: Adjusts green light duration (10s – 120s) based on real-time vehicle density and moving averages.
- **Emergency Preemption**: Instantly switches targeted lanes to GREEN and others to RED for emergency vehicles.
- **Multi-Sensor Fusion**: Combines IR, Ultrasonic, Inductive loops, Camera, and V2X sensor data.
- **Congestion Management**: Automatically triggers alternate routing modes when system load exceeds 85%.
- **Night Mode**: Automatically enters blinking-yellow power-saving mode between 22:00 and 05:00.

---

## 📐 Circuit & Wiring Diagram

The system interfaces with 12 traffic light signals (PORT A) and 12 hardware sensors (PORT B). 

### Electrical Schematics (Driver Details)

![Traffic Light Driver Details](hardware/circuit-drivers.svg)

![Sensor Input Isolation](hardware/circuit-sensors.svg)

### System Block Architecture

```mermaid
graph LR
    subgraph ARM Cortex-M4 MCU
        direction TB
        PA[PORT A - Traffic Light Drivers]
        PB[PORT B - Sensor Inputs]
    end

    subgraph Traffic Lights
        direction TB
        N_L[North: PA0=Red, PA1=Yel, PA2=Grn]
        S_L[South: PA3=Red, PA4=Yel, PA5=Grn]
        E_L[East:  PA6=Red, PA7=Yel, PA8=Grn]
        W_L[West:  PA9=Red, PA10=Yel, PA11=Grn]
    end

    subgraph Intersection Sensors
        direction TB
        N_S[North: PB0=IR, PB4=US_Trig, PB5=US_Echo, PB8=Inductive]
        S_S[South: PB1=IR, PB6=US_Trig, PB7=US_Echo, PB9=Inductive]
        E_S[East:  PB2=IR, PB10=Inductive, V2X Data]
        W_S[West:  PB3=IR, PB11=Inductive, V2X Data]
    end

    PA ==>|Relay/MOSFET Driver| N_L
    PA ==>|Relay/MOSFET Driver| S_L
    PA ==>|Relay/MOSFET Driver| E_L
    PA ==>|Relay/MOSFET Driver| W_L

    PB <==|ADC / Digital IN| N_S
    PB <==|ADC / Digital IN| S_S
    PB <==|ADC / Digital IN| E_S
    PB <==|ADC / Digital IN| W_S
```

### 📌 GPIO Pin Map (`hal.h`)

| Direction | Light: RED | Light: YELLOW | Light: GREEN | IR Sensor | Ultrasonic | Inductive Loop |
|-----------|------------|---------------|--------------|-----------|------------|----------------|
| **North** | PA0 | PA1 | PA2 | PB0 | Trig: PB4, Echo: PB5 | PB8 |
| **South** | PA3 | PA4 | PA5 | PB1 | Trig: PB6, Echo: PB7 | PB9 |
| **East**  | PA6 | PA7 | PA8 | PB2 | - | PB10 |
| **West**  | PA9 | PA10| PA11| PB3 | - | PB11 |

> **Note on Outputs:** MCU GPIO pins cannot drive traffic lights directly. PORT A pins should connect to a driver stage (e.g., ULN2003 transistor array or Optoisolated Relay Module) which then switches the high-voltage traffic lamps.

---

## ⚙️ Traffic Logic Reference

### Phase Timing (Adaptive)
- **Minimum Green**: 10 seconds
- **Maximum Green**: 120 seconds
- **Yellow Phase**: 3 seconds
- **All-Red Clearance**: 2 seconds
- **Pedestrian Walk**: 15 seconds (Adds +5 to +10s to Green phase if active)

### Sensor Prioritization
When selecting the next phase, the system calculates priority using:
`Priority = (Vehicle_Count * 1) + (Pedestrians_Waiting * 2) + (Emergency * 1000)`

### Modes of Operation
1. `MODE_NORMAL`: Standard adaptive round-robin phase execution.
2. `MODE_EMERGENCY`: Preemption mode triggered by emergency vehicles (Overrides all normal timing).
3. `MODE_CONGESTION`: System density > 85%. Modifies routing and timing buffers.
4. `MODE_NIGHT`: Engaged from 22:00 to 05:00. All lights flash YELLOW.
5. `MODE_FAULT`: Engaged if a light is stuck GREEN for > 130s or sensor confidence < 30%. Lights flash RED.

---

## 📡 Binary Communication Protocol (`protocol.h`)

The system utilizes a lightweight binary protocol for V2X and central server communications.

**Frame Structure (9 + N bytes):**
`[SYNC: 0x55 0xAA] [LEN: 2 bytes] [CMD: 1 byte] [PAYLOAD: N bytes] [CRC32: 4 bytes]`

**Key Commands:**
- `0x10`: Set Light State
- `0x12`: Set Timing Configuration
- `0x30`: Trigger Emergency Preemption
- `0x60`: Start Over-The-Air (OTA) Firmware Update

---

## 🔨 Building & Running

### 1. POSIX Simulator (PC/Linux)
The monolithic simulator tests the adaptive algorithm, queue logic, and multi-threading capabilities on a standard OS.

```bash
# Compile
gcc -o traffic_system traffic_management_system.c -lpthread -lm -Wall -Wextra -Werror

# Run
./traffic_system
```

### 2. Embedded Modular Firmware
The split files (`protocol.c`, `scheduler.c`, `watchdog.c`, `hal_sim.c`) represent the embedded RTOS structure.

```bash
# Compile (Example using standard gcc for the simulator HAL)
gcc -o traffic_firmware hal_sim.c protocol.c scheduler.c watchdog.c utils.c -Wall -Wextra
```

---

## ⚠️ Known Design Constraints

| Component | Issue | Resolution |
|-----------|-------|------------|
| **File Structure** | Extraneous `Vehicle-Safety-System` files (`main.c`, `alerts.h`, `Makefile`) were present in the root directory. | These files belong to a separate ATmega328P project and are safely ignored by the POSIX build process. |
| **Traffic Light Drive** | Cortex-M4 GPIO outputs 3.3V at ~20mA max. | Must use external relay modules or solid-state relays (SSR) to switch actual traffic lamps. |
| **Thread Safety** | Simultaneous sensor updates and phase transitions. | Handled via a global `pthread_mutex_t g_system.lock` in the POSIX simulation. |

---

## 📄 License

MIT License — Copyright © 2026 Joydeep Majumdar.
