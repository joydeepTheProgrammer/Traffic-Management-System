# 🚗 Vehicle Safety System

<p align="center">
  <img src="https://img.shields.io/badge/MCU-ATmega328P-blue?logo=arduino&logoColor=white" />
  <img src="https://img.shields.io/badge/Clock-16%20MHz-brightgreen" />
  <img src="https://img.shields.io/badge/Language-Embedded%20C99-A8B9CC?logo=c&logoColor=black" />
  <img src="https://img.shields.io/badge/Toolchain-AVR--GCC-orange" />
  <img src="https://img.shields.io/badge/Subsystems-8-blueviolet" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

<p align="center">
  <b>An 8-subsystem intelligent vehicle safety platform on a single ATmega328P.</b><br/>
  Real-time collision avoidance · Crash/rollover detection · Alcohol detection · Lane departure · GPS emergency beacon — all autonomous, no cloud.
</p>

---

## 📖 System Overview

An embedded firmware for the **ATmega328P @ 16 MHz** that continuously orchestrates 8 safety subsystems in a cooperative main loop, with a Timer0-driven 1 ms system tick. All actuator responses (brake relay, engine cut, buzzer, SMS) are triggered deterministically within the same loop iteration that detects the hazard.

```
Sensors ──────────────────────────────────────── ATmega328P ──────────── Actuators
  HC-SR04 ×4  (Collision F/R/L/R) ─ PORTB,C ──►                ──►  Brake Relay  K2
  MPU-6050    (Crash / Rollover)  ─ I2C/TWI ──► Main Loop        ──►  Engine Relay K1
  MQ-3        (Alcohol)           ─ ADC5    ──► 50ms cycle       ──►  Headlight K3
  DHT11       (Cabin Temp)        ─ PD7     ──►                  ──►  Buzzer BZ1
  IR ×2       (Lane Departure)    ─ PC4,PC5 ──►                  ──►  LCD 16×2
  LDR         (Auto Headlights)   ─ ADC4    ──►                  ──►  LEDs ×5
  NEO-6M GPS  (Location/Speed)    ─ PD5,PD6 ──►                  ──►  SIM800L SMS
  SIM800L GSM (Emergency SMS)     ─ PD3,PD4 ──►
```

---

## ✨ Safety Subsystems

| # | Subsystem | Sensors | Response |
|---|---|---|---|
| 1 | **Collision Avoidance** | 4× HC-SR04 (F/R/L/R) | Brake relay (< 30 cm) · Buzzer (< 80 cm) |
| 2 | **Drunk Driver Detection** | MQ-3 on ADC5 | Engine cut relay · Emergency SMS |
| 3 | **Crash / Rollover** | MPU-6050 IMU · I2C 0x68 | Emergency SMS · Brake relay |
| 4 | **Lane Departure Warning** | 2× IR (PC4/PC5) | Buzzer + LCD when speed > 10 km/h |
| 5 | **Driver Drowsiness** | GPS speed heuristic | Warning after 2× 30 s stable windows |
| 6 | **Auto Headlights** | LDR on ADC4 | Headlight relay when ADC > 700 |
| 7 | **Cabin Overheat / Fire** | DHT11 on PD7 | Warning ≥ 55°C · Evacuation ≥ 65°C |
| 8 | **GPS Emergency Beacon** | NEO-6M on PD5/PD6 | Lat/Lon embedded in all emergency SMS |

---

## 🏗️ Architecture

### Main Loop Schedule

```
while(1)  [~50 ms cycle due to _delay_ms(50)]
 ├─ Every iteration:
 │   ├─ collision_update()          HC-SR04 ×4 → COL_STATE_CLEAR/WARNING/CRITICAL
 │   ├─ mpu6050_read()              IMU → crash/rollover detection
 │   └─ ir_get_lane_status()        IR ×2 → LANE_OK/DEV_LEFT/DEV_RIGHT/DEV_BOTH
 │
 ├─ Every 5 loops (~250 ms):
 │   └─ mq3_get_status()            ADC5 → MQ3_CLEAN/WARN/DANGER
 │
 ├─ Every 10 loops (~500 ms):
 │   ├─ dht11_read()                PD7 → temperature + humidity
 │   └─ gps_read() + drowsiness     PD5/PD6 NMEA → lat/lon/speed
 │
 ├─ Every 20 loops (~1 s):
 │   ├─ ldr_update_headlights()     ADC4 → headlight relay
 │   └─ uart debug output
 │
 └─ alerts_update(flags, level)     Composite: buzzer + LEDs per alert bitmask
```

### Timer Architecture

| Timer | Mode | Period | Use |
|---|---|---|---|
| Timer0 | CTC (OCIE0A) | **1 ms** | `g_system_ms` system clock — `OCR0A=249`, prescaler=64 |
| Timer1 | Free-running | 0.5 µs/tick | HC-SR04 echo pulse measurement |

> **OCR0A formula:** `(F_CPU / (prescaler × 1000)) − 1 = (16000000 / 64000) − 1 = 249`

---

## ⚙️ Safety Logic Reference

### Collision Avoidance (HC-SR04 × 4)

| Distance | State | Response |
|---|---|---|
| > 80 cm | `COL_STATE_CLEAR` | No action |
| 30–80 cm | `COL_STATE_WARNING` | Buzzer warning · LCD shows direction |
| < 30 cm | `COL_STATE_CRITICAL` | Brake relay engaged · Continuous alarm |

### Alcohol Detection (MQ-3 · ADC5 · 10-bit 0–1023)

| ADC Reading | Status | Response |
|---|---|---|
| < 300 | `MQ3_CLEAN` | Engine relay restored |
| 300 – 499 | `MQ3_WARN` | Audible warning · LED_ALCOHOL · LCD |
| ≥ 500 | `MQ3_DANGER` | Engine cut relay · Emergency SMS |

> ⚠️ Requires 30-second warm-up (`MQ3_WARMUP_MS = 30000`). Readings before warm-up are discarded by `mq3_is_warmed_up()`.

### Crash / Rollover (MPU-6050 · I2C 0x68 · ±2g / ±250°/s)

| Condition | Threshold | Response |
|---|---|---|
| `accel_magnitude_g` > 3.5g | `CRASH_ACCEL_THRESHOLD_G` | Emergency SMS · Brake relay |
| `|gx_ds or gy_ds|` > 180°/s | `ROLLOVER_GYRO_THRESHOLD` | Emergency SMS · Brake relay |

> SMS is rate-limited: maximum 1 per 60 seconds (`last_sms_ms` check).

### Lane Departure (IR × 2 · PC4 / PC5)

| State | Meaning | Response |
|---|---|---|
| `LANE_OK` | Both sensors on lane | None |
| `LANE_DEV_LEFT` | Left IR lost lane | Warning if speed > 10 km/h |
| `LANE_DEV_RIGHT` | Right IR lost lane | Warning if speed > 10 km/h |
| `LANE_DEV_BOTH` | Both lost | Severe warning |

> IR detection level: `IR_LANE_DETECTED_LEVEL = 0` (LOW = lane marking detected)

### Drowsiness Detection (GPS speed heuristic)

- Only active when `gps_speed > 15.0 km/h` (`DROWSY_MIN_DRIVE_SPEED_KMH`)
- Checks every 30 seconds (`DROWSY_CHECK_INTERVAL_MS = 30000`)
- Triggers after **2 consecutive** windows where speed delta < 5 km/h (`DROWSY_SPEED_STABLE_KMH`)

### Auto Headlights (LDR · ADC4 · PC4)

| ADC4 Value | Condition | Response |
|---|---|---|
| > 700 | Dark | Headlight relay K3 ON |
| ≤ 700 | Bright | Headlight relay K3 OFF |

> `LDR_DARK_THRESHOLD = 700` (10-bit, VREF = 5V)

### Cabin Temperature (DHT11 · PD7)

| Temperature | Response |
|---|---|
| < 55°C | Normal |
| ≥ 55°C (`DHT11_TEMP_WARN_C`) | Warning: LCD "Cabin Temp High!" |
| ≥ 65°C (`DHT11_TEMP_DANGER_C`) | Critical: LCD "FIRE / OVERHEAT!" + evacuation alarm |

---

## 📌 GPIO Pin Map (ATmega328P)

### Port B

| Pin | Signal | Dir | Component | Notes |
|---|---|---|---|---|
| PB0 | US_FRONT_TRIG | OUT | HC-SR04 Front | 10µs pulse |
| PB1 | US_FRONT_ECHO | IN | HC-SR04 Front | Timer1 echo timing |
| PB2 | US_REAR_TRIG | OUT | HC-SR04 Rear | |
| PB3 | US_REAR_ECHO | IN | HC-SR04 Rear | |
| PB4 | HEADLIGHT_RELAY | OUT | Relay K3 (NPN Q3) | Active HIGH drives base via R8 |
| PB5 | BUZZER | OUT | BZ1 passive buzzer | Via R9 100Ω |
| PB6 | LED_COLLISION | OUT | Red LED1 | Via R10 330Ω |
| PB7 | LED_ALCOHOL | OUT | Yellow LED2 | Via R11 330Ω |

### Port C

| Pin | Signal | Dir | Component | Notes |
|---|---|---|---|---|
| PC0 | US_LEFT_TRIG | OUT | HC-SR04 Left | |
| PC1 | US_LEFT_ECHO | IN | HC-SR04 Left | |
| PC2 | US_RIGHT_TRIG | OUT | HC-SR04 Right | |
| PC3 | US_RIGHT_ECHO | IN | HC-SR04 Right | |
| PC4 | LDR ADC (ADC4) | IN | LDR + R5 divider | Analog: headlight sensing |
| PC5 | MQ-3 ADC (ADC5) | IN | MQ-3 gas sensor | Analog: alcohol sensing |
| PC4 | SDA (I2C TWI) | I/O | MPU-6050 + PCF8574 LCD | Shared with ADC4 (mutually exclusive) |
| PC5 | SCL (I2C TWI) | OUT | MPU-6050 + PCF8574 LCD | Shared with ADC5 |

> **⚠️ PC4/PC5 shared-use:** I2C transactions and ADC sampling are scheduled at different loop phases so they never overlap. For production, use a dedicated I/O expander for IR sensors and dedicate PC4/PC5 solely to I2C.

### Port D

| Pin | Signal | Dir | Component | Notes |
|---|---|---|---|---|
| PD0 | ENGINE_RELAY | OUT | Relay K1 (NPN Q1) | Active LOW — RELAY_ON = GPIO_LOW |
| PD1 | BRAKE_RELAY | OUT | Relay K2 (NPN Q2) | Active LOW — RELAY_ON = GPIO_LOW |
| PD2 | GSM_PWRKEY | OUT | SIM800L PWR | Toggle to power on/off |
| PD3 | GSM_TX (soft UART) | OUT | SIM800L RX | 9600 baud |
| PD4 | GSM_RX (soft UART) | IN | SIM800L TX | 9600 baud |
| PD5 | GPS_TX (soft UART) | OUT | NEO-6M RX | 9600 baud |
| PD6 | GPS_RX (soft UART) | IN | NEO-6M TX | 9600 baud |
| PD7 | DHT11_DATA | I/O | DHT11 1-wire | Pull-up R4 10kΩ |

> **⚠️ LED conflict resolved:** LED_CRASH, LED_LANE, LED_TEMP cannot be placed on PD3–PD5 (used by GSM/GPS soft UART). In this firmware these LEDs are driven via the **PCF8574 I2C expander** on the LCD backpack (GP4, GP5, GP6 outputs), not direct GPIO.

### UART0 (Hardware)

| Pin | Signal | Use |
|---|---|---|
| PD0 (RX) | DEBUG_RX | Serial debug (9600 baud 8N1) |
| PD1 (TX) | DEBUG_TX | Serial debug output |

> Hardware UART0 uses PD0/PD1 — these conflict with ENGINE_RELAY (PD0) and BRAKE_RELAY (PD1) in the pin map above. In this design, the **hardware UART0 is TX-only for debug** and PD0/PD1 double as relay control (relay signals dominate; UART used only at boot for debug strings before relays are active, or via a dedicated UART-to-relay mux in `uart.c`). For production, use a separate USB-UART debug header on PD0/PD1 and move relay control to Port B extended via PCF8574.

---

## 📡 Communication Protocols

| Protocol | Interface | Config | Module |
|---|---|---|---|
| HW UART0 | PD0/PD1 | 9600 baud 8N1 | Serial debug (PC) |
| Soft UART | PD3/PD4 | 9600 baud 8N1 | SIM800L GSM |
| Soft UART | PD5/PD6 | 9600 baud 8N1 | NEO-6M GPS |
| I2C / TWI | PC4(SDA)/PC5(SCL) | 400 kHz | MPU-6050 (0x68) + LCD PCF8574 (0x27) |
| ADC | ADC4 (PC4) | 10-bit, 125 kHz | LDR light sensor |
| ADC | ADC5 (PC5) | 10-bit, 125 kHz | MQ-3 alcohol sensor |
| 1-Wire | PD7 | Bit-banged | DHT11 temperature |

---

## ⚡ Power Supply

```
12V Vehicle Battery
 │
 ├─► L7805 (U3, TO-220) ──► +5V
 │     ├── C6 (470µF)  : input filter cap
 │     └── C7 (100µF)  : output filter cap
 │        │
 │        ├── MCU, sensors, relays, LCD, GPS (5V logic)
 │        └── ──► AMS1117-4.0 (U2) ──► +4V
 │                  └── C5 (100µF tantalum) : SIM800L supply
 │
 └─────────────────────────────────────── SIM800L 4V supply
```

> ⚠️ SIM800L draws up to **2A burst** during GSM transmission. Use a bulk capacitor ≥ 1000µF on the 4V rail and a dedicated regulator capable of 2A continuous.

---

## 🧩 Hardware Components

| Ref | Component | Qty | Interface | Notes |
|---|---|---|---|---|
| U1 | ATmega328P-PU | 1 | — | DIP-28, 16 MHz crystal |
| Y1 | 16 MHz Crystal | 1 | XTAL1/2 | With C1=C2=22pF load caps |
| J1–J4 | HC-SR04 Ultrasonic | 4 | PB0–PB3, PC0–PC3 | F / R / L / R |
| J5 | MPU-6050 (GY-521) | 1 | I2C 0x68 | ±2g accel, ±250°/s gyro |
| J6 | MQ-3 Gas Sensor | 1 | ADC5/PC5 | 30s warm-up |
| J7 | DHT11 | 1 | PD7 | ±2°C / ±5%RH |
| J8–J9 | TCRT5000 IR Module | 2 | PC4/PC5 (digital) | Lane detection |
| LDR1 | GL5528 Photoresistor | 1 | ADC4/PC4 | + R5 10kΩ divider |
| J10 | 16×2 LCD + PCF8574 | 1 | I2C 0x27 | 4-bit mode via I2C backpack |
| J11 | SIM800L GSM | 1 | Soft UART PD3/PD4 | 4V supply from AMS1117 |
| J12 | NEO-6M GPS | 1 | Soft UART PD5/PD6 | 9600 baud, NMEA $GPRMC |
| K1 | 5V SPDT Relay | 1 | PD0 via NPN Q1 | Engine cut-off |
| K2 | 5V SPDT Relay | 1 | PD1 via NPN Q2 | Brake assist |
| K3 | 5V SPDT Relay | 1 | PB4 via NPN Q3 | Auto headlights |
| Q1–Q3 | 2N2222 NPN | 3 | — | TO-92, relay drivers |
| D1–D3 | 1N4007 | 3 | — | Flyback protection across relays |
| BZ1 | Passive Buzzer 5V | 1 | PB5 via R9 | 3–5V, 85dB |
| LED1,3,5 | Red LED 3mm | 3 | PB6, PCF8574 GP4/GP6 | Collision + Crash + Overheat |
| LED2 | Yellow LED 3mm | 1 | PB7 | Alcohol warning |
| LED4 | Amber LED 3mm | 1 | PCF8574 GP5 | Lane departure |
| U2 | AMS1117-4.0 | 1 | — | SIM800L 4V regulator |
| U3 | L7805 | 1 | — | 5V system regulator |
| R1 | 10kΩ | 1 | RESET | Pull-up |
| R2, R3 | 4.7kΩ | 2 | PC4/PC5 | I2C bus pull-ups |
| R4 | 10kΩ | 1 | PD7 | DHT11 data pull-up |
| R5 | 10kΩ | 1 | ADC4 | LDR voltage divider |
| R6–R8 | 1kΩ | 3 | Q1–Q3 base | NPN relay base resistors |
| R9 | 100Ω | 1 | PB5 | Buzzer current limit |
| R10–R14 | 330Ω | 5 | LEDs | LED current limit (~10mA at 5V) |
| C1, C2 | 22pF | 2 | XTAL1/2 | Crystal load caps |
| C3 | 100nF | 1 | VCC | MCU decoupling |
| C4 | 10µF | 1 | VCC | MCU bulk decoupling |
| C5 | 100µF tantalum | 1 | AMS1117 out | GSM supply filter |
| C6 | 470µF | 1 | L7805 in | Input filter |
| C7 | 100µF | 1 | L7805 out | Output filter |

---

## 🗂️ Source File Layout

All files are in the **project root** (flat layout):

```
Vehicle-Safety-System/
├── main.c            Main loop · system_init() · timer0 · drowsiness FSM
│
├── alerts.c/.h       Buzzer patterns · relay macros · LED control · alerts_update()
├── collision.c/.h    Wraps ultrasonic ×4 → COL_STATE_CLEAR/WARNING/CRITICAL
│
├── ultrasonic.c/.h   HC-SR04 driver · Timer1 echo · 4 sensors
├── mpu6050.c/.h      MPU-6050 I2C · crash/rollover detect · sqrt via libm
├── mq3.c/.h          MQ-3 ADC5 · warm-up timer · MQ3_CLEAN/WARN/DANGER
├── dht11.c/.h        DHT11 1-wire PD7 · temp/humidity
├── ir_sensor.c/.h    IR ×2 PC4/PC5 → LANE_OK/DEV_LEFT/DEV_RIGHT/DEV_BOTH
├── ldr.c/.h          LDR ADC4 · ldr_update_headlights()
│
├── uart.c/.h         HW UART0 · 9600 baud · TX-only debug
├── lcd_i2c.c/.h      PCF8574 I2C LCD driver · lcd_print_str · lcd_set_cursor
├── gsm_sim800l.c/.h  SIM800L soft UART · AT commands · gsm_send_emergency_sms()
├── gps_neo6m.c/.h    NEO-6M soft UART · NMEA $GPRMC parser · lat/lon/speed
│
└── Makefile          avr-gcc · flat layout · make/flash/size/debug
```

---

## 🔨 Building & Flashing

### Prerequisites

```bash
avr-gcc --version          # 5.4+ recommended
avrdude --version          # For USBasp / Arduino bootloader
```

### Makefile Targets

```bash
make                  # Compile all → build/vehicle_safety_system.elf + .hex
make size             # Print Flash/SRAM usage (avr format for ATmega328P)
make flash            # Flash via USBasp  (avrdude -c usbasp)
make flash_uno PORT=COM3   # Flash via Arduino bootloader
make clean            # Remove build/ directory
```

### Fuse Bits (ATmega328P · 16 MHz external crystal)

```
Low  Fuse: 0xFF  — Full-swing crystal oscillator, no clock divide
High Fuse: 0xDE  — Serial programming enabled, brownout at 2.7V
Ext  Fuse: 0xFF  — Default

avrdude -c usbasp -p atmega328p -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m
```

---

## ⚠️ Known Design Constraints

| Issue | Detail | Resolution |
|---|---|---|
| **PD0/PD1 dual-use** | HW UART0 (debug) shares PD0/PD1 with Engine/Brake relay pins | UART used for boot debug only; relay signals dominate in runtime |
| **PC4/PC5 triple-use** | ADC4/ADC5 + I2C SDA/SCL + IR sensors all share PC4/PC5 | Firmware phases reads: I2C never active during ADC sampling or IR read |
| **LED_CRASH/LANE/TEMP** | PD3–PD5 needed by GSM/GPS soft UART, not available for LEDs | Routed via PCF8574 GP4/GP5/GP6 on LCD I2C backpack |
| **SIM800L 2A burst** | L7805 only rated 1.5A continuous | Dedicate AMS1117-4.0 + 1000µF bulk cap for GSM rail |
| **DHT11 blocking read** | DHT11 read takes ~22 ms; called every 10 loops (~500 ms) | Acceptable for safety latency; move to RTOS task for stricter timing |
| **ATmega328P has no PD8** | Original comment in main.c mentioned "PD8 if expanded" | ATmega328P is 28-pin, max PD7. Removed the misleading comment |

---

## 📄 License

MIT License — Copyright © 2026 Joydeep Majumdar.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
