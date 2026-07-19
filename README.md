# Traffic Management System

## POSIX simulation / firmware prototype

This repository is a C99 simulation of a four-way adaptive traffic controller. It is intended for algorithm development, protocol experiments, and host-side testing. It is **not certified traffic-control firmware** and must not be connected to real signals without a hardware safety design, independent fail-safe controller, and formal verification.

## What is implemented

- Four-way red/yellow/green phase controller with 3-second yellow and 2-second all-red clearance.
- Adaptive green duration based on queued vehicles and pedestrian requests.
- Emergency pre-emption: the requested direction receives green and the others receive red.
- Night-mode blinking-yellow simulation from 22:00 to 05:00.
- Congestion mode when queued traffic reaches 85% of total capacity.
- Simulated IR, inductive, camera/V2X-style sensor values and power readings.
- Watchdog task registration, task heartbeats, fault logging, and health checks.
- Binary UART framing with CRC-32.

## Circuit Diagram

<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/d35424a9-70be-4309-b65a-95a83630c5fa" />


## Current limits

- The HAL is a host-side simulator. GPIO, UART, ADC, flash, and watchdog calls do not control hardware.
- The UART simulator has no external input source, so protocol commands need a test harness or a real HAL implementation.
- Firmware updates, persistent logging, sensor calibration/configuration, route optimization, and network communications are not implemented.
- `scheduler.c` is a small cooperative scheduling utility. The supplied POSIX simulation uses `pthread` worker threads instead, because its tasks are long-running loops.

## Project files

| File | Purpose |
|---|---|
| `main.c` | Corrected POSIX simulation executable and protocol command processing. |
| `traffic_management_system.c` | Self-contained standalone simulation. |
| `traffic_system.h`, `utils.c` | Shared traffic model, globals, utilities, and logging. |
| `hal.h`, `hal_sim.c` | Hardware abstraction API and host-side simulator. |
| `protocol.h`, `protocol.c` | Frame format, CRC-32, encoding/decoding, and status responses. |
| `watchdog.h`, `watchdog.c` | Software watchdog and circular fault log. |
| `scheduler.h`, `scheduler.c` | Optional cooperative scheduling and IPC utility. |

## Build

Use GCC with pthread support from the repository root.

```bash
gcc -o traffic_firmware main.c utils.c hal_sim.c scheduler.c protocol.c watchdog.c \
  -lpthread -lm -Wall -Wextra -Werror -O2

./traffic_firmware
```

Standalone version:

```bash
gcc -o traffic_system traffic_management_system.c \
  -lpthread -lm -Wall -Wextra -Werror -O2

./traffic_system
```

Press `Ctrl+C` to stop either simulation. The modular executable puts all simulated lights red before it exits.

## GPIO identifier format

The HAL uses a portable encoded identifier:

```c
GPIO_MAKE_PIN(port, pin_number)
```

For example, `TRAFFIC_EAST_GREEN` represents Port A, pin 8. Pins are numeric positions, not bitmasks; code must not use STM32 bitmask-style arithmetic with this simulator.

## Protocol

Every frame is serialized as:

```text
[0x55][0xAA][LEN_HI][LEN_LO][CMD][PAYLOAD...][CRC32_BE]
```

CRC-32 covers `[LEN_HI][LEN_LO][CMD][PAYLOAD...]`. This removes host-endianness dependence.

Implemented commands:

| Command | Code | Payload | Result |
|---|---:|---|---|
| `HEARTBEAT` | `0x01` | none | Returns OK. |
| `STATUS_REQ` | `0x02` | none | Returns `STATUS_RESP`. |
| `SET_LIGHT` | `0x10` | `LightControlPayload` | Sets red, yellow, or green for one direction. |
| `EMERGENCY_TRIGGER` | `0x30` | `EmergencyPayload` | Starts emergency pre-emption. |
| `EMERGENCY_CLEAR` | `0x31` | `EmergencyPayload` | Clears pre-emption for one direction. |
| `PEDESTRIAN_REQUEST` | `0x40` | one direction byte | Adds a pedestrian request. |

All other declared protocol command IDs return `RESP_NOT_SUPPORTED`. Incoming payload lengths and direction/state ranges are validated before use.

## Porting to hardware

Replace `hal_sim.c` with a board-specific implementation that preserves the API in `hal.h`. In particular, implement safe GPIO startup states, non-blocking UART receive, monotonic timing, watchdog refresh, ADC calibration, and flash-write rules. A real deployment also needs independent conflict monitoring and a fail-safe all-red hardware path.

## Validation performed

The corrected copy was checked with:

```bash
gcc -fsyntax-only main.c utils.c hal_sim.c scheduler.c protocol.c watchdog.c \
  -Wall -Wextra -Werror -O2

gcc -fsyntax-only traffic_management_system.c \
  -Wall -Wextra -Werror -O2
```

Both targets pass strict syntax checks and build successfully. A protocol encode/decode CRC round-trip test also passes.
---

# License

Unless otherwise specified, all content in this repository—including, but not
limited to, software source code, firmware, hardware design files (schematics,
PCB layouts, Gerber files, BOMs, CAD files), documentation, configuration
files, examples, and supporting materials—is made available under the MIT
License.

---

## MIT License

Copyright (c) 2026 Joydeep Majumdar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---
