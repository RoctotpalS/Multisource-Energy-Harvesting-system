# Intelligent Selective Mechanism for Efficient Energy Harvesting from Hybrid Multimodal Energy Sources

> B.Tech Thesis Project — Advanced Technology Development Centre (ATDC), IIT Kharagpur
> **Author:** Roctotpal Sandilya (Roll No: 22IE10035)
> **Supervisors:** Dr. Banibrata Mukherjee & Dr. Somnath Sengupta

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software / Firmware](#software--firmware)
- [Simulation](#simulation)
- [Results](#results)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [How It Works](#how-it-works)
- [Performance](#performance)
- [Future Work](#future-work)
- [References](#references)
- [License](#license)

---

## Overview

This project presents an **intelligent multi-source energy harvesting system** capable of dynamically evaluating, selecting, and combining multiple ambient energy sources (solar, thermal, vibration, RF, etc.) for efficient power harvesting.

Traditional harvesters rely on a single source or use passive diode-OR combining, which suffers from phase mismatch and reverse current leakage. This system overcomes these limitations by:

- **Evaluating** each source's instantaneous energy potential using an RC-based capacitor charging technique
- **Selecting** the strongest source as the primary harvester
- **Dynamically integrating** secondary sources only when they are in-phase with the primary, preventing destructive interference

The system was designed around the **MSP430G2553 microcontroller** and validated through MATLAB/Simulink simulation and hardware prototyping, demonstrating a **~57.6% improvement** in harvested voltage over conventional passive harvesting methods.

---

## Key Features

- **Multi-source support** — scales to 8, 16, or 24 sources using shift registers and analog multiplexers
- **Real-time phase matching** — prevents destructive interference between combined sources
- **Minimal GPIO usage** — only 3 MCU pins control all switches via 74HC595 shift register chain
- **Low power design** — MSP430 consumes ~1.1 mW active, ~250 µW in sleep mode
- **Scalable architecture** — hardware and firmware designed to accommodate N sources with minimal changes
- **Validated results** — both simulation (MATLAB/Simulink) and hardware experiments confirm performance improvement

---

## System Architecture

```
  Source 1 ──┐
  Source 2 ──┤──> Signal Conditioning ──> Switching & Control ──> Rectifier ──> Storage Capacitor
  Source N ──┘         Circuit              (SWS / SWB)                        (C_storage)
                           │                     │
                           └──> Microcontroller <─┘
                               (MSP430G2553)
                                    │
                          ┌─────────┴──────────┐
                     Shift Register         Analog MUX
                      (74HC595)             (CD4051B)
                    [Switch Control]     [Voltage Sensing]
```

### Two Operational Phases

| Phase | Description |
|---|---|
| **Evaluation Phase** | Each source is sequentially connected to C_eval. The MCU measures ΔV = V_cap_high − V_cap_low to rank sources by energy potential. |
| **Harvesting Phase** | The strongest source (max ΔV) is connected to C_storage. Secondary sources are dynamically added/removed based on real-time phase alignment. |

---

## Hardware Components

| Component | Role | Part Number |
|---|---|---|
| Microcontroller | Central control, ADC, timing | MSP430G2553 |
| Shift Register | Scalable switch control (3 pins → N outputs) | 74HC595 |
| Analog Multiplexer | Sequential voltage measurement via single ADC pin | CD4051B (3×8) |
| Signal Switch | Connects source to harvesting path | AQV202 / TLP222A (PhotoMOS) |
| Block Switch | Isolates source to prevent back-feeding | AQV202 / TLP222A (PhotoMOS) |
| DC-DC Converter | Regulated output, ultra-low quiescent current | LTC3388-3 |
| Voltage Inverter | Negative rail generation for signal conditioning | TC7660 |
| Evaluation Capacitor | Temporary charge buffer for source evaluation | C_eval |
| Storage Capacitor | Primary energy buffer | C_storage |

### Pin Mapping (MSP430)

| MCU Pin | Function |
|---|---|
| P1.0 (A0) | MUX output — source voltage measurement |
| P1.2 (A2) | Evaluation capacitor HIGH node |
| P1.3 (A3) | Evaluation capacitor LOW node |
| P1.4 (A4) | Board reference / ground sample |
| P1.5 | Shift register SER (data) |
| P1.6 | Shift register SRCLK (shift clock) |
| P1.7 | Shift register RCLK (latch) |
| P2.0 | CSW — evaluation capacitor switch |
| P2.1 | CSWS — storage capacitor switch |
| P2.2–P2.4 | MUX address lines S0, S1, S2 |
| P2.5–P2.7 | MUX bank enables (EN0, EN1, EN2) |

---

## Software / Firmware

The firmware is written in **C for MSP430** and implements the full two-phase algorithm.

### Key Firmware Functions

| Function | Description |
|---|---|
| `setup()` | Initializes clock, GPIO, ADC10, Timer A, shift register |
| `blockAllSources()` | Sets SWS=0, SWB=1 for all sources — safe initial state |
| `setSourceSwitches(i, sws, swb)` | Toggles SWS/SWB for source `i` and updates shift register |
| `updateShiftRegister()` | Shifts 64-bit `shiftState` MSB-first into 74HC595 chain |
| `readMuxedSource(i)` | Routes source `i` through MUX to ADC, returns raw ADC value |
| `analogReadToIndex(pin, idx)` | Triggers ADC10 conversion, stores result via ISR |
| `getCurrentTimeSeconds()` | Returns elapsed time using Timer A overflow counter |

### Shift Register Bit Mapping

Each source occupies 2 bits in the 64-bit `shiftState`:

```
Bit 0  → Source 0 SWS (signal switch)
Bit 1  → Source 0 SWB (block switch)
Bit 2  → Source 1 SWS
Bit 3  → Source 1 SWB
...
Bit 2i → Source i SWS
Bit 2i+1 → Source i SWB
```

### Firmware Configuration

To configure for your number of sources, edit the top of `multisource_energy_harvester_n_sources.c`:

```c
#define MAX_SOURCES     16      // Set to your number of sources (max 24 for 3×8 MUX)
#define HARVEST_SECONDS 300UL   // Harvesting window duration in seconds
```

### Phase Matching Condition

The secondary source `s` is enabled only when:

```c
if ((vp_ref > 0 && vdiff > 0) || (vp_ref < 0 && vdiff < 0))
    // Enable source s — constructive addition
else
    // Block source s — destructive, isolate it
```

where `vp_ref = vp − vref` and `vdiff = vs − vp`.

---

## Simulation

The system was modeled and validated in **MATLAB/Simulink** before hardware implementation.

### Simulation Setup

- **Number of sources:** 4 independent AC sources
- **Amplitudes:** 3V – 9V (varied to test adaptive selection)
- **Frequencies:** 50Hz – 70Hz with random phase offsets (0° – 180°)
- **Evaluation window:** 0.5s per source
- **Total simulation time:** 10s – 30s

### Simulink Subsystems

1. **Energy Source Subsystem** — 4 sinusoidal AC sources with varied parameters
2. **Switching & Rectification Subsystem** — SWS/SWB controlled switches + full-bridge rectifiers
3. **Evaluation Capacitor Unit** — charges sequentially per source, voltage sampled for ranking
4. **Storage Capacitor + Power Conditioning** — LTC3388-modeled DC-DC stage for regulated output

---

## Results

### Simulation Results — Output Voltage vs Frequency

| Frequency (Hz) | Output Voltage (V) |
|---|---|
| 1 | 30.0 |
| 5 | 28.0 |
| 10 | 27.0 |
| 15 | 27.0 |
| 20 | 30.5 |
| 30 | 30.5 |
| 40 | 30.5 |
| 50 | 19.0 |
| 100 | 19.0 |

> Performance degrades above ~40Hz due to MCU sampling rate limitations and reduced phase-detection window.

### Hardware Experimental Results

| Metric | Value |
|---|---|
| Proposed system output voltage | ~1.48V |
| Conventional SEH output voltage | ~0.92V |
| **Voltage improvement** | **~57.6%** |
| Harvested power (proposed) | 21.9 µW |
| Harvested power (conventional) | 8.46 µW |
| Simulation voltage improvement | ~271% |
| Simulation energy improvement | ~639% |

### Power Dissipation of Key Components

| Component | Typical Dissipation |
|---|---|
| MSP430 (active) | 1.1 mW |
| MSP430 (sleep) | 250 µW |
| AQV202 Optocoupler (switching) | 6–10 mW per device |
| Signal conditioning network | ~75 µW |
| TC7660 Voltage Inverter | 0.3–0.4 mW |
| LTC3388-3 Buck Converter | 5–8% of output power |

---

## Repository Structure

```
├── firmware/
│   └── multisource_energy_harvester_n_sources.c   # Main MSP430 firmware (C)
├── simulation/
│   └── simulink_model/                            # MATLAB/Simulink model files
├── hardware/
│   ├── circuit_diagram/                           # Circuit schematics
│   └── block_diagram/                             # System block diagrams
├── docs/
│   └── BTP_final.pdf                              # Full thesis report
├── results/
│   └── oscilloscope_captures/                     # Experimental waveform images
└── README.md
```

---

## Getting Started

### Prerequisites

- **Hardware:** MSP430G2553 LaunchPad, 74HC595 shift registers, CD4051B MUX ICs, AQV202 optocouplers, LTC3388 DC-DC converter, TC7660
- **Software:** Code Composer Studio (CCS) or IAR Embedded Workbench for MSP430
- **Simulation:** MATLAB R2020a or later with Simulink

### Flashing the Firmware

1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/<repo-name>.git
   ```

2. Open `firmware/multisource_energy_harvester_n_sources.c` in Code Composer Studio.

3. Configure the number of sources:
   ```c
   #define MAX_SOURCES 3   // Change to match your hardware setup
   ```

4. Build and flash to MSP430G2553 via the LaunchPad USB programmer.

### Running the Simulation

1. Open MATLAB and navigate to `simulation/simulink_model/`
2. Open the `.slx` Simulink model file
3. Set simulation parameters (number of sources, frequencies, amplitudes) in the model
4. Run the simulation and observe the storage capacitor voltage and switching behavior

---

## How It Works

### Step-by-Step Algorithm

```
1. INITIALIZATION
   └── Block all sources (SWS=0, SWB=1 for all)

2. EVALUATION PHASE  (repeated for each source i = 0 to N-1)
   ├── Block all sources
   ├── Enable source i (SWS=1, SWB=0)
   ├── Connect evaluation capacitor (CSW = HIGH)
   ├── Wait 0.5s for charging
   ├── Sample V_cap_high (A2) and V_cap_low (A3)
   ├── Compute ΔV[i] = V_cap_high − V_cap_low
   ├── Disconnect capacitor (CSW = LOW)
   ├── Block source i
   └── Wait 1s for discharge/stabilization

3. SOURCE SELECTION
   └── primary = argmax(ΔV[i])

4. HARVESTING PHASE  (runs for HARVEST_SECONDS)
   ├── Connect storage capacitor (CSWS = HIGH)
   ├── Enable primary source continuously
   └── For each secondary source s:
       ├── Read instantaneous voltage via MUX
       ├── Compare polarity with primary
       ├── If in-phase → Enable s (constructive addition)
       └── If out-of-phase → Block s (destructive, isolate)

5. RESET → Return to Evaluation Phase
```

---

## Performance

The system operates most reliably at **input frequencies below 20Hz**. At higher frequencies, the MSP430's ADC sampling rate and processing speed limit accurate phase detection.

| Frequency Range | System Behavior |
|---|---|
| 1–10 Hz | Excellent — clear phase detection, reliable switching |
| 10–20 Hz | Good — switching becomes faster, minor timing uncertainty |
| 20–40 Hz | Moderate — reduced phase detection window |
| >40 Hz | Degraded — MSP430 unable to reliably track polarity changes |

---

## Future Work

- **Scale to N > 10 sources** using integrated switching arrays
- **Replace optocouplers** with analog switch ICs (ADG series) for lower switching losses
- **Implement MPPT** (Maximum Power Point Tracking) for solar/thermoelectric sources
- **Machine learning adaptation** — lightweight models to predict optimal source combinations
- **PCB integration** — compact, optimized PCB layout for IoT/wearable deployment
- **Higher frequency support** — faster MCU or FPGA-based controller for >40Hz sources

---

## References

1. P. D. Mitcheson et al., "Energy harvesting from human and machine motion for wireless electronic devices," *Proceedings of the IEEE*, vol. 96, no. 9, pp. 1457–1486, 2008.
2. J. A. Paradiso and T. Starner, "Energy scavenging for mobile and wireless electronics," *IEEE Pervasive Computing*, vol. 4, no. 1, pp. 18–27, 2005.
3. Texas Instruments, "MSP430x2xx Family User's Guide," SLAU144, 2022.
4. Analog Devices, "LTC3388 – 20V Nanopower Synchronous Buck Converter Datasheet," 2021.
5. A. Raj, "Design and Implementation of Intelligent Dual-Source Energy Harvesting Architecture Using MSP430 Microcontroller," M.Tech Thesis, ATDC, IIT Kharagpur, 2024.

---

## License

This project is submitted as a B.Tech thesis at **IIT Kharagpur** under the **Advanced Technology Development Centre (ATDC)**.

© 2025 Roctotpal Sandilya. All rights reserved.

For academic use and reference, please cite this work appropriately.

---

*Made with ❤️ at IIT Kharagpur*
