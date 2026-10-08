# Hydrogen IC Engine Retrofit for Heavy Commercial Vehicles

![Project Overview](images/screenshot.jpg)

## Overview

This project focuses on the research and development of a **hydrogen-assisted retrofit system for existing compression-ignition (CI) engines used in heavy commercial vehicles (HCVs)**.

The objective is to investigate the integration of hydrogen fuel delivery, electronic engine control, combustion optimization, and safety systems while retaining the fundamental architecture of an existing CI engine platform.

The project is being developed as part of **HERRSCHER Mobility's research into sustainable powertrain technologies**.

> **Note:** This repository presents a sanitized overview of the engineering approach. Proprietary fuel formulations, detailed hardware designs, confidential calibration data, and intellectual property are intentionally excluded.

---

## Project Objectives

- Investigate hydrogen integration into existing CI engine platforms
- Develop a hydrogen fuel-delivery architecture
- Develop an electronic engine-control strategy
- Study hydrogen–air combustion behavior
- Investigate injection and combustion timing
- Evaluate engine performance and efficiency
- Study potential emissions-reduction pathways
- Develop a scalable retrofit concept for heavy commercial vehicles
- Establish a foundation for future experimental validation

---

## System Architecture

The high-level retrofit architecture is:

```text
Hydrogen Storage
       │
       ▼
Pressure Regulation
       │
       ▼
Fuel Delivery System
       │
       ▼
Hydrogen Injection
       │
       ▼
Existing CI Engine
       │
       ├──────────────► Crank Position Sensor
       ├──────────────► MAP / Pressure Sensor
       ├──────────────► Throttle / Load Input
       ├──────────────► Knock / Combustion Feedback
       │
       ▼
      ECU
       │
       ├──────────────► Injection Control
       └──────────────► Engine Control Outputs
```

The retrofit approach aims to preserve the existing engine architecture while introducing the required hydrogen fuel-system and electronic-control modifications.

---

## Engine Control Concept

The electronic control system is intended to acquire engine operating parameters and generate appropriate fuel-control outputs.

### Key Inputs

| Parameter | Purpose |
|---|---|
| Crankshaft Position | Engine position and RPM calculation |
| Engine Speed | Operating-point determination |
| MAP / Intake Pressure | Load estimation |
| Throttle / Load | Driver or generator demand |
| Knock / Combustion Feedback | Combustion monitoring |
| Temperature | Operating-condition monitoring |

### Control Outputs

| Output | Function |
|---|---|
| Hydrogen Injector | Hydrogen fuel delivery |
| Engine Actuator | Engine operating control |
| Warning / Safety Output | Fault indication and shutdown logic |

The control strategy can be implemented using an embedded microcontroller platform such as **STM32**, with future development focused on real-time sensing, timing calculation, mapping, diagnostics, and actuator control.

---

# Embedded Control Development

A key development direction of this project is the implementation of an engine-control prototype using STM32.

### Proposed Demonstrator

The embedded demonstrator will investigate:

```text
60-2 Trigger Wheel
        │
        ▼
Crank Position Detection
        │
        ▼
Tooth Timing Measurement
        │
        ▼
Missing-Tooth Detection
        │
        ▼
RPM Calculation
        │
        ▼
Engine Position Estimation
        │
        ▼
Timing Map
        │
        ▼
Injection / Control Output
```

### Development Goals

- Decode a 60-2 crank trigger signal
- Detect the missing-tooth reference
- Calculate engine RPM
- Determine crank position
- Generate timing events
- Implement a basic calibration map
- Generate an injection or control output
- Evaluate timing accuracy using simulation or hardware testing

This demonstrator is intended as a **research and learning platform** for understanding real-time engine-control systems.

---

# Simulation & Analysis

Simulation is used to investigate engine operating behavior before hardware implementation.

Potential analysis areas include:

- Hydrogen substitution ratio
- Air–fuel ratio
- Equivalence ratio
- Combustion behavior
- Engine efficiency
- Power and torque
- Injection timing
- Combustion timing
- Emissions trends
- Thermal behavior

Example workflow:

```text
Operating Conditions
        │
        ▼
Fuel & Air Parameters
        │
        ▼
Engine / Combustion Model
        │
        ▼
Simulation
        │
        ▼
Performance & Emissions Results
        │
        ▼
Control Strategy Optimization
```

Simulation tools may include:

- MATLAB
- Simulink
- ANSYS
- Engineering calculation tools

---

# Safety Considerations

Hydrogen introduces specific engineering and safety considerations that must be addressed during system development.

This section intentionally remains at a high level and does not disclose proprietary safety-system implementation.

| Hazard | High-Level Mitigation |
|---|---|
| Hydrogen leakage | Leak detection, ventilation and system isolation |
| Over-pressure | Pressure monitoring and suitable pressure-relief protection |
| Flashback | Appropriate combustion and fuel-system design |
| Uncontrolled combustion | Engine monitoring and controlled fuel delivery |
| Fuel-system fault | ECU diagnostics and controlled shutdown |
| High-pressure storage | Certified components and appropriate safety procedures |

Any physical prototype must be developed and tested using appropriate engineering standards, certified components, laboratory procedures, and qualified supervision.

---

# Development Methodology

The project follows a staged engineering-development approach:

### Phase 1 — System Definition
- Engine-platform assessment
- Retrofit requirements
- System architecture
- Operating conditions

### Phase 2 — Control Development
- Sensor selection
- Signal acquisition
- ECU architecture
- Control logic
- Timing strategy

### Phase 3 — Simulation
- Engine modeling
- Combustion analysis
- Parameter studies
- Performance evaluation

### Phase 4 — Hardware Development
- ECU prototype
- Sensor interfaces
- Actuator interfaces
- Fuel-system integration

### Phase 5 — Validation
- Bench testing
- Engine testing
- Performance measurement
- Combustion analysis
- Emissions evaluation

### Phase 6 — Optimization
- Calibration
- Efficiency improvement
- Stability improvement
- Safety validation

---

# Repository Structure

```text
Hydrogen-IC-Engine-Retrofit-for-Heavy-Commercial-Vehicles/
│
├── README.md
│
├── images/
│   ├── screenshot.jpg
│   ├── system-architecture.png
│   └── simulation-results.png
│
├── docs/
│   ├── system-architecture.md
│   ├── sensor-actuator-table.md
│   └── control-strategy.md
│
├── simulation/
│   ├── matlab/
│   └── simulink/
│
├── firmware/
│   ├── stm32/
│   └── documentation/
│
└── results/
    ├── plots/
    └── analysis/
```

---

# Current Development Status

**Status:** Research & Development

Current work focuses on:

- Retrofit-system architecture
- Hydrogen fuel integration concepts
- Engine-control architecture
- Embedded ECU development
- Simulation and engineering analysis
- Safety considerations
- Future experimental validation

---

# Technologies & Tools

### Engineering

- MATLAB / Simulink
- ANSYS
- SolidWorks
- Fusion 360
- 3DEXPERIENCE

### Embedded Systems

- STM32
- Embedded C
- ESP32
- Sensor interfacing
- Real-time control concepts

### Powertrain

- Internal Combustion Engines
- Compression-Ignition Engines
- Hydrogen Fuel Systems
- Combustion Analysis
- Engine Control Systems
- Powertrain Engineering

---

# Research Direction

The long-term objective is to investigate practical pathways for **decarbonizing existing heavy-duty powertrain platforms** through hydrogen-based fuel integration and intelligent electronic engine control.

The research direction includes:

- Hydrogen-assisted CI engine operation
- Advanced ECU control
- Combustion optimization
- Retrofit engineering
- Engine efficiency improvement
- Emissions reduction
- Scalable heavy-duty applications

---

# Intellectual Property & Disclosure

This repository is intended for **technical portfolio and research documentation purposes**.

The following information is intentionally excluded:

- Proprietary fuel formulations
- Confidential fuel compositions
- Detailed injector designs
- Proprietary ECU calibration maps
- Confidential CAD drawings
- Detailed pressure-system designs
- Unpublished experimental data
- Patent-sensitive information

Only non-confidential engineering concepts and publicly shareable development material are presented.

---

# Future Work

- Complete STM32 crank-position decoder
- Develop RPM measurement and timing logic
- Implement basic injection/control mapping
- Develop MATLAB/Simulink control models
- Perform engine-system simulations
- Develop ECU prototype
- Conduct controlled laboratory testing
- Evaluate engine performance
- Investigate emissions characteristics
- Improve control and safety strategies
- Progress toward experimental validation

---

# Author

**Arun Roshan A S**

Automotive R&D | Powertrain & Combustion | Hydrogen IC Engines | ECU & Embedded Systems

**HERRSCHER Mobility**


## Disclaimer

This repository documents research and development concepts and is not a construction or operating manual for hydrogen fuel systems or modified engines.

Any physical implementation involving hydrogen, high-pressure fuel systems, or engine modification should be undertaken using appropriate engineering standards, certified components, laboratory safety procedures, and qualified professional supervision.
