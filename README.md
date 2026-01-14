# Vehicle Controller FMUs

This project provides FMI-compliant Functional Mock-up Units (FMUs) for vehicle control in co-simulation environments, particularly designed for integration with BeamNG driving simulator.

## Overview

The project implements three FMU components for vehicle platooning and control simulations:

- **Controller** - Longitudinal and lateral vehicle controller with PID logic for throttle, brake, and steering
- **Driver** - Generates desired acceleration profiles (constant, sinusoidal, ramp patterns)
- **CACC** - Cooperative Adaptive Cruise Control for vehicle-to-vehicle platooning

These FMUs bridge MATLAB/Maestro co-simulation environments with BeamNG's physics engine, enabling multi-platform vehicle dynamics research.

## Building

This project uses [FMU4cpp](https://github.com/Viproma/FMU4cpp) as a framework for building cross-platform FMUs.

```bash
cmake -B build
cmake --build build
```

Generated FMUs are located in `build/models/fmi2/` and `build/models/fmi3/`.

## FMU Components

### Controller
Receives vehicle state (position, velocity, acceleration, orientation) from BeamNG and outputs control commands (throttle, brake, steering).

### Driver
Generates desired acceleration profiles for testing different driving scenarios:
- Mode 1: Constant acceleration
- Mode 2: Sinusoidal variation
- Mode 3: Sprint patterns (acceleration followed by deceleration)

### CACC
Implements cooperative adaptive cruise control using vehicle-to-vehicle communication. Maintains safe following distance and coordinates acceleration with neighboring vehicles in a platoon.

## Usage

The FMUs are designed to work with:
- BeamNG.tech simulator (for vehicle dynamics)
- Maestro or similar FMI co-simulation orchestration tools
- MATLAB/Simulink with FMI Toolbox

All FMUs operate with a fixed time step of 5 ms.

## Configuration

Controller and Driver parameters can be configured through FMI parameters, including:
- Control gains and time constants
- Attack simulation parameters
- Operational modes and driving patterns
- Target distances for CACC
