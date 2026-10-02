# Autonomous Vehicle Simulation & Navigation System

[![Luau](https://img.shields.io/badge/Language-Luau%20--!strict-00A2FF?style=flat-square&logo=lua)](https://luau.org)
[![Roblox](https://img.shields.io/badge/Platform-Roblox%20Engine-black?style=flat-square&logo=roblox)](https://www.roblox.com)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square)](https://opensource.org/licenses/MIT)

An enterprise-grade autonomous vehicle simulation and navigation controller engineered for constraint-based vehicles in the Roblox engine.

---

## Direct Code Link
- **Main Script**: [`AutonomousVehicleSystem.luau`](https://github.com/rdj415/autonomous-vehicle-simulation/blob/main/AutonomousVehicleSystem.luau)

---

## Architectural Highlights

### 1. Object-Oriented State Encapsulation
- Constructed with Luau metatables (`__index`) and strict static typing (`--!strict`).
- Complete lifecycle management (`new`, `Start`, `Update`, `Destroy`).

### 2. Kinematics & Coordinate Space Transformations
- Uses `CFrame:VectorToObjectSpace` to convert world `AssemblyLinearVelocity` into chassis-local axes, isolating longitudinal speed (`-relativeVelocity.Z`) and lateral drift.
- Calculates target heading deviation in vehicle local space using `CFrame:PointToObjectSpace` and `math.atan2(localTarget.X, -localTarget.Z)`.

### 3. Multi-Ray Sensory Obstacle Avoidance
- Projects 3 predictive directional rays from the vehicle origin:
  - **Center Ray**: Direct forward trajectory.
  - **Left Ray**: +28° yaw offset via `CFrame.Angles(0, math.rad(28), 0)`.
  - **Right Ray**: -28° yaw offset via `CFrame.Angles(0, math.rad(-28), 0)`.
- Weighs repulsion bias inversely proportional to obstacle proximity ($1 - \frac{\text{dist}}{\text{range}}$) to guide the steering rack away from barriers smoothly.

### 4. Dynamic Wheel Slip & Friction Degradation
- Continuously calculates tire perimeter surface speed ($v_{\text{surface}} = \omega \cdot r$) against part `AssemblyLinearVelocity.Magnitude`.
- Dynamically assigns `PhysicalProperties.new(density, friction, elasticity)` on wheel parts when slip exceeds thresholds, simulating transition between static rolling traction and kinetic skidding.

### 5. Transmission Dynamics & Powertrain Curves
- Low-pass filters virtual engine RPM/speed to replicate drivetrain inertia.
- Torque interpolation curve delivering high low-end torque for static break-away force and tapered torque at maximum speed.
- Speed-sensitive steering reduction to prevent high-speed rollover instability.

### 6. Fail-Safe Recovery Routines
- **Anti-Flip Recovery**: Evaluates seat `CFrame.UpVector.Y`. If tilted beyond recoverable threshold ($< 0.25$) for over 2 seconds, zeroes angular momentum and uprights the chassis.
- **Unstuck Routine**: Monitors vehicle displacement under high commanded engine speed. Triggers a two-stage counter-steer and reverse throttle sequence when immobilized.

---

## Verification & Installation

1. Clone or download this repository.
2. In Roblox Studio, place `AutonomousVehicleSystem.luau` inside any constraint-based vehicle Model (requiring `Chassis`, `Engine` with 4 drive motors, `Steering` with `SteeringRack`, and `Wheels` folder).
3. Optional: Add a `Waypoints` folder in `Workspace` containing numbered parts to establish a cyclic patrol route.

---

## Author
- GitHub: [@rdj415](https://github.com/rdj415)
- Roblox: `DiscoUnicorn478`
- Discord: `rd_rb`
