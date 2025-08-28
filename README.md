
# vsa_supervisor

## Overview

The `vsa_supervisor` package is responsible for managing the overall system behavior of an autonomous vehicle by implementing a state machine. It acts as the central node, coordinating guidance, plan execution, teleoperation, and error handling.

## Main Features

- **State Machine Management:** Handles system states such as BOOT, SERVICE, CALIBRATION, MANEUVER, EXTERNAL (teleoperation), and ERROR.
- **Plan Execution:** Integrates with a plan database to execute maneuver plans, manage plan transitions, and handle plan control commands.
- **Guidance Control:** Interfaces with guidance controllers (PID for thruster, pitch, and yaw) to compute actuator signals for autonomous navigation.
- **Teleoperation Support:** Allows manual control via teleoperation messages, overriding autonomous behavior when required.
- **Health Monitoring:** Periodically checks system health and transitions to safe states if issues are detected.

## Node: `Supervisor`

### Main Loop

- Runs at a configurable rate.
- Checks system health.
- Sets outputs (publishes states, actuator signals).
- Gets inputs (updates odometry).
- Executes the state machine logic.

### State Machine

- **BOOT:** Initial state, transitions to SERVICE after a delay.
- **SERVICE:** Idle/standby state.
- **CALIBRATION:** Prepares the system for maneuver execution.
- **MANEUVER:** Executes the current plan using guidance controllers.
- **EXTERNAL:** Teleoperation mode, accepts manual actuator commands.
- **ERROR:** Stops actuators and transitions to SERVICE.

### Plan and Guidance

- Receives plan control and plan database messages to start, stop, or modify plans.
- Converts maneuver waypoints to trajectory setpoints for guidance.
- Publishes estimated state based on odometry and orientation.

### Actuator Control

- Computes and publishes thruster and rudder signals based on guidance or teleoperation input.

## Usage

Launch the node as part of your ROS2 system. Configure parameters as needed for your vehicle and environment.

## Dependencies

- ROS2 (rclcpp)
- Custom message types: `neptus_msgs`, `vsa_guidance`
- Guidance and plan database modules

## File Structure

- `src/vsa_supervisor_node.cpp`: Main node implementation.
- `include/`: Header files for guidance, plan database, and control algorithms.
- `data/plan_db.json`: Example plan database.

---
