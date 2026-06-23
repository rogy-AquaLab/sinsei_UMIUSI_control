# sinsei_umiusi_control

[![Main CI](https://github.com/rogy-AquaLab/sinsei_UMIUSI_control/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/rogy-AquaLab/sinsei_UMIUSI_control/actions/workflows/main.yml)

## Structure

```mermaid
flowchart LR
    classDef topics fill:#ffe3f2,stroke:#d84f9b,color:#222;
    classDef controller fill:#fff0df,stroke:#ff9838,color:#222;
    classDef hwc fill:#e8f2ff,stroke:#4a9df5,color:#222;
    classDef hw fill:#e5faf7,stroke:#57cfc0,color:#222;
    classDef state fill:#eaf8ea,stroke:#62b36f,color:#222;
    classDef topicEdge stroke:#d84f9b,stroke-width:3px,color:#d84f9b;
    classDef commandEdge stroke:#7a5af8,stroke-width:3px,color:#7a5af8;
    classDef stateEdge stroke:#62b36f,stroke-width:3px,color:#62b36f;
    classDef physicalEdge stroke:#4a9df5,stroke-width:3px,color:#4a9df5;

    subgraph ROS["ROS Topics"]
        direction TB
        CMD["cmd/(indicator_led, main_power, led_tape, headlights)_output<br/>cmd/thruster_runnable_all<br/>cmd/target"]
        MANUAL["cmd/direct/thruster_controller/output_*<br/>cmd/direct/thruster_controller/output_all"]
        STATE["state/(main_power_enabled, imu_state, thruster_state_all)<br/>state/(low_power_circuit_info, high_power_circuit_info)"]
    end

    subgraph CTRL["Controllers"]
        direction TB
        GATE["GateController"]
        ATT["AttitudeController"]
        THR["ThrusterController x4"]
    end

    subgraph HWC["Hardware Components"]
        direction TB
        LEDC["Indicator LED<br/>(System)"]
        CANC["CAN<br/>(System)"]
        IMUC["IMU<br/>(Sensor)"]
        HLC["Headlights<br/>(System)"]
    end

    subgraph HW["Hardwares"]
        direction TB
        LED["Indicator LED"]
        MCP["CAN Controller<br/>(MCP2515)"]
        VESC["VESC Boards x4<br/>+ Power Board"]
        BNO["IMU<br/>(BNO055)"]
        HL["High Beam<br/>Low Beam<br/>IR"]
    end

    CMD topic_inputs@--> GATE
    GATE topic_outputs@--> STATE

    GATE gate_to_attitude@-- "attitude_controller/target_(orientation, velocity).*" --> ATT
    ATT attitude_to_gate@-- "attitude_controller/imu/(quaternion, acceleration, angular_velocity).*<br/>attitude_controller/thruster_controller_*/thruster/esc/rpm" --> GATE

    GATE gate_to_thruster@-- "thruster_controller_*/(esc, servo)/runnable" --> THR
    ATT attitude_to_thruster@-- "thruster_controller_*/esc/duty_cycle<br/>thruster_controller_*/servo/angle" --> THR
    MANUAL manual_override@-- "manual override" --> THR
    THR thruster_to_gate@-- "thruster_controller_*/esc/(mode, duty_cycle)<br/>thruster_controller_*/servo/(mode, angle)<br/>thruster_controller_*/thruster/esc/(voltage, water_leaked)" --> GATE

    GATE gate_to_indicator@-- "indicator_led/enabled" --> LEDC
    LEDC indicator_gpio@-- "GPIO" --> LED

    GATE gate_to_can@-- "main_power/enabled<br/>led_tape/color" --> CANC
    THR thruster_to_can@-- "thrusterN/esc/(allowed, duty_cycle)<br/>thrusterN/servo/(allowed, angle)" --> CANC
    CANC can_to_gate@-- "main_power/(battery_voltage, battery_current, temperature, water_leaked)<br/>can/health" --> GATE
    CANC can_to_thruster@-- "thrusterN/esc/(rpm, voltage, water_leaked)" --> THR
    CANC can_spi@-- "SPI" --> MCP
    MCP vesc_can@-- "CAN" --> VESC

    IMUC imu_to_attitude@-- "imu/(quaternion, acceleration, angular_velocity).*" --> ATT
    IMUC imu_to_gate@-- "imu/temperature<br/>imu/health" --> GATE
    IMUC imu_i2c@-- "I2C" --> BNO

    GATE gate_to_headlights@-- "headlights/(high_beam, low_beam, ir)_enabled" --> HLC
    HLC headlights_gpio@-- "GPIO" --> HL

    class topic_inputs,topic_outputs,manual_override topicEdge;
    class gate_to_attitude,gate_to_thruster,attitude_to_thruster,gate_to_indicator,gate_to_can,thruster_to_can,gate_to_headlights commandEdge;
    class attitude_to_gate,thruster_to_gate,can_to_gate,can_to_thruster,imu_to_attitude,imu_to_gate stateEdge;
    class indicator_gpio,can_spi,vesc_can,imu_i2c,headlights_gpio physicalEdge;

    class CMD,MANUAL topics;
    class STATE state;
    class GATE,ATT,THR controller;
    class LEDC,CANC,IMUC,HLC hwc;
    class LED,MCP,VESC,BNO,HL hw;
```

### Legend

| Visual       | Meaning                            |
| ------------ | ---------------------------------- |
| Pink nodes   | ROS topics (command)               |
| Green nodes  | ROS topics (state)                 |
| Orange nodes | `ros2_control` controllers         |
| Blue nodes   | `ros2_control` hardware components |
| Cyan nodes   | Physical hardware                  |
| Pink edges   | ROS topics                         |
| Purple edges | `ros2_control` command interfaces  |
| Green edges  | `ros2_control` state interfaces    |
| Blue edges   | Physical buses                     |

## Topics

All types of messages are defined in [sinsei_UMIUSI_msgs](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs).

### Subscribed by `GateController`

| Topic Name                  | Type name (URL to `.msg` file)                                                                                    | Description                                                                               |
| --------------------------- | ----------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `cmd/indicator_led_output`  | [`IndicatorLedOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/IndicatorLedOutput.msg)   | Indicator LED output (Enabled / Disabled)                                                 |
| `cmd/main_power_output`     | [`MainPowerOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/MainPowerOutput.msg)         | Main power output (Enabled / Disabled)                                                    |
| `cmd/led_tape_output`       | [`LedTapeOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/LedTapeOutput.msg)             | LED tape output (RGBA values)                                                             |
| `cmd/headlights_output`     | [`HeadlightsOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/HeadlightsOutput.msg)       | Headlights output (Enabled / Disabled) for each of High beam, Low beam, and IR            |
| `cmd/thruster_runnable_all` | [`ThrusterRunnableAll`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterRunnableAll.msg) | Thruster runnable status (True / False for each of ESC and Servo motor) for each thruster |
| `cmd/target`                | [`Target`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/Target.msg)                           | Target of the robot (Velocity and Angular vector)                                         |

### Subscribed by `ThrusterController`

| Topic Name                                  | Type name (URL to `.msg` file)                                                                                | Description                                                                             |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| `cmd/direct/thruster_controller/output_lf`  | [`ThrusterOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterOutput.msg)       | Thruster output (`ThrusterRunnable`, ESC duty and Servo angle) for Left Front thruster  |
| `cmd/direct/thruster_controller/output_lb`  | [`ThrusterOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterOutput.msg)       | Thruster output (`ThrusterRunnable`, ESC duty and Servo angle) for Left Back thruster   |
| `cmd/direct/thruster_controller/output_rb`  | [`ThrusterOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterOutput.msg)       | Thruster output (`ThrusterRunnable`, ESC duty and Servo angle) for Right Back thruster  |
| `cmd/direct/thruster_controller/output_rf`  | [`ThrusterOutput`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterOutput.msg)       | Thruster output (`ThrusterRunnable`, ESC duty and Servo angle) for Right Front thruster |
| `cmd/direct/thruster_controller/output_all` | [`ThrusterOutputAll`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterOutputAll.msg) | Thruster output (`ThrusterOutput`) for each thruster                                    |

### Published by `GateController`

| Topic Name                      | Type name (URL to `.msg` file)                                                                                      | Description                                                                             |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| `state/main_power_enabled`      | [`MainPowerEnabled`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/MainPowerEnabled.msg)         | Main power enabled / disabled status                                                    |
| `state/imu_state`               | [`ImuState`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ImuState.msg)                         | IMU status (Acceleration, Angular Velocity, Quaternion, etc.)                           |
| `state/thruster_state_all`      | [`ThrusterStateAll`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/ThrusterStateAll.msg)         | Thruster status (Mode, Duty Cycle, Angle, and RPM) for each thruster                    |
| `state/low_power_circuit_info`  | [`LowPowerCircuitInfo`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/LowPowerCircuitInfo.msg)   | Health status (`0` for ok / `1` for error) for each low-power circuit                   |
| `state/high_power_circuit_info` | [`HighPowerCircuitInfo`](https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs/tree/main/msg/HighPowerCircuitInfo.msg) | Health-related values for each high-power circuit (Voltage, Current, WaterLeaked, etc.) |
