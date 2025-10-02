# raptor_dbw_interface
Autoware vehicle interface for New Eagle Raptor DBW (drive-by-wire) 

`raptor_dbw_interface` is a ROS 2 package to connect Autoware with Raptor DBW hardware.

### Communication Architecture between Autoware and New Eagle Raptor DBW (drive-by-wire) 

  | Subscribed Topic from Autoware                   | Published Topic via DBW                           | Second Topic (accel & brake pedal etc.) or notes                        |          
  | ------------------------------------------------ | ------------------------------------------------- | ----------------------------------------------------------------------- |
  | `/control/command/control_cmd`                   | `/raptor_dbw_interface/accelerator_pedal_cmd`     | `/raptor_dbw_interface/brake_cmd`&`/raptor_dbw_interface/steering_cmd`  |
  | `/vehicle/command/actuation_cmd`                 | `/raptor_dbw_interface/accelerator_pedal_cmd`     | `/raptor_dbw_interface/brake_cmd`&`/raptor_dbw_interface/steering_cmd`  |
  | `/control/command/gear_cmd`                      | `/raptor_dbw_interface/gear_cmd`                  | gear command                                                            |
  | `/control/command/turn_indicators_cmd`           | `/raptor_dbw_interface/misc_cmd`                  | turn indicators command                                                 |
  | `/control/command/hazard_lights_cmd`             | `/raptor_dbw_interface/misc_cmd`                  | hazard lights command                                                   |
  | `/vehicle/engage`                                | `/raptor_dbw_interface/global_enable_cmd`         | engage command                                                          |
  | `/control/command/emergency_cmd`                 | `/raptor_dbw_interface/accelerator_pedal_cmd`     | `/raptor_dbw_interface/brake_cmd`&`/raptor_dbw_interface/steering_cmd`  |
  
  | Subscribed Topic from DBW                        | Published Topic via Autoware                      | Second Topic (accel & brake pedal etc.) or notes                        |
  | -------------------------------------------------| ------------------------------------------------- | ----------------------------------------------------------------------- |
  | `/raptor_dbw_interface/steering_report`          | `/vehicle/status/steering_status`                 | current steering wheel angle                                            |
  | `/raptor_dbw_interface/wheel_speed_report`       |  odometry                                         | current wheel speed                                                     |
  | `/raptor_dbw_interface/accelerator_pedal_report` | `/vehicle/status/actuation_status`                | current accel pedal                                                     |
  | `/raptor_dbw_interface/brake_report`             | `/vehicle/status/actuation_status`  	             | current brake pedal                                                     |
  | `/raptor_dbw_interface/gear_report`              | `/vehicle/status/gear_status`                     | current gear status                                                     |
  | `/raptor_dbw_interface/driver_input_report`      | `/vehicle/status/turn_indicators_status`          | current turn indicators status                                          |
  | `/raptor_dbw_interface/driver_input_report`      | '`/vehicle/status/hazard_lights_status`'          | current hazard lights status                                            |
  | `/raptor_dbw_interface/misc_report`              | `/vehicle/status/velocity_status`                 | current velocity of the vehicle                                         |


## Input / Output Topics and Their Message Types

### Input Topics

- From Autoware

  | Topic                                  | Type                                              | Description                                           |
  | -------------------------------------- | ------------------------------------------------- | ----------------------------------------------------- |
  | `/control/command/control_cmd`         | autoware_control_msgs::msg::Control               | lateral and longitudinal control command              |
  | `/control/command/gear_cmd`            | autoware_vehicle_msgs::msg::GearCommand           | gear command                                          |
  | `/control/command/turn_indicators_cmd` | autoware_vehicle_msgs::msg::TurnIndicatorsCommand | turn indicators command                               |
  | `/control/command/hazard_lights_cmd`   | autoware_vehicle_msgs::msg::HazardLightsCommand   | hazard lights command                                 |
  | `/vehicle/engage`                      | autoware_vehicle_msgs::msg::Engage                | engage command                                        |
  | `/vehicle/command/actuation_cmd`       | tier4_vehicle_msgs::msg::ActuationCommandStamped  | actuation (accel/brake pedal, steering wheel) command |
  | `/control/command/emergency_cmd`       | tier4_vehicle_msgs::msg::VehicleEmergencyStamped  | emergency command                                     |

- From Raptor DBW

  | Topic                                            | Type                                         | Description                                                             |
  | -------------------------------------------------| ---------------------------------------------| ----------------------------------------------------------------------- |
  | `/raptor_dbw_interface/steering_report`          | raptor_dbw_msgs::msg::SteeringReport         | current steering wheel angle                                            |
  | `/raptor_dbw_interface/wheel_speed_report`       | raptor_dbw_msgs::msg::WheelSpeedReport       | current wheel speed                                                     |
  | `/raptor_dbw_interface/accelerator_pedal_report` | raptor_dbw_msgs::msg::AcceleratorPedalReport | current accel pedal                                                     |
  | `/raptor_dbw_interface/brake_report`             | raptor_dbw_msgs::msg::BrakeReport            | current brake pedal                                                     |
  | `/raptor_dbw_interface/gear_report`              | raptor_dbw_msgs::msg::GearReport             | current gear status                                                     |
  | `/raptor_dbw_interface/driver_input_report`      | raptor_dbw_msgs::msg::DriverInputReport      | current turn indicators status                                          |
  | `/raptor_dbw_interface/misc_report`              | raptor_dbw_msgs::msg::MiscReport             | current velocity of the vehicle                                         |

### Output Topics

- To Raptor DBW

  | Name                                          | Type                                      | Description                                           |
  | ----------------------------------------------| ------------------------------------------| ----------------------------------------------------- |
  | `/raptor_dbw_interface/accelerator_pedal_cmd` | raptor_dbw_msgs::msg::AcceleratorPedalCmd | accel pedal command (position or vehicle speed        |
  | `/raptor_dbw_interface/brake_cmd`             | raptor_dbw_msgs::msg::BrakeCmd            | brake pedal command (position)                        |
  | `/raptor_dbw_interface/steering_cmd`          | raptor_dbw_msgs::msg::SteeringCmd         | steering wheel angle and angular velocity command     |
  | `/raptor_dbw_interface/gear_cmd`              | raptor_dbw_msgs::msg::GearCmd             | gear command                                          |
  | `/raptor_dbw_interface/misc_cmd`              | raptor_dbw_msgs::msg::MiscCmd             | turn indicators and hazard lights command             |


- To Autoware

  | Name                                     | Type                                               | Description                                          |
  | ---------------------------------------- | -------------------------------------------------- | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`           | autoware_vehicle_msgs::msg::ControlModeReport      | control mode (Autonomous or Manual)                  |
  | `/vehicle/status/velocity_status`        | autoware_vehicle_msgs::msg::VelocityReport         | current velocity of the vehicle                      |
  | `/vehicle/status/steering_status`        | autoware_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights_status`   | autoware_vehicle_msgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status`       | autoware_vehicle_msgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |

---

## Development Path
## 🔹 Autoware → DBW

### Timers

* [x] **actuation_timer_** → Publishes: `/vehicle/status/actuation_status`  
  Callback: `publishActuationStatusTimerCallback`

* [x] **autoware_cmd_timer_** → Publishes: `accel, brake, steering, gear, misc, and enable commands`  
  Callback: `publishAutowareControlCmdTimerCallback`

### Subscribers

* [x] **ackermann_sub_** → listens: `/control/command/control_cmd`  
  Callback: `ackermannCmdCallback`

* [x] **gear_cmd_sub_** → listens: `/control/command/gear_cmd`  
  Callback: `gearCmdCallback`

* [x] **turn_cmd_sub_** → listens: `/control/command/turn_indicators_cmd`  
  Callback: `turnCmdCallback`

* [x] **hazard_cmd_sub_** → listens: `/control/command/hazard_lights_cmd`  
  Callback: `hazardCmdCallback`

* [x] **engage_sub_** → listens: `/vehicle/engage`  
  Callback: `engageCallback`

### Publishers

* [x] **accel_pub_** → `/raptor_dbw_interface/accelerator_pedal_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `ackermannCmdCallback`

* [x] **brake_pub_** → `/raptor_dbw_interface/brake_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `ackermannCmdCallback`

* [x] **steering_pub_** → `/raptor_dbw_interface/steering_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `ackermannCmdCallback`

* [x] **gear_pub_** → `/raptor_dbw_interface/gear_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `gearCmdCallback`

* [x] **enable_pub_** → `/raptor_dbw_interface/global_enable_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `engageCallback`

* [x] **misc_pub_** → `/raptor_dbw_interface/misc_cmd`  
  Used in: `publishAutowareControlCmdTimerCallback`, `turnCmdCallback`, `hazardCmdCallback`

---

## 🔹 DBW → Autoware

### Subscribers

* [x] **steering_report_sub_** → listens: `/raptor_dbw_interface/steering_report`  
  Callback: `steeringReportCallback`

* [x] **accel_report_sub_** → listens: `/raptor_dbw_interface/accelerator_pedal_report`  
  Callback: `accelReportCallback`

* [x] **brake_report_sub_** → listens: `/raptor_dbw_interface/brake_report`  
  Callback: `brakeReportCallback`

* [x] **wheel_speed_report_sub_** → listens: `/raptor_dbw_interface/wheel_speed_report`  
  Callback: `wheelSpeedReportCallback`

* [x] **gear_report_sub_** → listens: `/raptor_dbw_interface/gear_report`  
  Callback: `gearReportCallback`

* [x] **driver_input_report_sub_** → listens: `/raptor_dbw_interface/driver_input_report`  
  Callback: `driverInputReportCallback`

* [x] **misc_report_sub_** → listens: `/raptor_dbw_interface/misc_report`  
  Callback: `miscReportCallback`

### Publishers

* [x] **steering_status_pub_** → `/vehicle/status/steering_status`  
  Published in: `steeringReportCallback`

* [x] **velocity_pub_** → `/vehicle/status/velocity_status`  
  Published in: `miscReportCallback`

* [x] **control_mode_pub_** → `/vehicle/status/control_mode`  
  Published in: `miscReportCallback`

* [x] **gear_status_pub_** → `/vehicle/status/gear_status`  
  Published in: `gearReportCallback`

* [x] **turn_indicators_pub_** → `/vehicle/status/turn_indicators_status`  
  Published in: `driverInputReportCallback`

* [x] **hazard_lights_pub_** → `/vehicle/status/hazard_lights_status`  
  Published in: `driverInputReportCallback`

* [x] **actuation_status_pub_** → `/vehicle/status/actuation_status`  
  Published in: `publishActuationStatusTimerCallback`, updated in: `accelReportCallback`, `brakeReportCallback`, `steeringReportCallback`

---

## 🔹 Summary

* **Subscribers (12)** → each has a dedicated callback  
* **Publishers (11)** → triggered inside corresponding callbacks/timers  

**Execution flow:**  

* `ackermannCmdCallback` → accel, brake, steering | fill messages (from control_cmd)  
* `gearCmdCallback` → gear | fill message (from gear_cmd)  
* `turnCmdCallback` + `hazardCmdCallback` → misc | fill messages (from turn/hazard commands)  
* `engageCallback` → enable | fill message (from engage flag)  
* `steeringReportCallback` → steering_status, actuation_status (steer) | publish steering_status, update actuation_status  
* `accelReportCallback` → actuation_status (accel) | update actuation_status only  
* `brakeReportCallback` → actuation_status (brake) | update actuation_status only  
* `driverInputReportCallback` → turn_indicators, hazard_lights | publish turn/hazard reports  
* `miscReportCallback` → velocity, control_mode | publish velocity & control_mode  
* `publishActuationStatusTimerCallback` → actuation_status | publish periodically (actuation status)  
* `publishAutowareControlCmdTimerCallback` → accel, brake, steering, gear, misc, enable | publish all control commands periodically

---

## 🔹 TO DO List

- [ ] Local odometry data can be provided using encoder values. DBW already publishes wheel speed and wheel position on the following topics:
/raptor_dbw_interface/wheel_speed_report & /raptor_dbw_interface/wheel_position_report

- [ ] Velocity status should be derived from the average velocity of all wheels, using the /raptor_dbw_interface/wheel_speed_report topic.

- [ ] Control mode status is currently published inside the misc callback. This logic should be moved into the enable callback for better structure.

- [ ] The vehicle interface package currently supports velocity-reference-based driving; integrate it to also support pedal-position-based reference from Autoware.

---
