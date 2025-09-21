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
  | `/vehicle/engage`                                |                                                   | engage command                                                          |
  | `/control/command/emergency_cmd`                 |                                                   | emergency command                                                       |
  
  | Subscribed Topic from DBW                        | Published Topic to Autoware                       | Second Topic (accel & brake pedal etc.) or notes                        |
  | -------------------------------------------------| ------------------------------------------------- | ----------------------------------------------------------------------- |
  | `/raptor_dbw_interface/steering_report`          | `/vehicle/status/steering_status`                 | current steering wheel angle                                            |
  | `/raptor_dbw_interface/wheel_speed_report`       |  odometry                                         | current wheel speed                                                     |
  | `/raptor_dbw_interface/accelerator_pedal_report` | `/vehicle/status/actuation_status`                | current accel pedal                                                     |
  | `/raptor_dbw_interface/brake_report`             | `/vehicle/status/actuation_status`  	             | current brake pedal                                                     |
  | `/raptor_dbw_interface/gear_report`              | `/vehicle/status/gear_status`                     | current gear status                                                     |
  | `/raptor_dbw_interface/driver_input_report`      | `/vehicle/status/turn_indicators_status`          | current turn indicators status                                          |
  | `/raptor_dbw_interface/misc_report`              |                                                   | current status of other parameters (e.g. override_active, can_time_out) |


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
  | `/raptor_dbw_interface/misc_report`              | raptor_dbw_msgs::msg::MiscReport             | current status of other parameters (e.g. override_active, can_time_out) |

### Output Topics

- To Raptor DBW

  | Name                                          | Type                                      | Description                                           |
  | ----------------------------------------------| ------------------------------------------| ----------------------------------------------------- |
  | `/raptor_dbw_interface/accelerator_pedal_cmd` | raptor_dbw_msgs::msg::AcceleratorPedalCmd | accel pedal command                                   |
  | `/raptor_dbw_interface/brake_cmd`             | raptor_dbw_msgs::msg::BrakeCmd            | brake pedal command                                   |
  | `/raptor_dbw_interface/steering_cmd`          | raptor_dbw_msgs::msg::SteeringCmd         | steering wheel angle and angular velocity command     |
  | `/raptor_dbw_interface/gear_cmd`              | raptor_dbw_msgs::msg::GearCmd             | gear command                                          |
  | `/raptor_dbw_interface/misc_cmd`              | raptor_dbw_msgs::msg::MiscCmd             | turn indicators command                               |


- To Autoware

  | Name                                     | Type                                               | Description                                          |
  | ---------------------------------------- | -------------------------------------------------- | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`           | autoware_vehicle_msgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/velocity_status`        | autoware_vehicle_msgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering_status`        | autoware_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights_status`   | autoware_vehicle_msgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status`       | autoware_vehicle_msgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |


Perfect 🚀 Here’s the same structure but now with **checkboxes** so you can track progress directly in your GitHub README.

---
##    Development Guide
## 🔹 Autoware → DBW

### Subscribers

* [x] **ackermann\_sub\_** → listens: `/control/command/control_cmd`
  Callback: `ackermannCmdCallback`

### Publishers

* [x] **accel\_pub\_** → `/raptor_dbw_interface/accelerator_pedal_cmd`
  Used in: `ackermannCmdCallback`

* [x] **brake\_pub\_** → `/raptor_dbw_interface/brake_cmd`
  Used in: `ackermannCmdCallback`

* [x] **steering\_pub\_** → `/raptor_dbw_interface/steering_cmd`
  Used in: `ackermannCmdCallback`

* [x] **gear\_pub\_** → `/raptor_dbw_interface/gear_cmd`
  Used in: `ackermannCmdCallback`

* [x] **enable\_pub\_** → `/raptor_dbw_interface/global_enable_cmd`
  Used in: `ackermannCmdCallback`

---

## 🔹 DBW → Autoware

### Subscribers

* [ ] **steering\_report\_sub\_** → listens: `/raptor_dbw_interface/steering_report`
  Callback: `steeringReportCallback`

* [ ] **brake\_report\_sub\_** → listens: `/raptor_dbw_interface/brake_report`
  Callback: `brakeReportCallback`

* [ ] **wheel\_speed\_report\_sub\_** → listens: `/raptor_dbw_interface/wheel_speed_report`
  Callback: `wheelSpeedReportCallback`

* [ ] **gear\_report\_sub\_** → listens: `/raptor_dbw_interface/gear_report`
  Callback: `gearReportCallback`

* [ ] **misc\_report\_sub\_** → listens: `/raptor_dbw_interface/misc_report`
  Callback: `miscReportCallback`

### Publishers

* [ ] **steering\_status\_pub\_** → `/vehicle/status/steering_status`
  Published in: `steeringReportCallback`

* [ ] **velocity\_status\_pub\_** → `/vehicle/status/velocity_status`
  Published in: `wheelSpeedReportCallback`

* [ ] **control\_mode\_pub\_** → `/vehicle/status/control_mode`
  Published in: `miscReportCallback`

* [ ] **gear\_status\_pub\_** → `/vehicle/status/gear_status`
  Published in: `gearReportCallback`

* [ ] **turn\_indicators\_pub\_** → `/vehicle/status/turn_indicators_status`
  Published in: `miscReportCallback`

* [ ] **hazard\_lights\_pub\_** → `/vehicle/status/hazard_lights_status`
  Published in: `miscReportCallback`

---

## 🔹 Summary

* **Subscribers (6)** → each has a dedicated callback
* **Publishers (11)** → triggered inside the corresponding callbacks

👉 Execution flow:

* `ackermannCmdCallback` → triggers 5 publishers ✅
* `steeringReportCallback` → triggers 1 publisher ⬜
* `wheelSpeedReportCallback` → triggers 1 publisher ⬜
* `gearReportCallback` → triggers 1 publisher ⬜
* `miscReportCallback` → triggers 3 publishers ⬜

---



