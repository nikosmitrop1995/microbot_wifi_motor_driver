# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-01-13

### Added
- Initial release of microbot WiFi motor driver package
- WiFi connectivity support for ESP32S3 microcontroller
- micro-ROS integration for ROS2 communication over WiFi (UDP)
- Motor driver implementation with dual H-bridge control
- PID controller for closed-loop velocity control of left and right wheels
- Encoder feedback with interrupt-based counting for both wheels
- ROS2 cmd_vel subscriber for receiving velocity commands
- ROS2 wheel_velocity publisher for feedback
- Configurable PID gains for independent left and right wheel tuning (exposed as dynamic parameters)
- 50 Hz control loop for real-time motor control
- Exponential moving average (EMA) filtering for wheel speed measurements
- PlatformIO project configuration for Seeed XIAO ESP32S3 board
- ROS2 package structure with setup.py and package.xml
- Command-line tools for compiling and uploading firmware (`ros2 run microbot_wifi_motor_driver compile` and `upload`)
- WiFi connection initialization function with built-in LED indicator
- Comprehensive documentation in README.md
- PID header and source files (`pid.h` and `pid.cpp`)
- Driver header and source files (`driver.h` and `driver.cpp`)
- micro-ROS integration files (`uros.h` and `uros.cpp`)
- SLP pin functionality for motor driver control

### Changed
- Replaced hardcoded WiFi credentials and IP address with placeholders in `main.cpp` for better security
- Updated README.md to reflect package name change and enhanced installation instructions
- Improved code to update parameters dynamically
- Separated files based on utility (driver, PID, micro-ROS)
- Updated gitignore to exclude unnecessary files from version control

### Fixed
- Changed pinout configuration of the driver
- Improved dynamic parameter update mechanism

### Technical Details
- Target platform: Seeed XIAO ESP32S3
- Framework: Arduino
- ROS2 distribution: Humble
- Transport: WiFi (UDP)
- Control frequency: 50 Hz
- Wheel radius: 0.0598 m
- Wheel base: 0.109 m
- Encoder resolution: 1954 counts per revolution
