# 📦 BRISA Firmware Libraries

This directory contains all the **private and project-specific libraries** required to run the firmware for the **BRISA** system. Its modular architecture ensures clarity, maintainability, and easy extensibility.

---

## 📁 Directory Structure: `lib/`

The project follows a hierarchical and modular organization, grouped by functionality:

```bash
lib/
│
├── common/                 # Reusable utilities and global configuration
│   ├── adc_measurements.{cpp,h}       # Sensor reading via ADC
│   ├── i2c_measurements.{cpp,h}       # Sensor reading via I2C
│   ├── one_wire_interface.{cpp,h}     # 1-Wire sensor interface
│   ├── uart_interface.{cpp,h}         # UART/RS485 communication
│   ├── global_config.h                # Global config data structures
│   ├── global_vars.{cpp,h}            # Global shared variables
│   ├── params_config.h                # Parameter macros and constants
│   └── filters.h                      # Signal filtering (e.g., moving average)
│
├── dynamics/              # System behavior split by dynamics type
│   ├── fast_dynamics.{cpp,h}          # Fast dynamics (e.g., IMU)
│   ├── medium_dynamics.{cpp,h}        # Medium dynamics (e.g., pressure, humidity)
│   ├── slow_dynamics.{cpp,h}          # Slow dynamics (e.g., temperature, status)
│   ├── power_monitor.{cpp,h}          # Power usage monitoring
│
├── orchestrator/          # Main coordinator of tasks and sensors
│   └── orchestrator.{cpp,h}
│
├── tasks/                 # FreeRTOS-based tasks per subsystem
│   ├── _dynamics_task.{cpp,h}         # Base task for dynamic modules: power monitor & med. dynamics
│   ├── slow_dynamics_task.{cpp,h}
│   ├── imu_logger_task.{cpp,h}
│   ├── LoRa_task.{cpp,h}
│   └── gps_task.{cpp,h}
│
└── README.md              # This file
```
---

## 🧩 Modular Design Principles

- **Layered Architecture:**  
  The firmware is organized in logical layers—common utilities, sensor-specific modules, task handlers, and orchestration logic—to ensure clarity and separation of concerns.

- **Reusable Components:**  
  Core functionality like ADC/I2C/UART communication and filtering algorithms is centralized in `common/` to promote code reuse across modules.

- **Low Coupling & Clear Interfaces:**  
  Each module is designed to operate independently with minimal dependencies, exposing only well-defined headers and interfaces.

- **Real-Time Task Scheduling:**  
  All runtime operations are implemented as FreeRTOS tasks, ensuring responsive, parallel execution of sensor polling, data logging, communication, and coordination.

- **Cross-Platform Build Support:**  
  Fully compatible with PlatformIO and ESP-IDF, combining the ease of Arduino APIs with the low-level control and performance of native ESP32 development.

---

## 📚 Useful References

- 📘 [PlatformIO Library Management Guide](https://docs.platformio.org/page/librarymanager/config.html)  
  Documentation for managing local and remote libraries in a PlatformIO project.

- 📗 [ESP-IDF Component Integration (CMake)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)  
  How to structure reusable firmware components with proper dependency resolution in ESP-IDF projects.
