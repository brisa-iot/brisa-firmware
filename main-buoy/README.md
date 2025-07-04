# 🌊 BRISA — Smart, Low-Cost Interconnected Buoy for Coastal Monitoring Applications - ESP32 Firmware

**BRISA** is an innovative, low-cost, and interconnected buoy platform specifically designed for **coastal environmental monitoring**.

This repository hosts the **firmware for the ESP32-based BRISA system**. This firmware is the brain of the buoy, responsible for the real-time acquisition, processing, and transmission of vital environmental and water quality data.

---

## 🔧 Platform Overview

The BRISA firmware is built to run on an **Espressif ESP32 Dev Board**, orchestrating a variety of tasks essential for autonomous sensing in challenging marine and coastal environments.

### Key Responsibilities:

* **🧪 Comprehensive Data Collection:** The firmware periodically gathers data from a suite of onboard sensors, including:
    * **Wind Speed and Direction:** Utilizing both analog output and ModBus signals for accurate wind measurements.
    * **Water Quality Parameters:** Monitoring pH, conductivity, and dissolved oxygen via analog sensors, alongside precise water temperature readings via a OneWire interface.
    * **Environmental Conditions:** Capturing atmospheric temperature, pressure, and humidity with a BME280 sensor.
    * **Orientation and Motion:** Tracking the buoy's movement and tilt using an IMU (accelerometer, gyroscope, and magnetometer).
    * **Geolocation and Time:** Providing precise GPS coordinates and UTC timestamps.
* **⚙️ Intelligent Data Processing:** Raw sensor measurements are processed using per-sensor calibration (scale/offset) and averaged over a fixed number of samples for each variable, ensuring high data accuracy.
* **⚡ Smart Power Management:** To maximize operational longevity, the system employs an efficient power management scheme:
    * Measurements occur during configurable **measurement periods** (e.g., every hour) and last for a defined **measurement window** (e.g., 20 minutes).  
      After the window ends, the firmware triggers the **IMU data logging task**, storing high-frequency motion data to internal flash.  
      Once logging is complete, the **ESP32 MCU enters light sleep mode** to conserve power.
    * The MCU can be **woken up by two types of interrupts**:
      - ⏰ **Timer-based interrupt**: triggers the start of the next measurement period.
      - 📡 **UART interrupt**: occurs when a **LoRa message is received**, allowing asynchronous remote activation or reconfiguration.
    * Sensor power rails and subsystems are dynamically enabled or disabled as required, significantly reducing power consumption during idle periods.
* **📡 Robust Data Transmission:** Processed data is transmitted via UART to a **LoRa module** (e.g., LILYGO TTGO SX1262), which then forwards it to receiver node.
* **💾 High-Rate IMU Data Logging:** At the end of each measurement window, high-rate IMU data is logged to flash memory (LittleFS) as `.csv` files, organized by date. This serves as the sole onboard local storage mechanism.

The firmware leverages the **ESP-IDF** and **Arduino** frameworks, structured with **FreeRTOS tasks** to efficiently manage sampling, logging, and communication processes.
---

### 🖼️ System Architecture Diagram

Below is a high-level architecture of the BRISA firmware:

<p align="center">
  <img src="docs/brisa_platform_diagram.svg" alt="BRISA Platform Architecture Diagram" width="80%">
</p>

## Project Structure 📁

```bash
main-buoy-tested/
│
├── .pio/                  
├── .vscode/               
├── include/               # Global header files for the project
├── lib/                   # Project libraries (modularized as classes)
│   ├── common/            # Reusable utilities and global configuration
│   │   ├── adc_measurements.{cpp,h}       # Sensor reading via ADC
│   │   ├── i2c_measurements.{cpp,h}       # Sensor reading via I2C
│   │   ├── one_wire_interface.{cpp,h}     # 1-Wire sensor interface
│   │   ├── uart_interface.{cpp,h}         # UART/RS485 communication
│   │   ├── global_config.h                  # Global configuration data structures
│   │   ├── global_vars.{cpp,h}              # Global shared variables
│   │   ├── params_config.h                  # Parameter macros and constants
│   │   └── filters.h                        # Signal filtering (e.g., moving average)
│   │
│   ├── dynamics/          # System behavior split by dynamics type
│   │   ├── fast_dynamics.{cpp,h}          # Fast dynamics (e.g., IMU)
│   │   ├── medium_dynamics.{cpp,h}        # Medium dynamics (e.g., pressure, humidity)
│   │   ├── slow_dynamics.{cpp,h}          # Slow dynamics (e.g., temperature, status)
│   │   ├── power_monitor.{cpp,h}           # Power usage monitoring
│   │
│   ├── orchestrator/      # Main coordinator of tasks and sensors
│   │   └── orchestrator.{cpp,h}
│   │
│   ├── tasks/             # FreeRTOS-based tasks per subsystem
│   │   ├── _dynamics_task.{cpp,h}          # Base task for dynamic modules: power monitor & med. dynamics
│   │   ├── slow_dynamics_task.{cpp,h}
│   │   ├── imu_logger_task.{cpp,h}
│   │   ├── LoRa_task.{cpp,h}
│   │   └── gps_task.{cpp,h}    
│
├── src/                   # Main application code
│   └── main.cpp           # Entry point: creates the main task (Orchestrator) that controls the entire system,
│                           # manages sensors, communications, and sleep cycles using FreeRTOS
│
├── test/                  
├── .gitignore             
├── platformio.ini         # PlatformIO environment configuration
├── README.md              # Main project documentation
```

---

## 🔌 Hardware Requirements

This firmware has been developed and tested for optimal performance with an **ESP32 DevKit v1** and the following sensor modules:

* **[ESP32 DevKit v1](https://www.espressif.com/en/products/devkits/esp32-devkitc/overview)** – The core Espressif development board.
* **[BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)** – I2C sensor for integrated temperature, humidity, and pressure measurements.
* **[Analog pH Sensor Kit – Gravity by DFRobot](https://wiki.dfrobot.com/Analog_pH_Meter_Pro_SKU_SEN0169)** – Provides analog output for pH level monitoring.
* **[Analog Dissolved Oxygen Sensor – Gravity by DFRobot](https://wiki.dfrobot.com/Gravity__Analog_Dissolved_Oxygen_Sensor_SKU_SEN0237)** – Essential for dissolved oxygen (DO) measurements.
* **[Analog Electrical Conductivity Sensor – Gravity by DFRobot](https://es.aliexpress.com/item/1005003479288815.html#nav-specification)** – For accurate water conductivity readings.
* **[OneWire Water Temperature Sensor – Gravity by DFRobot](https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_)** – A waterproof DS18B20 sensor for precise water temperature.
* **[Analog Anemometer – For Arduino by DFRobot (IP65)](https://wiki.dfrobot.com/Wind_Speed_Sensor_Voltage_Type_0-5V__SKU_SEN0170)** – An IP65-rated wind speed sensor with 0-5V analog output.
* **[ModBus Vane Sensor](https://es.aliexpress.com/item/1005006987700011.html)** – A wind direction sensor featuring ModBus RS485 (4-20mA/0-5V) output.
* **[MAX3485 – ModBus to UART Converter](https://es.aliexpress.com/item/1005006007545162.html)** – Required for converting ModBus signals to UART for ESP32 compatibility.
* **[MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/)** – An I2C IMU providing acceleration, angular velocity, and magnetometer data.
* **[NEO-8M GPS Module](https://www.u-blox.com/en/product/neo-m8-series)** – A reliable UART GPS receiver for geolocation.
* **[LILYGO® TTGO T-Beam V1.1 M8N & SX1262 915 MHz](https://meshtastic.org/docs/hardware/devices/lilygo/tbeam/)** – Utilized for robust long-range wireless data transmission via LoRa.

---

### 📦 External Libraries

The following libraries are declared in the `lib_deps` section of `platformio.ini` and are automatically installed by PlatformIO during build time:

- [`MPU9250_asukiaaa`](https://registry.platformio.org/libraries/asukiaaa/MPU9250_asukiaaa) `@^1.5.13`
- [`TinyGPSPlus`](https://registry.platformio.org/libraries/mikalhart/TinyGPSPlus) `@^1.1.0`
- [`Adafruit BME280 Library`](https://registry.platformio.org/libraries/adafruit/Adafruit%20BME280%20Library) `@^2.3.0`
- [`Adafruit INA219`](https://registry.platformio.org/libraries/adafruit/Adafruit%20INA219) `@^1.2.3`
- [`ModbusMaster`](https://registry.platformio.org/libraries/4-20ma/ModbusMaster) `@^2.0.1`
- [`OneWire`](https://registry.platformio.org/libraries/paulstoffregen/OneWire) `@^2.3.8`
- [`DallasTemperature`](https://registry.platformio.org/libraries/milesburton/DallasTemperature) `@^4.0.4`

> ⚠️ **Calibration Reminder:**  
> All sensors used in BRISA deployments must be individually calibrated prior to installation. Calibration constants (e.g., scale and offset) should be derived from lab or field experiments and entered in `params_config.h` to ensure measurement accuracy and consistency across deployments.

---

## ⚙️ Configuration Overview

BRISA’s configuration is managed through modular, centralized header files—primarily `common/params_config.h`—which encapsulate all critical parameters controlling hardware interfacing and software operation, including:

- **Hardware interfacing:** GPIO pin assignments, ADC channels and attenuation settings, I2C and UART interface parameters, sensor addresses, and communication baud rates.
- **Software parameters:** Measurement intervals and windows, sensor initialization delays, IMU logging sample rates and averaging, power management timing, data storage partitions and file retention policies, and payload size limits for LoRa communication.

For example, `params_config.h` defines:

- **GPIO and communication interfaces:**  
  - LoRa UART pins and baud rate (`TXD0_LORA`, `RXD0_LORA`, `UART_NUM_LORA`, `LORA_BAUD_RATE`)  
  - I2C pins (`I2C_SDA_PIN`, `I2C_SCL_PIN`), sensor addresses (e.g., `MPU9250_ADDRESS`, `BME280_ADDR`)  
  - UART pins and baud for GPS and Modbus RS-485

- **Measurement scheduling and timing:**  
  - General measurement intervals (`MEASUREMENT_INTERVAL_MINUTES`), measurement windows (`MEASUREMENT_WINDOW_MINUTES`)  
  - IMU logger sampling and averaging parameters (`SAMPLE_TIME_IMU_LOGGER_MS`, `SAMPLES_TO_AVERAGE_IMU`)  
  - Sampling times for medium, slow, and power dynamics data

- **ADC configurations for sensors:**  
  - ADC channels, attenuations, scales, and offsets for anemometer, pH, conductivity, dissolved oxygen sensors

- **Data storage:**  
  - File system partition label (`PARTITION_LABEL`), base path (`FS_BASE_PATH`), logs directory, and retention policy (`MAX_NUM_DAYS_LOG_FILES`)  

- **Other system parameters:**  
  - Initial timestamps (`TIMESTAMP_INIT_MS`), sensor initialization delays, maximum payload sizes for LoRa, and more.

This centralized approach allows you to quickly adapt BRISA firmware for different hardware variants or deployment scenarios simply by editing `params_config.h`. It also ensures consistent usage of parameters throughout the codebase, reducing errors and easing maintenance.

---

## 📋 Full Configuration Reference

A detailed, tabulated reference of all parameters—covering pin mappings, sensor calibration constants, communication settings, measurement timing, and logging configuration—is maintained in the official document:

🔗 [**BRISA Configuration Tables (Google Sheets)**](https://docs.google.com/spreadsheets/d/1Mb5BljTlgpizOpPcZprDrpHmMhPn8mJedaDzGkmwkvs/edit?gid=0#gid=0)

⚠️ **Important:** To guarantee consistent, traceable, and replicable system operation, **any changes to parameters must be synchronized both in the Google Sheets document and the firmware header files (`params_config.h`, etc.)**.

---

## 🚧 Boundary Conditions & Valid Configuration Modes

BRISA’s firmware supports **three validated operational profiles**, selected based on daily energy consumption budgets. These define acceptable values for:

- `MEASUREMENT_INTERVAL_MINUTES` – Total cycle duration  
- `MEASUREMENT_WINDOW_MINUTES` – Duration within each cycle where sensors are powered and data is collected

### ⏱️ Valid Operating Modes

| Profile Name      | Interval (`MEASUREMENT_INTERVAL_MINUTES`) | Window (`MEASUREMENT_WINDOW_MINUTES`) | Duty Cycle | Use Case                        |
|-------------------|--------------------------------------------|----------------------------------------|------------|----------------------------------|
| **Low Power**      | 120 minutes                                | 20 minutes                             | ~16.7%     | Long-term deployments, low sun  |
| **Balanced**       | 60 minutes                                 | 10 minutes                             | ~16.7%     | Default mode for normal energy  |
| **High Resolution**| 30 minutes                                 | 5 minutes                              | ~16.7%     | Short-term tests, ample power   |

- **Only these combinations are supported.**  
  The duty cycle is fixed (~16.7%) to ensure consistent energy usage.
- `MEASUREMENT_WINDOW_MINUTES` must **never exceed** `MEASUREMENT_INTERVAL_MINUTES`.

---

### 🗂 Data Logging Constraints

BRISA logs only **IMU data** to internal flash memory, and this occurs **after the window measurement cycle and LoRa transmission completes** to avoid i2c-bus conflicts.

#### 🔄 Circular File Storage

- IMU data is stored as daily CSV files in `/spiffs/logs/`.
- A **fixed circular buffer of 2 days** is enforced:
  - Only the latest **2 days of logs are kept**.
  - Older logs are automatically **overwritten** to manage limited flash space.
  - **This retention period is fixed and cannot be modified** without repartitioning the flash.

#### ⏳ Configurable Logging Window Length and Duration

- The duration of IMU data logging (i.e., how long the logging task runs) is **bounded between 2 and 4 minutes** after the LoRa transmission finishes.
- The length of the IMU measurement window (how long data is collected for each logging cycle) is **configurable via the macro**:

  ```c
  #define MEASUREMENT_WINDOW_IMU  (180000)  // Example: 180 miliseconds

- This macro defines the length of the measurement window during which IMU data is collected and logged.
- he sampling rate for the IMU is fixed at 100 ms, with 5 samples averaged to produce one logged datapoint every 500 ms.
- Thus, total logged samples per cycle depend on `MEASUREMENT_WINDOW_IMU`.

---

### 🧭 Logging Timing and Process

- The logging task is expected to run for a **minimum of 2 minutes** and a **maximum of 4 minutes**.
- Sampling and averaging ensure consistent, manageable file sizes within the fixed retention period.

---

### 📦 Filesystem Backend

Logging uses the **LittleFS** filesystem, which provides wear leveling and robust handling of flash storage:

🔗 [Espressif LittleFS Component Documentation](https://components.espressif.com/components/joltwallet/littlefs)

---

⚠️ **Warning:** Changing retention days or IMU sampling rates beyond recommended values requires adjusting the partition layout (`partitions.csv`) to avoid data loss or memory overflows or even forced reboot system.


### 🧠 Recommendations

- Avoid custom timings unless the firmware is recompiled and energy availability is recalculated.
- All measurements, buffer sizes, and logging volumes are sized to support these modes.
- Changing these values arbitrarily may **violate flash write cycles, RAM buffer limits, or cause power brownouts**.
For example, when adjusting `MEASUREMENT_WINDOW_MINUTES`, make sure to manually update **sample buffer sizes** like:
  ```c
  #define MAX_SAMPLES_MEDIUM_DYNAMICS   (measurement_window_minutes / sample_interval_minutes)
  #define MAX_SAMPLES_POWER_DYNAMICS    (measurement_window_minutes / sample_interval_minutes)
  #define MAX_SAMPLES_SLOW_DYNAMICS     (measurement_window_minutes / sample_interval_minutes)  
- To switch between modes, simply update the two macros in `params_config.h`:
  ```c
  #define MEASUREMENT_INTERVAL_MINUTES    60    // e.g. 30, 60, or 120
  #define MEASUREMENT_WINDOW_MINUTES      10    // e.g. 5, 10, or 20
  ```
-  **Empirical Finding:** On ESP32 with LittleFS and standard memory partitioning, attempting to allocate more than ~3 MB of heap memory for data storage (e.g., large in-memory IMU buffers) will likely fail, even if the total flash size appears sufficient.

--- 
## 🛠️ Build & Flash Instructions (PlatformIO)

BRISA firmware is built and deployed using [**PlatformIO**](https://platformio.org/), a modern development environment that integrates seamlessly with ESP-IDF and Arduino frameworks.

### ✅ Prerequisites

Before building or flashing:

1. **Install PlatformIO Core**:
   - Recommended: [Install PlatformIO as a VSCode extension](https://platformio.org/install/ide?install=vscode).

2. **Clone the BRISA repository**:
   ```bash
   git clone https://github.com/your-org/brisa-firmware.git
   cd brisa-firmware/main-buoy
   ```

### ⚙️ Build & Flash Instructions
To learn how to:
- Compile the project
- Upload the firmware to the device
- Monitor serial output
- Run unit tests and debug

please refer to the official PlatformIO guide for **Espressif32 + Arduino**:  
👉 [Espressif32 Debugging & Unit Testing — PlatformIO Docs](https://docs.platformio.org/en/latest/tutorials/espressif32/arduino_debugging_unit_testing.html)

> 💡 **Tip:** Double-check your `platformio.ini` file to confirm the upload port, serial monitor baud rate, and library dependencies.

---


### 📦 Project Structure Reminder

Key files for build configuration:

- `platformio.ini` – Defines board, framework, partitions, monitor speed, etc.
- `partitions.csv` – Defines flash memory layout (code, spiffs, etc.)
- `lib/` – Contains all firmware logic, organized by module
- `include/` – Public header files if needed
- `src/` – Entrypoint code (e.g., `main.cpp`)


