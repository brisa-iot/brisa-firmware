# 🌊 BRISA — ESP32 Firmware (Versión Simplificada)

**BRISA** es una plataforma de boya interconectada, de bajo costo, diseñada para **monitoreo ambiental y de calidad de agua** en zonas costeras.  
Este repositorio contiene la **versión simplificada del firmware** desarrollada para pruebas de hardware, validación de consumo energético y testeo de transmisión básica por LoRa.

---

## 📌 Estado Actual de Implementación

Esta versión **NO** incluye toda la lógica avanzada de la versión final.  
Actualmente implementa solo lo siguiente:

- ✅ **Lectura básica de sensores** (IMU, ambientales, agua).
- ✅ **Transmisión de datos por LoRa** en formato JSON.
- ✅ **Selección de escenarios de operación** mediante macros.
- ✅ **Tasa de muestreo común** para todos los sensores.

> 🚫 **Nota:** No se incluyen tareas concurrentes (FreeRTOS), registro local de datos IMU ni manejo de archivos LittleFS.  
> Esta versión se usa para validar consumo energético, verificar hardware y optimizar parámetros antes de migrar a la versión final.

---

## ⚙️ Configuración Actual

Todos los parámetros configurables están en `main.cpp`:

```cpp
#define MEASUREMENT_INTERVAL_US     30000000    // intervalo entre ciclos (30 s)
#define SAMPLING_TIME_WINDOW_MS     10000       // tasa de recolocción --> envío de datos dentro de la ventana
#define NUM_SAMPLES_PER_WINDOW      30          // número de muestras a recolectar
```

---

## 🔌 Requisitos de Hardware

Esta versión simplificada del firmware ha sido desarrollada y probada para una **ESP32 DevKit v1** y los siguientes módulos de sensores:

* **[ESP32 DevKit v1](https://www.espressif.com/en/products/devkits/esp32-devkitc/overview)** – Placa de desarrollo principal basada en el chip ESP32.
* **[BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)** – Sensor I2C para mediciones de temperatura, humedad y presión atmosférica.
* **[MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/)** – IMU por I2C para datos de aceleración, velocidad angular y magnetómetro.
* **[Módulo GPS NEO-8M](https://www.u-blox.com/en/product/neo-m8-series)** – Receptor GPS UART para geolocalización y sincronización horaria.
* **[LILYGO® TTGO T-Beam V1.1 M8N & SX1262 915 MHz](https://meshtastic.org/docs/hardware/devices/lilygo/tbeam/)** – Módulo LoRa para transmisión inalámbrica de largo alcance.
* **[Sensor de pH analógico – Gravity by DFRobot](https://wiki.dfrobot.com/Analog_pH_Meter_Pro_SKU_SEN0169)**  
* **[Sensor de Oxígeno Disuelto analógico – Gravity by DFRobot](https://wiki.dfrobot.com/Gravity__Analog_Dissolved_Oxygen_Sensor_SKU_SEN0237)**  
* **[Sensor de Conductividad Eléctrica – Gravity by DFRobot](https://wiki.dfrobot.com/SKU_SEN0451_Gravity_Analog_Electrical_Conductivity_Sensor_PRO_K_1)**  
* **[Sensor de Temperatura DS18B20 OneWire](https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_)** – Sensor impermeable para temperatura del agua.
* **[Anemómetro y Veleta](https://wiki.dfrobot.com/Wind_Speed_Sensor_Voltage_Type_0-5V__SKU_SEN0170)** – Sensor de velocidad del viento con salida analógica.
* **[MAX3485 – Convertidor ModBus a UART](https://es.aliexpress.com/item/1005006007545162.html)** – Requerido para sensores ModBus.

---

## 📦 Librerías Externas

Las siguientes librerías están declaradas en la sección `lib_deps` del archivo `platformio.ini` y se instalan automáticamente con PlatformIO:

- [`MPU9250_asukiaaa`](https://registry.platformio.org/libraries/asukiaaa/MPU9250_asukiaaa) `@^1.5.13`
- [`TinyGPSPlus`](https://registry.platformio.org/libraries/mikalhart/TinyGPSPlus) `@^1.1.0`
- [`Adafruit BME280 Library`](https://registry.platformio.org/libraries/adafruit/Adafruit%20BME280%20Library) `@^2.3.0`
- [`ModbusMaster`](https://registry.platformio.org/libraries/4-20ma/ModbusMaster) `@^2.0.1`
- [`OneWire`](https://registry.platformio.org/libraries/paulstoffregen/OneWire) `@^2.3.8`
- [`DallasTemperature`](https://registry.platformio.org/libraries/milesburton/DallasTemperature) `@^4.0.4`

---

## 🗂️ Estructura del Proyecto

El proyecto sigue una **arquitectura modular y orientada a clases**, diseñada para mantener el código organizado y facilitar futuras ampliaciones.  
A diferencia de la versión completa del firmware BRISA, aquí los sensores **no se agrupan por dinámica de muestreo** (rápida, media, lenta), sino por el **tipo de variable medida**: agua, ambiente, potencia, IMU y posición.  
Cada módulo encapsula su propia lógica en clases.

```bash
main-buoy-tested/
│
├── .pio/                  
├── .vscode/               
├── include/               
├── lib/                   # Librerías del proyecto (modularizadas como clases)
│   ├── common/            # Clases comunes: ADC, I2C, UART, filtros de señal
│   │   ├── adc_lib.cpp
│   │   ├── adc_lib.h
│   │   ├── filters.h
│   │   ├── i2c_lib.cpp
│   │   ├── i2c_lib.h
│   │   ├── uart_lib.cpp
│   │   ├── uart_lib.h
│   │
│   ├── data_structs/      # Estructuras de datos globales compartidas
│   │   └── data_structs.h
│   │
│   ├── environment/       # Clase para sensores de ambiente (BME280, GPS)
│   │   ├── environment.cpp
│   │   └── environment.h
│   │
│   ├── power/             # Clase para medición de potencia (batería, solar)
│   │   ├── power.cpp
│   │   └── power.h
│   │
│   ├── water/             # Clase para calidad de agua (pH, conductividad, DO)
│   │   ├── water.cpp
│   │   └── water.h
│
├── src/                   # Código principal de la aplicación
│   └── main.cpp           # Entrypoint: instancia clases
│
├── test/                  
├── .gitignore             
├── platformio.ini         # Configuración de PlatformIO
├── README.md              # Documentación principal del proyecto
```

---

## 🛠️ Instrucciones de Compilación y Flasheo (PlatformIO)

El firmware BRISA se construye y despliega usando [**PlatformIO**](https://platformio.org/), un entorno de desarrollo moderno que se integra perfectamente con los frameworks ESP-IDF y Arduino.

### ✅ Requisitos Previos

Antes de compilar o flashear:

1. **Instalar PlatformIO Core**:  
   - Recomendado: [Instalar PlatformIO como extensión de VSCode](https://platformio.org/install/ide?install=vscode).

2. **Clonar el repositorio de BRISA**:  
   ```bash
   git clone https://github.com/your-org/brisa-firmware.git
   cd brisa-firmware
### ⚙️ Instrucciones de Compilación y Flasheo  
Para aprender cómo:  
- Compilar el proyecto  
- Subir el firmware al dispositivo  
- Monitorizar la salida serial  
- Ejecutar pruebas unitarias y hacer debug  

Consulta la guía oficial de PlatformIO para **Espressif32 + Arduino**:  
👉 [Espressif32 Debugging & Unit Testing — PlatformIO Docs](https://docs.platformio.org/en/latest/tutorials/espressif32/arduino_debugging_unit_testing.html)

> 💡 **Consejo:** Verifica tu archivo `platformio.ini` para confirmar el puerto de subida, la velocidad del monitor serial y las dependencias de librerías.


