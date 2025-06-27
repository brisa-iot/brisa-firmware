#include "i2c_measurements.h"
#include "esp_log.h"

static const char* TAG_BME280 = "BME280 Driver";
static const char* TAG_INA219 = "INA219 Driver";
static const char* TAG_IMU = "IMU Driver";

/**
 * BME280 Constructor definition & methods
 */
BME280::BME280(TwoWire* wire_bus_ptr, const BME280_Config_t& config)
    : _wire_bus(wire_bus_ptr), 
      _i2c_address(config.i2c_address), 
      _pressure_hPa_scale(config.pressure_hPa_scale)
{}

esp_err_t BME280::initialize() {
    if (_wire_bus) {
        if(!_bme280.begin(_i2c_address, _wire_bus)) {
            ESP_LOGE(TAG_BME280, "Failed to initialize BME280 at address 0x%02X", _i2c_address);
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG_BME280, "BME280 constructor received null I2C bus pointer!");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG_BME280, "BME280 initialized successfully at address 0x%02X", _i2c_address);
    return ESP_OK;
}

esp_err_t BME280::getRawData(float* temperature, float* pressure, float* humidity) {
    *temperature = _bme280.readTemperature();
    *pressure = _bme280.readPressure() / _pressure_hPa_scale; 
    *humidity = _bme280.readHumidity();
    return ESP_OK;
}


/**
 * INA219 Constructor definition & methods
 */
INA219::INA219(TwoWire* wire_bus_ptr,
         const INA219_Config_t& config)
    : _wire_bus(wire_bus_ptr), 
      _i2c_address_battery(config.i2c_address_battery),
      _i2c_address_solar(config.i2c_address_solar),
      _capacity_mah(config.capacity_mah),
      _dt_soc(config.dt_soc_ms),
      _ina219_battery(config.i2c_address_battery), 
      _ina219_solar(config.i2c_address_solar)      
{}

esp_err_t INA219::initialize() {
    if (_wire_bus) {

        if (!_ina219_battery.begin(_wire_bus)){
            ESP_LOGE(TAG_INA219, "Failed to initialize INA219 for battery at address 0x%02X", _i2c_address_battery);
            return ESP_FAIL;
        }
        _ina219_battery.setCalibration_32V_1A(); 

        if (!_ina219_solar.begin(_wire_bus)){
            ESP_LOGE(TAG_INA219, "Failed to initialize INA219 for solar at address 0x%02X", _i2c_address_solar);
            return ESP_FAIL;
        }
        _ina219_solar.setCalibration_32V_1A();

    } else {
        ESP_LOGE(TAG_INA219, "INA219 constructor received null I2C bus pointer!");
        return ESP_ERR_INVALID_ARG;
    }

    /* ---------------- Find Initial SoC ----------------*/
    float init_battery_voltage = _ina219_battery.getBusVoltage_V();
    int num_voltages = sizeof(_soc_lut[0]) / sizeof(_soc_lut[0][0]);
    int left = 0, right = num_voltages - 1;
    int mid = 0;
    while (left < right) {
        mid = left + (right - left) / 2;
        if (_soc_lut[0][mid] > init_battery_voltage) {
            left = mid + 1;
        } else {
            right = mid;
        }
    }
    if (left == 0) {
        _soc = _soc_lut[1][0];
    } else if (left == num_voltages) {
        _soc = _soc_lut[1][num_voltages - 1];
    } else {
        // Linear interpolation: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        _soc = _soc_lut[1][left - 1] + 
             (init_battery_voltage - _soc_lut[0][left - 1]) * 
             (_soc_lut[1][left] - _soc_lut[1][left - 1]) / 
             (_soc_lut[0][left] - _soc_lut[0][left - 1]);
    }

    ESP_LOGI(TAG_INA219, "INA219 initialized successfully with abttery address at 0x%02X and solar address at 0x%02X. Initial SoC: %.2f%%", 
        _i2c_address_battery, _i2c_address_solar, _soc);
    return ESP_OK;
}

void INA219::updateSoC(float* battery_current) {
    /**
     * TODO: Check units and if this method is working correctly.
     */
    // Coulomb counting: SoC = SoC - (I * dt / C)
    // where I is the current in mA, dt is the time in hours, and C is the capacity in mAh
    float _dt_hours = static_cast<float>(_dt_soc) / 3600000.0f; 
    float charge_change_mah = (*battery_current) * _dt_hours; 
    float soc_change_percent = (charge_change_mah / _capacity_mah) * 100.0f;
    // Substracting because INA219 positions current as positive when discharging
    _soc -= soc_change_percent;
    clampSoC(); 
}

esp_err_t INA219::getRawData(
    float* battery_voltage, float* battery_current, 
    float* solar_voltage, float* solar_current,
    float* soc
) 
{
    if (!_ina219_battery.success() || !_ina219_solar.success()) {
        //ESP_LOGE(TAG_INA219, "INA219 sensor initialization failed!");
        return ESP_FAIL;
    }

    *battery_voltage = _ina219_battery.getBusVoltage_V();
    *battery_current = _ina219_battery.getCurrent_mA();
    *solar_voltage = _ina219_solar.getBusVoltage_V();
    *solar_current = _ina219_solar.getCurrent_mA();

    updateSoC(battery_current);
    *soc = _soc;

    return ESP_OK;
}


/**
 * IMU Constructor definition & methods
 */
IMU::IMU(TwoWire* wire_bus_ptr,
         const IMU_Config_t& config)
    : _wire_bus(wire_bus_ptr), 
      _i2c_address(config.i2c_address),
      _filter_coeffs(config.filter_coeffs),
      _accelX_filter(config.filter_coeffs), 
      _accelY_filter(config.filter_coeffs),
      _accelZ_filter(config.filter_coeffs),
      _gyroX_filter(config.filter_coeffs),
      _gyroY_filter(config.filter_coeffs),
      _gyroZ_filter(config.filter_coeffs),
      _magX_filter(config.filter_coeffs),
      _magY_filter(config.filter_coeffs),
      _magZ_filter(config.filter_coeffs)
{}

// --- Remaining IMU Method Definitions (as discussed) ---
esp_err_t IMU::initialize() {
    if (_wire_bus) {
        _imu.setWire(_wire_bus);
    } else {
        ESP_LOGE(TAG_IMU, "IMU constructor received null I2C bus pointer!");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t chip_id_val = 0;
    if (_imu.readId(&chip_id_val) != 0) { 
        ESP_LOGE(TAG_IMU, "Failed to read MPU9250 chip ID via I2C (check wiring/power/address).");
        return ESP_FAIL; 
    }

    _imu.beginAccel(); 
    _imu.beginGyro(); 
    _imu.beginMag(); 

    return ESP_OK;
}

esp_err_t IMU::getRawData(
    float* accelX, float* accelY, float* accelZ,
    float* gyroX, float* gyroY, float* gyroZ,
    float* magX, float* magY, float* magZ
) {
    _imu.accelUpdate();
    _imu.gyroUpdate();
    _imu.magUpdate();

    *accelX = _imu.accelX();
    *accelY = _imu.accelY();
    *accelZ = _imu.accelZ();
    *gyroX = _imu.gyroX();
    *gyroY = _imu.gyroY();
    *gyroZ = _imu.gyroZ();
    *magX = _imu.magX();
    *magY = _imu.magY();
    *magZ = _imu.magZ();

    return ESP_OK;
}

esp_err_t IMU::getFilteredData(
    float* accelX_f, float* accelY_f, float* accelZ_f,
    float* gyroX_f, float* gyroY_f, float* gyroZ_f,
    float* magX_f, float* magY_f, float* magZ_f
) {
    float raw_accelX, raw_accelY, raw_accelZ;
    float raw_gyroX, raw_gyroY, raw_gyroZ;
    float raw_magX, raw_magY, raw_magZ;

    esp_err_t ret = getRawData(
        &raw_accelX, &raw_accelY, &raw_accelZ,
        &raw_gyroX, &raw_gyroY, &raw_gyroZ,
        &raw_magX, &raw_magY, &raw_magZ
    );

    if (ret != ESP_OK) {
        return ret;
    }

    *accelX_f = _accelX_filter.filter(raw_accelX);
    *accelY_f = _accelY_filter.filter(raw_accelY);
    *accelZ_f = _accelZ_filter.filter(raw_accelZ);
    *gyroX_f = _gyroX_filter.filter(raw_gyroX);
    *gyroY_f = _gyroY_filter.filter(raw_gyroY);
    *gyroZ_f = _gyroZ_filter.filter(raw_gyroZ);
    *magX_f = _magX_filter.filter(raw_magX);
    *magY_f = _magY_filter.filter(raw_magY);
    *magZ_f = _magZ_filter.filter(raw_magZ);
    return ESP_OK;
}

void IMU::resetFilterBuffers() {
    _accelX_filter.reset();
    _accelY_filter.reset();
    _accelZ_filter.reset();
    _gyroX_filter.reset();
    _gyroY_filter.reset();
    _gyroZ_filter.reset();
    _magX_filter.reset();
    _magY_filter.reset();
    _magZ_filter.reset();
}