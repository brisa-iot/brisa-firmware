#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/uart.h>
#include "water.h"
#include "data_structs.h"
#include "environment.h"
#include "power.h"
#include <HardwareSerial.h>


#define SWITCH_LOAD_PIN 23 

#define MEASUREMENT_INTERVAL_US     30000000
#define MEASUREMENT_WINDOW_MS       10000
#define NUM_SAMPLES_PER_WINDOW      30


// ------------- Sensor Wrappers ------------- //
WaterWrapper waterWrapper;
EnvironmentWrapper enviromentWrapper;
SensorData sensorData;
IMUSensor imuSensor;
GPSSensor gps_neo_8m;
PowerWrapper powerwrapper;
// --------------------------------------------- //

// ------------- Config. Parameters ------------- //
uint64_t wakeupTs = MEASUREMENT_INTERVAL_US; 
uint64_t windowTs = MEASUREMENT_WINDOW_MS;

uint16_t num_samples_per_window = NUM_SAMPLES_PER_WINDOW; 

unsigned long currentTime = millis();
unsigned long previousTime = 0;
// --------------------------------------------- //


void read_sensors(char* message, size_t messageSize);
void uart_send(const char* message);
void uart_receive();
void init_sensors();


void setup() {
  Wire.begin(21, 22); 
  Serial.begin(115200);
  delay(1000);

  uart_set_wakeup_threshold(UART_NUM_0, 3); 
	esp_sleep_enable_uart_wakeup(UART_NUM_0);
	esp_sleep_enable_timer_wakeup(wakeupTs); 

  pinMode(SWITCH_LOAD_PIN, OUTPUT);
  digitalWrite(SWITCH_LOAD_PIN, LOW); 
  delay(1000); 
}

void loop() {
  currentTime = millis();
	esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    
    digitalWrite(SWITCH_LOAD_PIN, HIGH);
    delay(2000);
    init_sensors();
    
    for (int i = 0; i < num_samples_per_window; i++){
      char message[237];
      read_sensors(message, sizeof(message));
      uart_send(message);  

      previousTime = millis();
      delay(windowTs);
    }

    esp_sleep_enable_timer_wakeup(wakeupTs); 
    digitalWrite(SWITCH_LOAD_PIN, LOW); 

	} else if (wakeup_reason == ESP_SLEEP_WAKEUP_UART) {
		esp_sleep_enable_timer_wakeup(wakeupTs - (currentTime - previousTime)*1000); 
    uart_receive();
	} else {
      esp_sleep_enable_timer_wakeup(wakeupTs - (currentTime - previousTime)*1000); 
  }
  Serial.flush();
	esp_light_sleep_start(); 
}


void read_sensors(char* message, size_t messageSize) {
  imuSensor.get_accel_gyro_mag(&sensorData);
  enviromentWrapper.readAll(&sensorData);
  powerwrapper.readAll(&sensorData);
  gps_neo_8m.get_location(&sensorData);
  gps_neo_8m.get_gps_time(&sensorData);
  waterWrapper.readAll(&sensorData);

  // Timestamp
  unsigned long timestamp = sensorData.gpsData.timestamp_s; 

  // Simulate environmental data
  float temperature_air = sensorData.envData.temperature;     
  float humidity = sensorData.envData.humidity;            
  float pressure = sensorData.envData.pressure;         
  float wind_speed = sensorData.envData.windSpeed;           
  int wind_direction = sensorData.envData.windDirection;                 
  
  float water_temp = sensorData.waterData.temperature;          
  float ph = random(600, 850) / 100.0;                 
  float conductivity = sensorData.waterData.sigma;             
  float dissolved_oxygen = random(500, 1200) / 100.0;  

  // Simulate GPS/GNSS
  float latitude = sensorData.gpsData.latitude;  
  float longitude = sensorData.gpsData.longitude; 
  float altitude = sensorData.gpsData.altitude;                    

  // Simulate IMU
  float accel_x = sensorData.imuPosData.accelX;        
  float accel_y = sensorData.imuPosData.accelY;
  float accel_z = sensorData.imuPosData.accelZ;        
  float gyro_x = sensorData.imuPosData.gyroX;          
  float gyro_y = sensorData.imuPosData.gyroY;
  float gyro_z = sensorData.imuPosData.gyroZ;          

  // SOC - V - A values
  float SOC = sensorData.powerData.batterySoC; 

  // Create JSON message
  snprintf(message, messageSize,
          "{"
          "\"ts\":%lu,"
          "\"ta\":%.1f,\"hr\":%.1f,\"pa\":%.1f,"
          "\"ws\":%.1f,\"wd\":%d,"
          "\"tw\":%.1f,\"ph\":%.2f,\"ec\":%.1f,\"do\":%.2f,"
          "\"lat\":%.4f,\"lon\":%.4f,\"alt\":%.1f,"
          "\"a\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
          "\"g\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
          "\"soc\":%.2f"
          "}",
          timestamp,
          temperature_air, humidity, pressure,
          wind_speed, wind_direction,
          water_temp, ph, conductivity, dissolved_oxygen,
          latitude, longitude, altitude,
          accel_x, accel_y, accel_z,
          gyro_x, gyro_y, gyro_z, SOC);
}

void uart_send(const char* message) {
  Serial.print(message);
}

void uart_receive() {
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
  }
}

void init_sensors(){
  // Initialize each sensor (ADC setup, etc.)
  enviromentWrapper.initializeAll();
  //Serial.println("Enviroment ready");
  delay(50);
  imuSensor.initialize();
  //Serial.println("IMU ready");
  gps_neo_8m.initialize();
  delay(50);
  powerwrapper.initializeAll();
  delay(50);
  waterWrapper.initializeAll();
  //Serial.println("Water ready");
  delay(50);
}
  