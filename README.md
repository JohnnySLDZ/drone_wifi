# Quadcopter Drone Control System with a Real-Time Web Interface
## (ESP32-Based Flight Controller with MPU6050, BMP280, and Wireless Monitoring)

## Description
This project is a quadcopter drone with a control system featuring a web interface for real-time monitoring and parameter adjustment. The drone uses an ESP32 as the main controller, equipped with sensors such as the MPU6050 (for orientation measurements: yaw, pitch, and roll) and the BMP280 (for altitude measurement). The web interface displays this data in real time and allows the user to start/stop the drone and adjust the throttle (motor speed). The system implements PID control for stabilization and uses WiFi communication for data transmission.

## Video Demo⬇️⬇️⬇️:  
[![Alt text](https://img.youtube.com/vi/OVLGnlZOAvw/0.jpg)](https://www.youtube.com/watch?v=OVLGnlZOAvw)

<[Video](https://youtube.com/shorts/OVLGnlZOAvw)>

## Drone Wiring and Electrical Diagram
![electric_diagram](https://github.com/user-attachments/assets/2f5bc61b-deb1-4ce0-82f0-249bf4e5d390)

## Web Interface
![user_interface](https://github.com/user-attachments/assets/53f3ac72-0abf-4f97-a271-23c2388f39ae)

## Requirements
[![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-1.8.19-blue)](https://www.arduino.cc/en/software)

### Libraries
| Library                 | Primary Use                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `ESC.h`                | Brushless motor control using PWM signals.                                 |
| `WiFi.h`               | WiFi network connection and management for ESP32.                           |
| `Adafruit_BMP280.h`    | Interface for BMP280 sensor (pressure and altitude measurement).           |
| `SimpleKalmanFilter.h` | Kalman filter implementation for sensor reading smoothing.                 |
| `AsyncTCP.h`           | Asynchronous TCP connection handling for web server.                        |
| `ESPAsyncWebServer.h`  | Asynchronous web server for ESP32.                                          |
| `Arduino_JSON.h`       | JSON data manipulation.                                                    |
| `SPIFFS.h`             | File system for web resource storage in flash memory.                      |
| `I2Cdev.h`             | I2C communication facilitator for devices like MPU6050.                   |
| `MPU6050_6Axis_MotionApps20.h` | Advanced MPU6050 sensor usage with DMP (Digital Motion Processor).     |
| `Wire.h`               | I2C communication for Arduino/ESP32 platforms.                             |

**Web Interface based on:**
[This MPU6050 Project](https://randomnerdtutorials.com/esp32-mpu-6050-web-server/)
