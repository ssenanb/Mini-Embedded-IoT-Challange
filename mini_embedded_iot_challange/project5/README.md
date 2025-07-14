# CLI-With-MPU6050-IMU
## Description

In this project, the STM32 continously reads accelerometer and gyroscope data from the IMU sensor. Since the raw data is often noisy and affected by various environmental factors, an offset calibration is implemented to improve measurement accuracy. The system includes a Command Line Interface (CLI) over UART, allowing the user to interact with the system by sending commands. 

This project is designed for developers who want to test, calibrate, and visualize raw IMU sensor data in real-time via a serial CLI. It can be used as a base for motion tracking, robotics, or sensor fusion systems.

-> CLI Commands

  > help : list all available commands

  > gyro : display real-time gyroscope data

  > acc : display real-time accelerometer data
  
  > last_gyro : retrieve the last 5 gyroscope readings
  
  > last_acc : retrieve the last 5 accelerometer readings

-> Offset Calibration

- During system startup, the device reads 100 samples from each axis and computes the average.
- This average is subtracted from subsequent raw values to reduce drift and noise.

## Software Architecture

- The CLI is implemented using UART interrupt handlers. Incoming characters are stored in a buffer.
- When a newline character is detected, the buffer is parsed to determine the command.
- The system maintains two circular buffers for storing the last 5 accelerometer and gyroscope readings.
- I2C communication is handled using STM32 HAL drivers.
- Data is displayed over UART using `sprintf` and `HAL_UART_Transmit`.

## Components Used

STM32F0DISC

MPU6050 IMU Sensor

UART Module

Jumper Cables

Figure 1 : System Overview 

<img src="https://github.com/ssenanb/CLI-With-MPU6050-IMU/blob/main/system_overview.jpeg" alt="System Overview" width="500"/>

<img src="https://github.com/ssenanb/CLI-With-MPU6050-IMU/blob/main/sensor_image.jpeg" alt="System Overview" width="500"/>

Figure 2 : Output in the Termite

<img src="https://github.com/ssenanb/CLI-With-MPU6050-IMU/blob/main/termite.png" alt="CLI" width="500"/>

## Configuration

Figure 3 : Configuration in the STM32CubeIDE

<img src="https://github.com/ssenanb/CLI-With-MPU6050-IMU/blob/main/configuration.png" alt="Configuration" width="500"/>

PA9 -> USART1_TX (with Interrupt)

PA10 -> USART1_RX (with Interrupt)

PB6 -> I2C1_SCL (to the IMU SCL)

PB7 -> I2C1_SDA (to the IMU SDA)

