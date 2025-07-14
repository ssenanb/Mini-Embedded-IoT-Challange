# Distance-Logger-with-Web-Interface

# Description

In this project, a real time sensor data monitoring system is designed using STM32 and ESP32 microcontrollers. The STM32 reads data from the HC-SR04 distance sensor data and the DS3231 RTC module. This data is formatted in JSON and sent to the ESP32 via UART. The ESP32 parses the received data and displays it through a local web interface over Wİ-Fi. If the measured distance data is above a threshold, it is displayed on the web interface in real time. 

STM32 : It reads distance and time data, transforms it into the JSON format and sends it via UART.

ESP32 : It receive a the UART data from the STM32, parses it, sets up a web server and displays the data.

Web Server : A simple page created using HTML and JavaScript, which displays data by updating it every second.

Trigger : Only data above the threshold is displayed, prevent unnecessary data transmission.

This project provides an effective example of integrating embedded system with IoT technologies. It demonstrates how meaningful and filtered data can be wirelessly visualized using low power devices.

# Components Used

STM32F0DISC

HC-SR04 Distance Sensor

DS3231 RTC Module

ESP32 WROOM 32D

Jumper Cables

Figure 1 : System Overview

<img src="https://github.com/ssenanb/Distance-Logger-with-Web-Interface/blob/main/System_Overview.jpeg" alt="System Overview" width="400"/>

Figure 2 : Serial Monitor Output

<img src="https://github.com/ssenanb/Distance-Logger-with-Web-Interface/blob/main/Serial_Monitor.png" alt="Output" width="400"/>

Figure 3 : Web Interface

<img src="https://github.com/ssenanb/Distance-Logger-with-Web-Interface/blob/main/server.png" alt="Server" width="400"/>

# Pin Configuration 

Figure 4 : Pin Configuration in the STM32CubeIDE

<img src="https://github.com/ssenanb/Distance-Logger-with-Web-Interface/blob/main/configuration.png" alt="config" width="400"/>

PA1 -> TIM2_CH2 -> HC-SR04 - Echo Pin (with Input Capture PWM Mode and Interrupt)

PA2 -> GPIO_Output -> HC-SR04 - Trig Pin

PA9 -> USART1_TX

PA10 -> USART1_RX

PB6 -> I2C1_SCL -> DS3231 RTC SCL

PB7 -> I2C1_SDA -> DS3231 RTC SDA

STM32F0DISC -> 5V -> Board

STM32F0DISC -> GND -> Board

All the GNDs are connected.
