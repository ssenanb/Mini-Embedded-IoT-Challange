# Audio Warning System with Bluetooth Controlled Pseudo-Radar

# Project Description

This project involves environmental detection through angular scanning using a HC-SR04 on a servo motor.
The system is based on a simple radar simulation. The easured distance values are:

* Transmitted from the ESP32 to the ESP32 via UART.

* Then forwarded from the ESP32 to a smartphone via Bluetooth.

* Displayed in real time on a Bluetooth Serial Terminal running on Android.

If the measured distance falls a below certain threshold, the buzzer on the STM32 is activated. The system also supports remote control. When an `OFF` command is sent from the Bluetooth Terminal, the ESP32 forwards this command to the STM32 via UART and the buzzer is deactivated. 

NOTE : I used a median filter for the HC-SR04 sensor to reduce measurement noise. To sort the sensor values, I implemented the selection sort algorithm.

Figure 1 : System Ovreview

<img src="https://github.com/ssenanb/Danger-Detection-and-Audio-Warning-with-Bluetooth-Based-Radar-System/blob/main/system_overview.jpeg" alt="System Overview" width="500"/>

Figure 2 : Servo + HC-SR04 

<img src="https://github.com/ssenanb/Danger-Detection-and-Audio-Warning-with-Bluetooth-Based-Radar-System/blob/main/servo_sensor.jpeg" alt="System Sensor" width="500"/>

Figure 3 : Bluetooth Terminal

<img src="https://github.com/ssenanb/Danger-Detection-and-Audio-Warning-with-Bluetooth-Based-Radar-System/blob/main/bluetooth_terminal.jpeg" alt="Terminal" width="500"/>

# Components

STM32F0DISC

ESP32 WROOM 32D

HC-SR04 Ultrasonic Distance Sensor

SG-90 Mini Servo Motor

Active Buzzer

4 * 1.5V Battaries

Jumper Cables

# Pin Configuration

Figure 4 : Pin configuration

<img src="https://github.com/ssenanb/Danger-Detection-and-Audio-Warning-with-Bluetooth-Based-Radar-System/blob/main/configuration.png" alt="Configuration" width="500"/>

PA1 -> TIM2_CH2 -> Servo Motor

PA8 -> TIM1_CH1 ->  HC-SR04 - Echo Pin (with Input Capture PWM Mode and Interrupt)

PA9 -> USART1_TX (with Interrupt)

PA10 -> USART1_RX (with Interrupt)

PC8 -> GPIO_Output -> Active Buzzer

PC9 -> GPIO_Output -> HC-SR04 - Trig Pin

All the GNDs are connected.