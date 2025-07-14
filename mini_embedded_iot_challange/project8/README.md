# STM32-ESP32-MQTT-Controlled-Trigger-and-Alarm-System

# Description

In this project, environmental triggers are detected using sound and motion sensors on the STM32. When a trigger is detected, the system sends a "Motion certainly detected!" message to the ESP32 via UART. The ESP32 transmits this message to the Adafruit IO Dashboard using the MQTT protocol. The buzzer is simultaneously activated and produces a loud warning sound. A button on the dashboard allows the user to remotely turn off the buzzer. If the user does not turn off the buzzer by pressing the button within 10 seconds, the system automatically deactivates the buzzer.

Figure 1 : System Overview

<img src="https://github.com/ssenanb/STM32-ESP32-MQTT-Controlled-Trigger-and-Alarm-System/blob/main/System_Overview.jpeg" alt="System Overview" width="500"/>

Figure 2 : Adafruit IO Dashboard Output

<img src="https://github.com/ssenanb/STM32-ESP32-MQTT-Controlled-Trigger-and-Alarm-System/blob/main/dashboard.png" alt="Dashboard" width="500"/>

# Software Architecture

This system is built on an event-driven software architecture. Two embedded devices (STM32 and ESP32) are connected at the hardware level via UART and their tasks are divided as follows:

-> STM32 Tasks

Sensor Detection: It detects triggers using the PIR and sound sensors via hardware interrupts.

Buuzer Control: When a trigger is detected, the buzzer is activated. It automatically turns off after 10 seconds.

UART Communication: It sends a trigger message to the ESP32. It also reads incoming commands and turns off the buzzer if commanded.

Time Management: It tracks time using `HAL_GetTick()` and checks how long the buzzer has been activate.

-> ESP32 Tasks

UART Interface: It receives the message from the STM32 via the serial port.

MQTT Communication: It transmits the message to the Adafruit IO Dashboard using the MQTT protocol.

Command Transmission: It receives the `OFF` message from the dashboard and sends it to the STM32.

# Components Used

STM32F0DISC

ESP32 WROOM 32D

Sound Sensor

PIR Sensor

Active Buzzer

Jumper Cables

# Pin Configuration

Figure 3 : Pin Configuration on the STM32CubeIDE

<img src="https://github.com/ssenanb/STM32-ESP32-MQTT-Controlled-Trigger-and-Alarm-System/blob/main/configuration.png" alt="System Configuration" width="500"/>

PA0 -> GPIO_EXTI0 -> Sound Sensor

PA1 -> GPIO_EXTI1 -> PIR Sensor

PA9 -> USART1_TX (with Interrupt)

PA10 -> USART1_RX (with Interrupt)

PC9 -> GPIO_Output -> Active Buzzer
