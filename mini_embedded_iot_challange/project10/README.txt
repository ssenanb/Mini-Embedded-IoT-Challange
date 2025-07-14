# Embedded-IoT-Data-Acquisition-Transmission-System-with-RTOS

# Project Description
This project is based on a multitasking data collection and transmission system using an STM32 microcontroller, FreeRTOS (CMSIS RTOS v2 API), an ESP32 module and the ThingSpeak cloud platform. The system runs on a real-time operating system (RTOS) infrastructure and practically demonstrates key embedded systems concepts such as task separation, synchronization via semaphores, message queues and communication.

Data from IR and LDR sensors is collected by the ADC task. However, data acquisition is not continuous by default — it is triggered by an external button press, which generates an interrupt. The interrupt service routine (ISR) releases a semaphore that activates the ADC task. Once activated, the ADC task samples both sensors and sends the data to the UART task via a message queue.

The UART task formats the sensor values in JSON and transmits them to the ESP32 module. To ensure thread-safe access to the UART peripheral, a mutex is used during the transmission process. This prevents race conditions between tasks and guarantees reliable data communication.

Figure 1 : System Overview

<img src="https://github.com/ssenanb/Embedded-IoT-Data-Acquisition-Transmission-System-with-RTOS/blob/main/system_ovreview.jpeg" alt="System Overview" width="500"/>

Figure 2 : ThingSpeak Cloud

<img src="https://github.com/ssenanb/Embedded-IoT-Data-Acquisition-Transmission-System-with-RTOS/blob/main/dashboard.png" alt="Cloud" width="500"/>

# Components

STM32F0DISC

ESP32 WROOM 32D

IR Distance Sensor

LDR

LED

Button

Resistances

Jumper Cables

# Pin Configuration

Figure 3 : Pin Configuration in the STM32CubeIDE

<img src="https://github.com/ssenanb/Embedded-IoT-Data-Acquisition-Transmission-System-with-RTOS/blob/main/configuration.png" alt="Configuration" width="500"/>

PA0 -> GPIO_Output -> LED

PA1 -> GPIO_EXTI1 -> Button

PA2 ->  ADC_IN2 -> IR Distance Sensor

PA3 ->  ADC_IN3 -> LDR

PA9 -> USART1_TX 

PA10 -> USART1_RX S