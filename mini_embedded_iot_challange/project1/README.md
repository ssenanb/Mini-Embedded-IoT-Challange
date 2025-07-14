# UART-Supported-Environment-Control
## Project Description

This project monitors the environment by collecting data from temperature and motion sensors. Based on the collected data, it controls a fan (DC motor with a propeller) and the brightness of an LED. The system supports both manual and automatic operation modes. It can be manually controlled via UART and displays its current status on an LCD screen.

## Software Architecture

- The system is structured as two modes: `AUTO` and `MANUAL`.
- UART commands are parsed using interrupt-based reception.
- PWM signal is generated for motor speed control using Timer 2 Channel 2.
- LCD updates are handled periodically via polling.

## Components Used

STM32FODISC

IR Distance Sensor

DHT11 Temperature and Humidity Sensor

LED

Resistance(330)

3-6V DC Motor

L298N Motor Driver

4 * 1.5V Batteries

UART Module

16x2 I2C LCD 

Arduino Uno (used only as power source for LCD)

Jumper Cables

## Features

* Automatic Mode : If the temperature is higher than the threshold, the fan turns on and if the distance is smaller than the threshold, the LED turns on. All of the status display in the LCD screen.
  
* Manual Mode : If you send the `MANUAL` command via UART, the system switches to manual mode. All of the command in the below :

  `AUTO` -> The system switches to automatic mode again. (Auto Mode' is printed to the Termite terminal via UART.)
  
  `MANUAL` -> The system switches to manual mode. (Manual Mode' is printed to the Termite terminal via UART)

  `FAN ON` -> The fan turns on.

  `FAN OFF` -> The fan turns off.

  `LED ON` -> The LED turns on.

  `LED OFF` -> The LED turns off.

All of the stages display in the LCD screen.

Figure 1 : System Overview

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/system_installation.jpeg" alt="System Overview" width="500"/>

Figure 2 : Automatic Mode

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/auto_mode.jpeg" alt="Auto Mode" width="500"/>

Figure 3 : Manual Mode

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/manuel_mode.jpeg" alt="Manuel Mode" width="500"/>

Fİgure 4 : UART Output (Termite)

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/termite_display.png" alt="Termite" width="500"/>

## Pin Configuration 

<img src="https://github.com/ssenanb/UART-Supported-Environment-Control/blob/main/configuration.png" alt="Termite" width="500"/>


PA0 -> ADC_IN0 -> IR Distance Sensor

PA1 -> GPIO_Output -> DHT11 Sensor

PA9 -> USART1_TX (with Interrupt)

PA10 -> USART1_RX (with Interrupt)

PB3 -> TIM2_CH2 -> DC Motor ENA

PB4 -> GPIO_Output -> DC Motor IN1

PB5 -> GPIO_Output -> DC Motor IN2

DC Motor VCC -> Battery VCC

DC Motor GND -> Battery GND & Board

PB6 -> I2C_SCL -> LCD

PB7 -> I2C_SDA -> LCD

PD2 -> GPIO_OUTPUT -> LED

I2C LCD VCC -> Arduino UNO 5V

I2C LCD -> GND -> Arduino Uno GND & Board

STM32F0DISC -> 5V -> Board

STM32F0DISC -> GND -> Board

All the GNDs are connected.

NOTE : The STM32F0DISCOVERY board could not supply sufficient current to the I2C LCD.
Therefore, I powered the LCD using Arduino Uno’s 5V pin and connected common GND between all devices.

I used these library : 

For the LCD :  https://github.com/alixahedi/i2c-lcd-stm32

For the DHT11 : https://github.com/quen0n/DHT11-DHT22-STM32-HAL

