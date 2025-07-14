# RTC-Based-Alarm-System

## Project Description

This project is an alarm system and a digital clock. In the default mode, the current time is shown on the LCD screen. When the USER button is pressed, the system switches to alarm-setting mode.Upon entering this mode, a message appears on the screen indicating that the alarm mode is active. You can set the alarm hour and minute using with buttons. After setting the alarm, the system returns to the default clock display mode. When the current time matches the set alarm time, the buzzer is activated.

## How It Works

- The system operates in two main modes: Clock Mode and Alarm Setting Mode.
- On boot, the DS3231 RTC module is initialized via I2C and current time is displayed.
- PA0 (USER Button) triggers an EXTI interrupt to switch to alarm mode.
- In alarm mode, PA8 and PA9 are used to increment hours and minutes.
- PA10 exits alarm setting mode.
- The buzzer on PB8 is triggered when the current time matches the set time.

## Components Used

STM32FODISC

DS3231 RTC Module

4 * Buttons (one of them is the USER button on the microcontroller) 

3 * Resistances (10k) 

Passive Buzzer

16x2 I2C LCD Screen

Jumper cables

Figure 1 : System Overview

<img src="https://github.com/ssenanb/RTC-Based-Alarm-System/blob/main/SystemOverwiev.jpeg?raw=true" alt="System Overwiev" width="500"/>

Figure 2 : One of the LCD Screens 

<img src="https://github.com/ssenanb/RTC-Based-Alarm-System/blob/main/lcd_setHour.jpeg?raw=true" alt="Set Hour" width="500"/>

Figure 3 : Default Mode

<img src="https://github.com/ssenanb/RTC-Based-Alarm-System/blob/main/defaultMode.jpeg?raw=true" alt="Default Mode" width="500"/>

There is a video demonstrating the alarm mode -> https://vimeo.com/1093398200/233cd4c41f?share=copy

## Pin Configuration 

Figue 4 : Pin Configuration In The STM32CubeIDE

<img src="https://github.com/ssenanb/RTC-Based-Alarm-System/blob/main/configuration.png?raw=true" alt="Configuration" width="500"/>


PA0 (USER Button) -> GPIO_EXTI0 -> Alarm Mode Starting

PA8 -> GPIO_INPUT -> Set Hour

PA9 -> GPIO_INPUT -> Set Minute

PA10 -> GPIO_INPUT -> Alarm Mode Closing

PB6 -> I2C1_SCL -> RTC Module SCL

PB7 -> I2C1_SDA -> RTC Module SDA

PB8 -> GPIO_OUTPUT -> Buzzer

PB10 -> I2C2_SCL -> I2C LCD Screen SCL

PB11 -> I2C2_SDA -> I2C LCD Screen SDA

STM32F0DISC -> 5V -> Board

STM32F0DISC -> GND -> Board

I used this library for the I2C LCD -> https://github.com/alixahedi/i2c-lcd-stm32





