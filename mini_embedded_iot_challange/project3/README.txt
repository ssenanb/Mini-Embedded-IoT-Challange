# LDR-Based-Light-Follower

## Project Description

This project uses 2 LDRs to detect ambient light and rotates the servo motor to the opposite side of the LDR where the light intensity is low. Thus, the servo moves to the area with higher light. Also, the servo position (turned right or left) is printed on the LCD screen.

## Components Used

STM32F0DISC

2 x LDRs

2 x Resistsnce (10k)

I2C LCD Screen 

SG-90 Servo Motor

1.5*4 - 6V Alkalin Battery

Jumper Cables

## Circuit Installation 

Figure 1 : 90°

<img src="https://github.com/ssenanb/LDR-Based-Light-Follower/blob/main/circuit-installation-90-angle.jpeg?raw=true" alt="Devre Kurulumu - 90 Derece Görseli" width="500"/>

Figure 2 : 0°

<img src="https://github.com/ssenanb/LDR-Based-Light-Follower/blob/main/circuit-installation-0-angle.jpeg?raw=true" alt="Devre Kurulumu - Açı Görseli" width="500"/>

Figure 3 : 180°

<img src="https://github.com/ssenanb/LDR-Based-Light-Follower/blob/main/circuit-installation-180-angle.jpeg?raw=true" alt="Devre Kurulumu - 180 Derece Görseli" width="500"/>

## Pin Configuration 

Figure 4 : Pin Configuration in the STM32CubeIDE

<img src="https://github.com/ssenanb/LDR-Based-Light-Follower/blob/main/pin-configuration.png?raw=true" alt="Pin Konfigürasyonu" width="500"/>

PA0 & PA1 -> LDR (ADC_IN0 & ADC_IN1)


PA5 -> Servo Motor - Signal Pin -> (TIM2_CH1)

Servo Motor -> VCC -> Battery VCC

Servo Motor -> GND -> Board & Battery GND


PB6 -> I2C LCD -> SCL

PB7 -> I2C LCD -> SDA

I2C LCD -> VCC -> Arduino Uno 5V

I2C LCD -> GND -> Arduino Uno GND & Board


STM32F0DISC -> 5V -> Board

STM32F0DISC -> GND -> Board

 
CONCLUSION: I used Arduino Uno due to I2C LCD did not enough to power from STM32F0DISC when it's VCC attaching the cable to STM32F0DISC and I connected a 5V adaptor to power to Arduino UNO.
Also, i do when the first time, servo motor did not enough to power from STM32F0DISC. Thus, i connected a 6V battery to motor. In result of i learned i should have connected a battery for servo motor and used external 5V adaptor for run I2C LCD properly.

-> Software

STM32CubeIDE

I2C LCD Library --> I used this library : https://github.com/alixahedi/i2c-lcd-stm32

C Programming Language

ST-Link Debugger

-> Working Principle

In this project, two LDRs  are used to detect ambient light intensity. These sensors change their resistance based on the amount of light falling on them. The resistance values are read through the microcontroller’s analog input pins and converted into digital values via ADC.

By comparing the values from both LDRs, the system determines which side receives less light. If the left LDR detects less light (i.e., it's darker), the servo motor rotates towards the right to face the light source, and vice versa. The servo motor is controlled using a PWM signal.

After the servo motor adjusts its position, the direction of movement is displayed on a 16x2 LCD using the I2C protocol. For example, if the servo turns right, the LCD will show a message like “Turned right.” This provides clear visual feedback about the system's behavior.