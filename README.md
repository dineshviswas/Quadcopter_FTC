# Fault Tolerant Control of Quadcopter 

![image](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)  ![image](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=Raspberry%20Pi&logoColor=white)  ![image](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)  

## Description
This project implements various control algorithms to check their fault tolerance(for one actuator fault/failure). 
The code is written in C using the Arduino framework and freeRTOS. 
You can clone this repository and upload the firmware to your microcontroller to build your own quadcopter.

## Hardware

1. **Microcontroller:** Raspberry Pi Pico 2 W ([Datasheet](https://datasheets.raspberrypi.com/picow/pico-2-w-pinout.pdf))
2. **IMU:**             Adafruit BNO085       ([Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-9-dof-orientation-imu-fusion-breakout-bno085.pdf))
3. **ESC:**             Tekko32 4 in 1        ([Datasheet](https://robu.in/wp-content/uploads/2023/12/1777637.pdf))
4. **Power:**           2xSamsung INR21700-50S 3.6V 5000mAh 9C Li-ion Battery
5. **Motors:**          4xEMAX ECO Micro 1106 2-3S 4500KV CW Brushless Motor

## How to use this

1. Connect all the components(refer datasheets for pinouts)
2. Upload the [Tare firmware](taring.ino)
3. Follow the calibration steps and place the quadcopter on a table for a few seconds for the tare process
4. Check the IMU readings(firmware from Adafruit_BNO08x examples) for confirmation
5. Upload the [Tuning firmware](PID_TUNING.ino)
