# ICM42688
Collection of Arduino sketches for TDK's combo accel/gyro motion sensor

The **basic sketch** configures the sensors' data rates and full scale sensitivities as well as low-pass data filters. It performs the sensor self tests, calibrates the sensors, sets up the data ready interrupts. The main loop is interrupt-based and best absolute orientation estimation results are obtained running the gyro and accel at 200 Hz or more. Sensor fusion using the open-source Madgwick fusion algorith is iterated every time new gyro data is available; magnetometer data is read upon interrupt at the 100 Hz maximum rate so that most of the fusion results use "stale" mag data. This shouldn't affect accuracy though, and with proper calibration this sensor combination is capable of delivering better than 1 degree rms heading accuracy. In fact, even with the somewhat crude calibration employed here, I have noticed the jitter for the 9 DoF sensors and Euler angles is very low.

The ICM42688 has an embedded function engine which will be the subject of the next sketch. I plan to demonstrate wake-on-motion, tap detection, as well as use of the FIFO to capture data prior to a wake event for later analysis. I might try some of the other embedded functions just for fun.

Lastly, the ICM42688 allows the 50 ppm accurate embedded PLL to be replaced by a 30 - 50 kHz clock signal. The STM32L4 is capable of outputing a 32.768 kHz square wave from the 1-ppm-accuracy RTC embedded clock. I am curious to see if any difference between using the internall ICM42688 PLL and a more accurate external CLKIN signal can be detected in gyro drift and/or heading accuracy. I will post the sketch I use and any results when obtained.

Overall, the ICM42688 is a worthy high-performance accel/gyro combo sensor that is a candidate to replace the LSM6DSM/T, its nearest competitor, in any absolute orientation estimation engine requiring the best accuracy.

![image](https://user-images.githubusercontent.com/6698410/149679962-022c7f2d-b55f-4f43-938d-be3684454ed9.jpg)

The design for the breakout board I designed for testing the ICM42688 can be found in the OSH Park shared space [here](https://oshpark.com/shared_projects/EZpXB0Te).

I am using the STM32L432KC ([Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/)) breakout board for testing the ICM42688. Ladybug available on Tindie.

Copyright 2022 Tlera Corporation. All rights reserved.
