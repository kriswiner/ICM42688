# ICM42688
Collection of Arduino sketches for TDK's combo accel/gyro motion sensor

The **basic sketch** configures the sensors' data rates and full scale sensitivities as well as low-pass data filters. It performs the sensor self tests, calibrates the sensors, sets up the data ready interrupts. The main loop is interrupt-based and best absolute orientation estimation results are obtained running the gyro and accel at 200 Hz or more. Sensor fusion using the open-source Madgwick fusion algorithm is iterated every time new gyro data is available; magnetometer data is read upon interrupt at the 100 Hz maximum rate so that most of the fusion results use "stale" mag data. This shouldn't affect accuracy though, and with proper calibration this sensor combination is capable of delivering better than 1 degree rms heading accuracy. In fact, even with the somewhat crude calibration employed here, I have noticed the jitter for the 9 DoF sensors and Euler angles is very low.

The **APEX** sketch uses the ICM42688 accel embedded function (APEX) engine to demonstrate two of the embedded functions, tilt detection, which uses the DMP, and wake-on-motion, which does not.  The sketch shows how to set up the wake-on-motion detect and DMP for tilt detection, route the APEX interrupts to INT2, and use the status registers to detect which APEX function is causing the interrupt. The APEX detection is run in parallel with the usual accel/gyro/barometer data output, but I have turned the magnetometer off here. These embedded functions are similar to those found in other accelerometers. They are not really the focus of my interest in this sensor but it is useful to be able to get them to work nonetheless. These and others APEX functions are explained in detail in TDK's AN-000173 "ICM-426xx APEX Motion Functions: Description and Usage" available on TDK's ICM42688 product page.

The **6DoF** sketch puts the magnetometer in powerdown mode and uses a 6 DoF (accel and gyro only) version of Madgwick's open-source fusion algorithm to calculate relative yaw, pitch, and roll. The ICM42688 allows the 10,000-ppm-accurate embedded PLL to be replaced by a 30 - 50 kHz external clock signal via the INT2 pin. The STM32L4 is capable of outputting a 32.768 kHz square wave from its 1-ppm-accuracy RTC embedded clock. The results of initial testing look promising.

![CLKIN](https://user-images.githubusercontent.com/6698410/151062460-4b1d920e-f8f1-49fb-a878-d431b87e62c8.jpg)

The use of the external STM32L432 32.768 kHz clock source reduces the gyro drift by more than 4x. This is a significant improvement in gyro stability (the major determinant of relative 6 DoF yaw drift) which could lead to further improvent in the more interesting 9 DoF absolute orientation estimation. More on this to come...

Overall, the ICM42688 is a worthy high-performance accel/gyro combo sensor that is a candidate to replace the LSM6DSM/T, its nearest competitor, in any absolute orientation estimation engine requiring the best accuracy.

![image](https://user-images.githubusercontent.com/6698410/149679962-022c7f2d-b55f-4f43-938d-be3684454ed9.jpg)

The design for the breakout board I designed for testing the ICM42688 can be found in the OSH Park shared space [here](https://oshpark.com/shared_projects/EZpXB0Te).

I am using the STM32L432KC ([Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/)) breakout board for testing the ICM42688. Ladybug available on Tindie.

Copyright 2022 Tlera Corporation. All rights reserved.
