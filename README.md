<img width="2635" height="1061" alt="accelerus" src="https://github.com/user-attachments/assets/369f8082-f502-448c-a9e0-eb035074f208" />



This rocket control system is built around a central flight computer, the **STMicroelectronics NUCLEO‑H753ZI**, which contains an STM32H7 microcontroller. This microcontroller runs all flight logic, sensor processing, control algorithms, and safety checks. It is programmed in C or C++ using standard embedded development tools. The microcontroller runs continuously from power‑on until after landing, without relying on external commands for critical decisions.

The flight computer is connected to multiple sensors that measure the physical state of the rocket. The **BMI088 accelerometer and gyroscope** is the primary motion sensor. It measures angular velocity and linear acceleration at a high rate. This data is used to determine how fast the rocket is rotating and whether it is accelerating upward, coasting, or falling. The **LIS3MDL magnetometer** provides a reference for Earth’s magnetic field and is mainly used during non‑powered phases to help with orientation correction. The **BMP388 pressure sensor** and the **MS5611‑01BA03 precision altimeter** both measure air pressure, which is converted into altitude. The MS5611 is treated as the primary altitude reference due to its higher precision, while the BMP388 can be used as a secondary reference or for redundancy. The **TMP102 temperature sensor** measures board or ambient temperature so that temperature effects on sensors and battery performance can be monitored and compensated in software.


<img width="541" height="794" alt="image" src="https://github.com/user-attachments/assets/5787e828-0f3b-494e-95c8-f3af04a894e5" />

All sensor data is read by the microcontroller through SPI or I²C buses at fixed intervals controlled by hardware timers. The raw sensor values are not used directly. Instead, the microcontroller runs estimation algorithms on them. A complementary filter is used to estimate the rocket’s orientation by combining fast gyroscope data from the BMI088 with slower accelerometer data. For vertical motion, a one‑dimensional Kalman filter runs on the microcontroller to estimate altitude and vertical velocity using acceleration data and pressure‑based altitude measurements from the MS5611. These filters run continuously during flight and update the estimated state of the rocket many times per second.

During powered ascent, the rocket uses movable fins for stabilization. Each fin is actuated using an **ST3215 servo with an encoder**, which allows precise control and position feedback. The microcontroller calculates stabilization commands using PID control loops that operate on angular rate data from the BMI088. The goal of these PID controllers is to reduce unwanted rotation and keep the rocket stable. Control signals are sent to the fin servos through a **MAX485 serial interface**, which allows reliable communication and noise resistance in a high‑vibration environment. If sensor data becomes invalid or unsafe conditions are detected, the software can command the fins to return to a neutral position.
![the rock](https://github.com/user-attachments/assets/f033e439-80f2-451c-bd1b-202f64dfda9f)

After the rocket reaches apogee, which is detected using altitude and vertical velocity estimates, the system transitions into descent mode. At this point, parachute deployment logic is executed autonomously. Parachute control is handled using **N20 coreless DC gearmotors with encoders**, driven by a **DRV8833 motor driver**. These motors slowly adjust the lengths of parachute control lines. The encoders provide feedback so the microcontroller knows how much each line has been pulled or released. This allows controlled steering during descent rather than a purely passive fall.

For navigation during descent, the system uses a **u‑blox NEO‑M8N GPS module**. The GPS provides position updates, which are compared to a predefined landing target stored in memory. The microcontroller computes simple heading and position errors and adjusts the parachute control lines accordingly. This descent control is intentionally slow and conservative to avoid instability or excessive mechanical stress.

<img width="1833" height="481" alt="image" src="https://github.com/user-attachments/assets/f42f7591-47b7-4474-9295-839cc85be4d0" />

All flight data is recorded to **SPI flash memory**, including sensor readings, estimated states, control outputs, and flight events. This data logging runs continuously and acts as a flight recorder for post‑flight analysis. Telemetry data is also transmitted during flight using a **LoRa SX1276 radio module**, connected to a **433 MHz antenna**, allowing basic real‑time monitoring from the ground. Telemetry is never used for control decisions and does not affect safety‑critical actions.

Power for the entire system comes from a **7.4 V Li‑ion battery**, specifically a **2S protected 18650 pack**. Voltage regulation is handled using multiple regulators. A **LM2596 buck converter** and a **REES52 RC BEC UBEC 5V 5A** provide stable voltage rails for the microcontroller, sensors, servos, motors, and radio. The battery capacity is far greater than required for the short duration of flight, ensuring stable operation even during current spikes from servos or motors.

<img width="523" height="328" alt="image" src="https://github.com/user-attachments/assets/f22c047b-5823-4dd6-a349-3ab7d06adc8d" />

Safety and reliability are handled entirely onboard. The microcontroller uses watchdog timers, time‑based backups, and sensor consistency checks to ensure recovery actions occur even if part of the system fails. Parachute deployment does not depend on radio commands and will occur automatically based on onboard logic. The system is designed to operate autonomously from launch to landing, with all critical decisions made locally by the flight computer.

This overall design represents a complete, theoretically flight‑capable rocket control system where sensing, computation, actuation, power, logging, telemetry, and safety are all handled in an integrated and deterministic manner.
