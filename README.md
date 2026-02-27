# Sea-State Stabilizer (Gimbal Roll-Pitch)
Work In Progress - Assembling hardware and components while fighting power distribution issues


## Project overview
I wanted to design and build a 2-Degree-of-Freedom stabilization platform as a first project to finally put to use the bunch of servos, PCBs, cables and the ESP32 I had bought months ago with the idea of building myself a little robot. Nowadays, I want to focus more on marine robotics as ROVs / AUVs / USVs are a great interest of mine. I figured this idea could be a fine, simple enough way to break free of analysis paralysis and take a first step into marine technology, control theory, electronics and other fundamentals of robotics. 

Its primary goal is to maintain a top-mounted sensor payload (HC-SR04 Ultrasonic Sensor and MPU6050) perfectly horizontal by actively compensating for external pitch and roll disturbances, simulating the motion of a boat on rough seas. Building this system from scratch serves as a practical dive into:
* **Kinematics and CAD:** Forward and Inverse kinematics validation using Onshape, designing a mechanical architecture that isolates the electronics from the moving payload.
* **Hardware and power management:** Managing high-current spikes from servo motors using an external power distribution system (PCA9685 and a dedicated 5V/3A PSU) to prevent microcontroller brownouts.
* **Control theory:** Implementing a PID (Proportional-Integral-Derivative) controller and a Complementary/Kalman filter to process raw accelerometer and gyroscope data from the MPU6050 into smooth, actionable servo movements.


---


## Dev log

### First steps
With my Software Engineering and Database courses out of the way and with the end of the exam session, I wanted to try my hand at hardware - have the silicon talk. I jumped straight into wiring the ESP32 to the PCA9685 PWM driver; a 38-pin ESP32 is an absolute unit, takes up almost the entire width of a standard breadboard, and leaves no room to plug jumper wires on the sides. So I had to less-elegantly demote the breadboard to just an oversized power strip at the center of the system.

Here is the initial wiring scheme:
* **ESP32 `3V3`** ➔ **PCA9685 `VCC`** *(Logic power)*
* **ESP32 `GND`** ➔ **PCA9685 `GND`**
* **ESP32 `GPIO 21`** ➔ **PCA9685 `SDA`** *(I2C Data)*
* **ESP32 `GPIO 22`** ➔ **PCA9685 `SCL`** *(I2C Clock)*
* **PCA9685 `Channels 0 and 1`** ➔ **Servos `PWM, V+, GND`**
* **USB Adapter screw terminals** ➔ **PCA9685 green terminal block** *(External power)*

My first taste of engineering obstacles came right away with power delivery. I had my very first attempt at stripping wires to connect a USB adapter's screw terminals to the PCA's ones. Turns out, thin stripped wires and screw terminals don't mix well, current dropped, and the servos refused to work properly. Originally, I had bought this USB adapter intending to use it in conjunction with a USB extension cable I already had, but the latter turned out to be faulty and almost fried all my components, a total liability. That meant a handful of tests just to figure out what survived and basically a full restart, all while having to kneel and stay close to the wall for my power supply without cables (finding better solutions to this = top priority).

Meanwhile, the MPU6050 arrived with its header pins unsoldered. This meant I got to have my first attempt at soldering! I enjoyed the process and the fumes, but the result has been horrid - nasty blobs and bubbles instead of the nice shiny cones the YouTube tutorials promised. I'll blame it 50% on my skills and 50% on the 12€ Amazon soldering kit.

Despite the hardware gore, running a simple I2C scanner script and finally seeing the `0x40` address pop up on the Serial Monitor meant I got to taste some practical IoT, and I got my hands on a baseline PID control draft to eventually map the MPU's tilt angles to the servos.

*[Placeholder for Draw.io schematic of the wiring and photos]*

### CAD Simulation
To avoid frying more components and myself, I decided to step back and digitally model my intended design first. This gave me the opportunity to learn the basics of CAD! Though I must say web-based, German-only (my fault for not looking for the language settings hard enough) Onshape has been awful to me. The goal was to ensure the Roll-Pitch gimbal configuration could physically achieve the required angles without self-collision, prove the stabilization logic via Inverse Kinematics, and most importantly have a nice simulation to glare at while solving IRL complications.

**1. Forward Kinematics:** Testing the operational limits of the physical joints (restricted to ±45° to simulate realistic wave compensation).  
*[Placeholder for Pan/Tilt GIF]* ![Forward Kinematics](link_gif_1.gif)

**2. Inverse Kinematics:** By anchoring the sensor platform and freeing the base, this simulation demonstrates the core objective: the base (boat) moves chaotically, while the mechanical arms dynamically adapt to keep the payload perfectly horizontal.  
*[Placeholder for POV Stabilization GIF here]* ![Inverse Kinematics](link_gif_2.gif)

### Prototyping, WIP
So I moved from the desk, to the cloud, and now back to the desk. Currently assembling and wiring the hardware on my custom cardboard chassis. 

* **Current Challenge:** Finally solving those power distribution issues. Dupont wires bottleneck the 3A current spikes required by the SG90 servos, causing the ESP32 to brownout. I'll implement thicker gauge copper wiring first, and check out what other solutions Amazon and AliExpress have to offer second.
* **Next Step:** Securing the MPU6050 exactly at the geometric center of the top platform to minimize centrifugal acceleration readings during roll/pitch movements.

*[Placeholder for photos]*

### Control logic and firmware, WIP
The final step to completion will involve fine-tuning the firmware in C++/Arduino IDE. 
* Reading I2C data from the MPU6050 smoothly.
* Tuning the PID loop to eliminate jitter and overshoot.
* Using the HC-SR04 to prove that distance measurements to a fixed overhead target remain constant despite the base moving under it.

---

## Bill of materials (BOM)
* **Microcontroller:** ESP32 (38-pin)
* **IMU Sensor:** MPU6050 (6-axis Accelerometer & Gyroscope)
* **Distance Sensor:** HC-SR04 Ultrasonic Module
* **Actuators:** 2x SG90 Micro Servos (180°)
* **PWM Driver:** PCA9685 (16-Channel 12-bit)
* **Power:** 5V 3A DC Power Supply (isolated from ESP32 logic)
* **Chassis:** Custom cardboard modeled structure
