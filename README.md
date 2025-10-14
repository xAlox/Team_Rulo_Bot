# Team Rulo Bot – WRO Future Engineers 2025
---

## Repository content

<kbd>3D_models</kbd> Includes the complete 3D design of the vehicle and rendered models for presentation.  

<kbd>schemes</kbd> Contains electrical and electronic schematic diagrams (PNG) showing system interconnections.  

<kbd>src</kbd> Source code for both the Raspberry Pi (Python) and Arduino (C++) programs, including PD steering, obstacle avoidance, and vision algorithms.  

<kbd>tphotos</kbd> Team Rulo Bot photos — competition sessions, testing, and behind-the-scenes moments.  

<kbd>tvideos</kbd> YouTube links or embedded demos showing the robot’s performance in each mission.  

<kbd>robot_photos</kbd> Robot gallery including top, side, and bottom perspectives of the vehicle.  

<kbd>readme</kbd> This Readme contain: 




- [Repository Contents](#repository-contents)
- [Team](#Team)
- [Executive Summary](#Executive-Summary)
- [Hardware](#hardware)
  - [Components (Bill of Materials)](#components-bill-of-materials)
  - [Mobility Management](#mobility-management)
  - [Power and Sense Management](#power-and-sense-management)
- [Software](#software)
- [Engineering Journal](#engineering-journal)
- [Photos & Videos](#photos--videos)
- [Testing & Validation](#testing--validation)
- [Project Management](#project-management)
- [How to Build](#how-to-build)
- [How to Run](#how-to-run)

---

## 👥 Team Rulo Bot — PUCMM SD

| Emil Velasquez | Wilmer Reyes | Alfonso Duverge |
|:---------------:|:-------------:|:----------------:|
| <img src="https://github.com/user-attachments/assets/25f1e48c-b0bc-413d-9e2e-95132b73f158" width="300"> | <img src="https://github.com/user-attachments/assets/d8a99eaf-8ccd-467a-b0cf-655299f9185f" width="300"> | <img src="https://github.com/user-attachments/assets/7e40e95a-828c-4b00-8adc-9d53fa350c04" width="300"> |

---



### Team Rulo Bot: 

![Imagen de WhatsApp 2025-08-23 a las 19 16 47_8771e2f9](https://github.com/user-attachments/assets/29deb411-65f3-4a44-be79-c59dec93f99c)

---

## Hardware

### Components (Bill of Materials)

| Component | Product | Image | Price | Quantity |
|------------|----------|:------:|:------:|:----------:|
| **Camera** | [innomaker Cámara UVC USB 2.0](https://www.amazon.com/dp/B0CNCSFQC1) | <img src="https://github.com/user-attachments/assets/6047e462-ebc7-4a6b-990e-7164869f2634" width="120"> | $18 | 1 |
| **Battery** | [Batería Lipo OVONIC 3S 50C 5200 mAh 11.1 V](https://a.co/d/hIsMyAB) | <img src="https://github.com/user-attachments/assets/a227fa6f-327b-4696-b1b1-d673a9621d41" width="120"> | $17 | 1 |
| **Traction Motor** | [Lego NXT Servo Motor](https://ebay.us/m/edWc1H) | <img src="https://github.com/user-attachments/assets/d69dd126-2536-4d84-bb0b-b062ca8b7bd2" width="120"> | $8 | 1 |
| **Motor Driver** | [DRV8871 Motor Driver](https://a.co/d/99h8can) | <img src="https://github.com/user-attachments/assets/bb43382e-0153-4fa2-90ef-53b1c02371e3" width="120"> | $23 | 1 |
| **Turning Motor** | [MG995 Servo Motor](https://a.co/d/fiWS9WQ) | <img src="https://github.com/user-attachments/assets/723466f3-fc8f-4c05-aa03-395575365e30" width="120"> | $15 | 1 |
| **Raspberry Pi 5 8 GB** | [RasTech Raspberry Pi 5 8 GB](https://a.co/d/0iE3Rae) | <img src="https://github.com/user-attachments/assets/6708444d-f624-42ef-81a7-70f443dfb033" width="120"> | $99 | 1 |
| **Voltage Converter** | [Mini360 DC-DC Converter](https://a.co/d/gaePhae) | <img src="https://github.com/user-attachments/assets/47b7fa63-0fb7-46a8-a4be-35bbdf58f11c" width="120"> | $8 | 3 |

**Total Car Hardware Cost:** **$188 USD**


---



### Executive Summary

Ackermann-steered autonomous car for WRO  2025. Fully 3D-printed chassis (SolidWorks, Bambu A1), LEGO NXT 9842 direct drive, MG995 steering, single-sensor vision (Innomaker UVC 2.0) on Raspberry Pi 5 with PD steering. Power: 3× Mini360 (Pi / DRV8871 logic / servo), Nano powered from Pi. Designed for robustness, fast serviceability, and strict rules compliance (≤300×200×300 mm).


| Front | Side | Top |
|:----:|:----:|:----:|
| <img src="media/front.jpg" width="250"> | <img src="media/side.jpg" width="250"> | <img src="media/top.jpg" width="250"> |


### Mobility Management

#### Chassis

The chassis is fully **3D-printed** and modeled in **SolidWorks**, then manufactured using a **Bambu Lab A1 printer**. It follows a **modular two-level architecture**, designed for easy assembly, maintenance, and compliance with **WRO Future Engineers** regulations (≤ 300 × 200 × 300 mm).

* **Lower level:** Contains the **LEGO NXT DC motor (9842, with encoder)**, which is **directly coupled to the rear axle** (no belt system). This direct drive ensures efficient torque transfer and minimizes mechanical losses. The motor is secured on a **dedicated printed motor mount**, integrated into the lower deck for maximum rigidity and precise alignment with the axle. The lower level also houses the **3S LiPo Battery 2200mAh 11.1V** and three **Mini360 DC-DC converters**. The battery is mounted in a dedicated bay that allows **fast hot-swap**, minimizing downtime during testing or competition.

* **Upper level:** Holds the **Raspberry Pi 5 (8GB)**, **Arduino Nano**, and additional electronics. This platform is mechanically isolated from the drivetrain to reduce vibration exposure to sensitive components. The **second level is attached to the lower deck using threaded standoffs**, creating a rigid but serviceable structure.

**Ackermann Steering System.**  

The steering system is a **fully 3D-printed Ackermann mechanism**, consisting of printed steering knuckles, tie rods, and a servo mount. An **MG995 servo motor** actuates the bell-crank mechanism, ensuring realistic wheel turning geometry and smooth maneuverability. Mechanical stops are integrated into the design to prevent over-travel and protect the servo. This system was optimized to operate within the servo’s safe range (24°–144°).

**Material selection.**

* The  **wheel supports**  are printed in **ABS**, providing greater strength, thermal stability, and impact resistance.  
* The remaining parts, such as covers and the camera mast, are printed in **PLA** for faster iteration and reduced weight.

**Fastening & serviceability.**  

All major parts are fixed with **heat-set threaded inserts** and M3 machine screws. This provides strong, reusable threads and makes repeated disassembly/reassembly possible without damaging the plastic. The modular assembly allows each subsystem (drive, steering, electronics, battery bay, camera mount) to be removed independently for quick service or replacement.

**Rigidity & accessibility.**  

The two-level design significantly increases torsional stiffness, preventing chassis flex during high-speed turns or obstacle maneuvers. The upper level can be detached in minutes, giving direct access to the drivetrain and battery compartment. Cable routing channels are integrated into the design to keep wiring organized and away from moving parts.

In summary, the chassis combines **strength, modularity, and accessibility**: a fully 3D-printed structure optimized for competition performance while remaining easy to maintain and upgrade.

 #### Potential Improvements
 
 


---


#### Design

![Imagen de WhatsApp 2025-09-17 a las 16 39 46_ed2e0446](https://github.com/user-attachments/assets/d28337ba-3f86-4259-9598-0add4eee0ef6)


#### Motors

The vehicle uses two active actuators: one for traction and one for steering. Both are integrated into the modular 3D-printed chassis and controlled by the Arduino Nano, with commands received from the Raspberry Pi 5.

**Traction Motor**  

- Component: **LEGO NXT DC Motor 9842 (with encoder)**  
- Placement: Mounted on the lower level of the chassis, fixed with a dedicated 3D-printed motor mount.  
- Coupling: The motor is **directly connected to the rear axle** (no belt or gear reduction). This configuration ensures efficient torque transfer and reduces mechanical complexity.  
- Performance: Powered at 11.1 V from the **3S LiPo Battery (2200mAh)**, it provides stable forward speed (~0.8 m/s) suitable for both the Open and Obstacle Challenges.  
- Encoder use: The integrated encoder allows for approximate lap counting and speed estimation, though not yet used for closed-loop speed control.

**Advantages of LEGO NXT DC Motor 9842**  

- Well-tested and reliable in educational robotics applications.  
- Integrated encoder provides basic feedback for distance/lap counting.  
- Compact design simplifies integration into the 3D-printed chassis.  
- Operates safely within the 11.1 V range of the 3S LiPo battery.  
- Adequate torque for a lightweight WRO Future Engineers vehicle.  

**Disadvantages of LEGO NXT DC Motor 9842**  

- Limited maximum speed compared to custom brushless DC motors.  
- Lower efficiency due to internal gearbox losses.  
- Encoder resolution is relatively coarse, limiting precision for advanced control.  
- Mechanical noise and vibration are higher than modern DC or stepper alternatives.  
- Availability is restricted, as the NXT system is discontinued and parts may only be sourced second-hand.  

---

**Steering Motor**  

- Component: **MG995 Servo Motor**  
- Placement: Mounted on the lower front section of the chassis, secured with 3D-printed brackets and heat-set inserts (PLA-based, not ABS).  
- Mechanism: The servo actuates a **fully 3D-printed Ackermann steering system** consisting of knuckles, tie rods, and a bell-crank.  
- Range: Calibrated to operate safely between **46° and 140°**, corresponding to full left and right lock.  
- Protection: Mechanical stops are integrated into the design to prevent over-travel and reduce stress on the servo gears.  
- Response: The servo provides sufficient torque and speed for sharp turns while maintaining stable control at competition speeds.

---

**System Integration**  

- The Arduino Nano controls both motors: PWM for traction through the **DRV8871 motor driver**, and PWM signal for the MG995 steering servo.  
- Both actuators are wired with strain relief and routed through dedicated cable channels in the chassis to avoid interference with moving parts.

---

#### Potential Improvements

To further enhance the motor system, the following upgrades are considered for future iterations:
1. Implementing **closed-loop speed control** using the LEGO NXT motor encoder for more precise lap timing and smoother acceleration.  
2. Replacing the **MG995 servo** with a digital high-torque servo for faster and more accurate steering response.  
3. Designing an **adjustable motor mount** that allows fine-tuning of axle alignment and easy replacement of the traction motor.  

These improvements would improve **precision, durability, and control**, especially under variable competition conditions.


---

### Power and Sense Management


![Imagen de WhatsApp 2025-09-17 a las 16 39 46_5ed24bac](https://github.com/user-attachments/assets/6ded3474-6ed6-440d-841a-7cd5bd2a3e89)

![Imagen de WhatsApp 2025-09-17 a las 16 51 18_32ba68bb](https://github.com/user-attachments/assets/ad3b5b2f-a5f9-4937-9d7e-7a979d43140c)



#### Sensors

For both the **Open Challenge** and the **Obstacle Challenge**, the vehicle relies exclusively on the **innomaker USB UVC 2.0 camera** as its primary and only sensing device. The camera is mounted on the upper level of the chassis and connected directly to the Raspberry Pi 5. Computer vision algorithms (OpenCV with LAB/HSV masks) are used to detect black lines, colored pillars (red and green), and the playfield boundaries.

This approach eliminates the need for ultrasonic sensors or IMUs, reducing system complexity and weight. However, it also introduces trade-offs that must be carefully considered.

**Advantages (Pros):**

- Simple architecture: only one sensing component to calibrate and maintain.  
- Weight reduction: fewer components, lighter overall design.  
- Cost efficiency: lower bill of materials compared to multi-sensor setups.  
- Flexibility: a single USB UVC camera can handle lane detection, wall detection, obstacle recognition, and color classification.  
- Compliance: fewer hardware points of failure, easier inspection during WRO competition checks.  

**Disadvantages (Cons):**

- Lighting sensitivity: performance depends heavily on ambient lighting; requires frequent mask calibration at each venue.  
- Processing load: real-time vision requires significant compute resources, relying on the Raspberry Pi 5 to maintain adequate FPS.  
- Failure risk: if the camera fails, the robot loses all sensing capability (no redundancy).  
- Limited robustness: vision may be affected by reflections, shadows, or worn playfield surfaces.  

**Summary:**  

The innomaker USB UVC 2.0 camera provides a **minimalistic and efficient sensing solution** for WRO Future Engineers, balancing cost, simplicity, and functionality. Nevertheless, it demands **careful calibration and robust vision algorithms** to mitigate the lack of distance sensing and lighting variability.

---

#### Potential Improvements

While the camera-only approach is functional, future iterations of the robot could be improved by:

1. Integrating an **IMU (gyroscope + accelerometer)** to improve orientation awareness and stability in sharp turns.  
2. Upgrading to a **global-shutter camera** to minimize motion blur at higher speeds.  
3. Implementing **sensor fusion** between vision and secondary sensors (ultrasonic + IMU) to increase robustness against noise and reflections.  
4. Introducing **redundancy**: a backup sensing modality would ensure the robot can still complete laps if the camera feed is compromised.  

These improvements would make the sensing system more **robust, adaptable to different environments, and resilient** to unexpected failures during competition.

#### Schematic

The schematic shows the complete power distribution and control system of the robot. It integrates the 3S LiPo battery, voltage regulators, the Arduino Nano, the Raspberry Pi 5, and the actuators (servo and DC motor).

![Imagen de WhatsApp 2025-08-14 a las 13 37 32_29657814](https://github.com/user-attachments/assets/e6b29fac-5bbe-44b5-b904-1bcb2de3b229)

**Power Source**  

The main source is a **3S LiPo battery (11.1 V, 2200 mAh)**. A main power switch (SW2) is placed between the battery and the system to allow safe startup and shutdown.

**Voltage Regulation**  

Three **Mini360 DC-DC converters** step down the 11.1 V battery to 5 V: one dedicated to the **Raspberry Pi 5**, one for the **MG995 steering servo**, and one for the **DRV8871 motor driver logic**. The Mini360 converters were chosen because of their compact size and ease of integration into the chassis. The **Arduino Nano is powered directly from the Raspberry Pi 5** via USB, sharing the same 5 V rail. This distribution ensures that each critical subsystem (compute, steering, and motor control) has its own regulated supply, reducing noise coupling and improving reliability.

**Control Electronics**  

The **Arduino Nano** reads the encoder from the LEGO NXT DC motor, controls the MG995 servo through PWM, and sends commands to the DRV8871 motor driver to control traction. It is powered directly by the Raspberry Pi 5. The **Raspberry Pi 5** runs computer vision using OpenCV, interfaces with the Innomaker UVC 2.0 USB camera, and sends steering and speed commands to the Arduino Nano via serial communication.

**Actuators**  

The **MG995 servo** is connected to the Arduino Nano (PWM) and powered by its dedicated Mini360 regulator. The **LEGO NXT DC Motor 9842** is connected to the DRV8871 motor driver, which is controlled by the Arduino Nano through IN1/IN2. The driver receives direct 11.1 V from the LiPo for the power stage and 5 V from a Mini360 for its logic.

**Feedback Devices**  

The **encoder from the LEGO NXT motor** is connected to the Arduino Nano digital inputs, providing lap and distance estimation (optional closed-loop control). A status LED (D1) connected to the Arduino Nano is used for debugging and arming indication.

---

#### Potential Improvements

Future iterations could include fuse protection on the LiPo input and each 5 V rail, TVS diodes or snubber circuits for the DC motor to protect against voltage spikes, higher-current buck converters to replace the Mini360 units for better load stability, additional filter capacitors near the servo and motor driver for noise suppression, and eventually a custom PCB to integrate regulators, the Arduino, and connectors into a more compact and reliable form factor.


---

## Software
### Initialization and Connection Process
### Object Management
#### Object Detection (Open Challenge & Obstacle Challenge)
#### Wall Detection/Management (Open & Obstacle)
#### Signal Pillar Detection/Management (Obstacle)
#### Turning (Open Challenge)
#### Turning (Obstacle Challenge)
#### Parking Lot Detection/Management (Obstacle)
#### Three-Point Turn (Obstacle)
#### Backing Up (Obstacle)
#### Stuck/Recovery Detection (Potential Improvement)

---

## Engineering Journal

---

## Photos & Videos

### Robot Photos (All Sides / Top / Bottom)

- Vista superior / Top view
![Imagen de WhatsApp 2025-09-10 a las 21 23 18_842bc3e9](https://github.com/user-attachments/assets/ff5a0810-a045-4091-b874-07834a525935)


- Vista lateral / side view

![Imagen de WhatsApp 2025-09-10 a las 21 23 18_51f527b5](https://github.com/user-attachments/assets/589cc5e2-9df6-4363-a30e-01bb171a2bc5)


- Frontal / Front
![Imagen de WhatsApp 2025-09-10 a las 21 23 18_63f6fab2](https://github.com/user-attachments/assets/ee56a6c2-5570-4eb1-90ed-79a46c1e125c)



### Performance Videos — Team Rulo Bot Testing Sessions

**AUTONOMOUS ROBOT TEST – Forward and Reverse Motion**  
[https://youtube.com/shorts/DLdXPhjLxYY](https://youtube.com/shorts/DLdXPhjLxYY)  
In this test, we were verifying the behavior of the traction system using the LEGO NXT 9842 motor and the DRV8871 driver.  
The goal was to confirm that the robot could move forward and backward smoothly under PWM control, maintaining stable direction changes without losing alignment or causing mechanical vibration.

**AUTONOMOUS ROBOT TEST – Vision Test: Red and Green Color Detection**  
[https://youtu.be/MnDNOvI_E2s](https://youtu.be/MnDNOvI_E2s)  
During this test, we were calibrating the color detection system of the provisional camera.  
We focused on adjusting the HSV and LAB masks in OpenCV to correctly identify red and green pillars, which are essential for obstacle decision-making during the Obstacle Challenge.

**TEST – Forward and Backward**  
[https://www.youtube.com/shorts/6pMi8gZCbPo](https://www.youtube.com/shorts/6pMi8gZCbPo)  
In this experiment, we were testing the encoder’s response and traction symmetry by moving the robot back and forth multiple times.  
The purpose was to verify that both acceleration and braking were consistent, and that the robot maintained linear stability when changing direction.

**OPEN CHALLENGE – Clockwise Run**  
[https://www.youtube.com/watch?v=ao2uUjXZPo4](https://www.youtube.com/watch?v=ao2uUjXZPo4)  
Here we were performing one of the first complete runs of the Open Challenge in the clockwise direction.  
We tested PD steering tuning, lane centering using the front ROI, and overall stability during continuous lap navigation around the playfield.

**OPEN CHALLENGE – Counterclockwise Run**  
[https://www.youtube.com/watch?v=F6bJ-BMMgI8](https://www.youtube.com/watch?v=F6bJ-BMMgI8)  
This session was focused on validating the same Open Challenge run but in the counterclockwise direction.  
We were confirming that the PD control, ROI detection, and camera calibration worked equally well in both orientations, ensuring symmetrical performance and algorithm consistency.




---

## Testing & Validation
### Test Plan
### Metrics
### Results

---

## Project Management

### Project Roadmap

<img width="1024" height="1024" alt="image" src="https://github.com/user-attachments/assets/416e7f67-6f73-4500-b4c9-ddf3adf59ac5" />


**Start date:** July 23, 2025  
**Competition date:** October 19, 2025  
**Robot completion deadline:** October 5, 2025  

---

#### Phase 1 — Project Kickoff (July 23 – August 5)
Define rules, scoring, team roles, and overall strategy.  
**Deliverable:** Finalized competition strategy and design concept.

#### Phase 2 — Design Bootcamp (August 6 – 9)
Complete mechanical and electronic design (SolidWorks + KiCad).  
**Deliverable:** Full CAD model and circuit ready for fabrication.

#### Phase 3 — Programming Bootcamp (August 11 – 12)
Configure Raspberry Pi 5, Arduino, OpenCV, and GitHub workflow.  
**Deliverable:** Base software environment and first prototype code.

#### Phase 4 — Assembly & First Tests (August 13 – 23)
Assemble printed and electronic components, perform initial testing.  
**Deliverable:** Robot assembled and first GitHub commit (Aug 23).

#### Phase 5 — Integration & Tuning (August 25 – 29)
Integrate sensors, camera, and PD control; run Open/Obstacle trials.  
**Deliverable:** Autonomous navigation demonstrated (Aug 30).

#### Phase 6 — Optimization & Validation (September 1 – 27)
Optimize performance, speed, and detection reliability.  
**Deliverable:** Final performance metrics and second GitHub commit (Sept 19).

#### Phase 7 — Final Review & Competition Prep (Sept 30 – Oct 3)
Stress tests, documentation, and competition readiness check.  
**Deliverable:** Final GitHub commit (Oct 4) and readiness for competition (Oct 19, Panama 🇵🇦).

### Milestones
### Risks & Mitigations

---

## How to Build
### Mechanical Assembly
### Electrical Wiring
### Calibration & Safety

---

## How to Run
### Prerequisites
### Installation
### Configuration
### Launch Commands

---

