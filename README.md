# Team Rulo Bot – WRO Future Engineers 2025
---

## 📑 Table of Contents
- [Repository Contents](#repository-contents)
- [Team](#team)
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

## Repository Contents
- `hardware/`
- `software/`
- `docs/`
- `datasets/`
- `tests/`

---

## Team

* Alfonso Duverge Urbaez, 20, alfonso1872002@gmail.com  
* Wilmer De Jesus Reyes Castillo, 18, reyeswilmer648@gmail.com  
* Emil Gustavo Velasquez Canela, 19, emilgvelasquez@gmail.com  

---

## Hardware

### Components (Bill of Materials)

| Name            | Product                                                                 | Price | Quantity |
|-----------------|-------------------------------------------------------------------------|-------|----------|
| Camera          | [innomaker Cámara UVC USB 2.0]( https://www.amazon.com/dp/B0CNCSFQC1)   | $20   | 1        |
| Battery         | [3S Lipo Battery 2200mAh 11.1V](https://a.co/d/3V4ITBv)                 | $34   | 1        |
| Traction Motor  | [Lego NXT Servo Motor](https://ebay.us/m/edWc1H)                        | $8    | 1        |
| Motor Driver    | [DRV8871 Motor Driver](https://a.co/d/99h8can)                          | $23   | 1        |
| Turning Motor   | [MG995 Servo Motor](https://a.co/d/fiWS9WQ)                             | $15   | 1        |
| Raspberry Pi    | [RasTech Raspberry Pi 5 8GB](https://a.co/d/0iE3Rae)                    | $99   | 1        |
| Voltage Converter | [Mini360 DC-DC Voltage Power Converter](https://a.co/d/gaePhae)       | $8    | 3        |

**Total Car Hardware Cost:** **$207 USD**

---

### Mobility Management

#### Chassis

The chassis is fully **3D-printed** and modeled in **SolidWorks**, then manufactured using a **Bambu Lab A1 printer**. It follows a **modular two-level architecture**, designed for easy assembly, maintenance, and compliance with **WRO Future Engineers** regulations (≤ 300 × 200 × 300 mm).

* **Lower level:** Contains the **LEGO NXT DC motor (9842, with encoder)**, which is **directly coupled to the rear axle** (no belt system). This direct drive ensures efficient torque transfer and minimizes mechanical losses. The motor is secured on a **dedicated printed motor mount**, integrated into the lower deck for maximum rigidity and precise alignment with the axle. The lower level also houses the **3S LiPo Battery 2200mAh 11.1V** and three **Mini360 DC-DC converters**. The battery is mounted in a dedicated bay that allows **fast hot-swap**, minimizing downtime during testing or competition.

* **Upper level:** Holds the **Raspberry Pi 5 (8GB)**, **Arduino Nano**, and additional electronics. This platform is mechanically isolated from the drivetrain to reduce vibration exposure to sensitive components. The **second level is attached to the lower deck using threaded standoffs**, creating a rigid but serviceable structure.

**Ackermann Steering System.**  
The steering system is a **fully 3D-printed Ackermann mechanism**, consisting of printed steering knuckles, tie rods, and a servo mount. An **MG995 servo motor** actuates the bell-crank mechanism, ensuring realistic wheel turning geometry and smooth maneuverability. Mechanical stops are integrated into the design to prevent over-travel and protect the servo. This system was optimized to operate within the servo’s safe range (46°–140°).

**Material selection.**
* The **steering knuckles** and **wheel supports**  are printed in **ABS**, providing greater strength, thermal stability, and impact resistance.  
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


---

### Power and Sense Management

#### Power and Wiring

![Imagen de WhatsApp 2025-09-17 a las 16 39 46_5ed24bac](https://github.com/user-attachments/assets/6ded3474-6ed6-440d-841a-7cd5bd2a3e89)
![Imagen de WhatsApp 2025-09-17 a las 16 51 18_32ba68bb](https://github.com/user-attachments/assets/ad3b5b2f-a5f9-4937-9d7e-7a979d43140c)



#### Sensors

#### Schematic

![Imagen de WhatsApp 2025-08-14 a las 13 37 32_29657814](https://github.com/user-attachments/assets/e6b29fac-5bbe-44b5-b904-1bcb2de3b229)

#### Potential Improvements

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

### Team Photos

![Imagen de WhatsApp 2025-08-23 a las 19 16 47_8771e2f9](https://github.com/user-attachments/assets/29deb411-65f3-4a44-be79-c59dec93f99c)

- [Alfonso Duverge Urbaez]
  
![Imagen de WhatsApp 2025-09-18 a las 18 30 46_69a62026](https://github.com/user-attachments/assets/7e40e95a-828c-4b00-8adc-9d53fa350c04)

- [Wilmer De Jesus Reyes Castillo]
  
![Imagen de WhatsApp 2025-09-18 a las 18 30 46_d27cedc8](https://github.com/user-attachments/assets/d8a99eaf-8ccd-467a-b0cf-655299f9185f)
  
- [Emil Gustavo Velasquez Canela]
  
![Imagen de WhatsApp 2025-09-18 a las 18 30 46_6bc5796d](https://github.com/user-attachments/assets/25f1e48c-b0bc-413d-9e2e-95132b73f158)

### Robot Photos (All Sides / Top / Bottom)

- Vista superior / Top view
![Imagen de WhatsApp 2025-09-10 a las 21 23 18_842bc3e9](https://github.com/user-attachments/assets/ff5a0810-a045-4091-b874-07834a525935)


- Vista lateral / side view

![Imagen de WhatsApp 2025-09-10 a las 21 23 18_51f527b5](https://github.com/user-attachments/assets/589cc5e2-9df6-4363-a30e-01bb171a2bc5)


- Frontal / Front
![Imagen de WhatsApp 2025-09-10 a las 21 23 18_63f6fab2](https://github.com/user-attachments/assets/ee56a6c2-5570-4eb1-90ed-79a46c1e125c)



  
### Performance Videos (Open / Obstacle)


AUTONOMOUS ROBOT TEST – Forward and Reverse Motion (https://youtube.com/shorts/DLdXPhjLxYY)

AUTONOMOUS ROBOT TEST  - Vision Test -  Red and Green Color Detection Test (https://youtu.be/MnDNOvI_E2s)

TEST FORWARD AND BACKWARD - https://www.youtube.com/shorts/6pMi8gZCbPo

OPEN CHALLENGE - CLOCKWISE - https://www.youtube.com/watch?v=ao2uUjXZPo4

OPEN CHALLENGE - COUNTERCLOCKWISE - https://www.youtube.com/watch?v=F6bJ-BMMgI8


---

## Testing & Validation
### Test Plan
### Metrics
### Results

---

## Project Management
### Roadmap
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

