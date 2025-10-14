<a id="top"></a>

# 1. Team Rulo Bot – WRO Future Engineers 2025



![Imagen de WhatsApp 2025-10-14 a las 14 31 22_ee88e1e7](https://github.com/user-attachments/assets/726679e4-21a8-4da4-be9c-fa01bc62a6e3)



---

## 1.1 Overview & Repository Content

| Folder/File        | Description                                                                                                     |
| :----------------- | :-------------------------------------------------------------------------------------------------------------- |
| **`3D_models`**    | 3D models of the vehicle and rendered images.                                                                   |
| **`schemes`**      | Electrical and electronic schematic diagrams (PNG).                                                             |
| **`technical_reports`**   | engineering documentation(Mechanical,Electronics & Power Report,Software Architecture,Development History & Modifications)                                                        |
| **`src`**          | Source code for Raspberry Pi (Python) and Arduino (C++): PD steering, obstacle avoidance and vision algorithms. |
| **`t-photos`**      | Photos of Team Rulo Bot during competition sessions, testing and behind-the-scenes moments.                     |
| **`t-videos`**      | YouTube links or embedded demos showing the robot’s performance in each mission.                                |
| **`robot-photos`** | Gallery of the robot including top, side and front perspectives of the vehicle.                                 |
| **`readme`**       | Team,Executive Summary,BOM,Gallery,Testing y validation,project Management                                      |

---

### 📑 Table of Contents
This README is organized into numbered sections for clarity and fast navigation:

* [2. Team](#sec-2)
* [3. Executive Summary](#sec-3)
* [4. Bill of Materials](#sec-4)
* [5. Gallery](#sec-5)
* [6. Testing & Validation](#sec-7)
* [7. Project Management](#sec-8)

---

<a id="sec-2"></a>

## 2. Team Rulo Bot — PUCMM SD

|                                              Emil Velasquez                                             |                                               Wilmer Reyes                                              |                                             Alfonso Duverge                                             |
| :-----------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------: |
| <img src="https://github.com/user-attachments/assets/25f1e48c-b0bc-413d-9e2e-95132b73f158" width="300"> | <img src="https://github.com/user-attachments/assets/d8a99eaf-8ccd-467a-b0cf-655299f9185f" width="300"> | <img src="https://github.com/user-attachments/assets/7e40e95a-828c-4b00-8adc-9d53fa350c04" width="300"> |

---

### 2.1 Team Rulo Bot

![Imagen de WhatsApp 2025-08-23 a las 19 16 47_8771e2f9](https://github.com/user-attachments/assets/29deb411-65f3-4a44-be79-c59dec93f99c)

[↑ Back to top](#top)

---

<a id="sec-3"></a>

## 3. Executive Summary

Ackermann-steered autonomous car for WRO 2025.  
Fully 3D-printed chassis (SolidWorks, Bambu A1), LEGO NXT 9842 direct drive, MG995 steering, single-sensor vision (Innomaker UVC 2.0) on Raspberry Pi 5 with PD steering.  
Power: 3× Mini360 (Pi / DRV8871 logic / servo), Nano powered from Pi.  
Designed for robustness, fast serviceability, and strict rules compliance (≤300×200×300 mm).

[↑ Back to top](#top)

---

<a id="sec-4"></a>

## 4. Bill of Materials (BOM)

### 4.1 Components

| Component               | Product                                                              |                                                  Image                                                  | Price | Quantity |
| ----------------------- | -------------------------------------------------------------------- | :-----------------------------------------------------------------------------------------------------: | :---: | :------: |
| **Camera**              | [innomaker Cámara UVC USB 2.0](https://www.amazon.com/dp/B0CNCSFQC1) | <img src="https://github.com/user-attachments/assets/6047e462-ebc7-4a6b-990e-7164869f2634" width="120"> |  $18  |     1    |
| **Battery**             | [Batería Lipo OVONIC 3S 50C 5200 mAh 11.1 V](https://a.co/d/hIsMyAB) | <img src="https://github.com/user-attachments/assets/a227fa6f-327b-4696-b1b1-d673a9621d41" width="120"> |  $17  |     1    |
| **Traction Motor**      | [Lego NXT Servo Motor](https://ebay.us/m/edWc1H)                     | <img src="https://github.com/user-attachments/assets/d69dd126-2536-4d84-bb0b-b062ca8b7bd2" width="120"> |   $8  |     1    |
| **Motor Driver**        | [DRV8871 Motor Driver](https://a.co/d/99h8can)                       | <img src="https://github.com/user-attachments/assets/bb43382e-0153-4fa2-90ef-53b1c02371e3" width="120"> |  $23  |     1    |
| **Turning Motor**       | [MG995 Servo Motor](https://a.co/d/fiWS9WQ)                          | <img src="https://github.com/user-attachments/assets/723466f3-fc8f-4c05-aa03-395575365e30" width="120"> |  $15  |     1    |
| **Raspberry Pi 5 8 GB** | [RasTech Raspberry Pi 5 8 GB](https://a.co/d/0iE3Rae)                | <img src="https://github.com/user-attachments/assets/6708444d-f624-42ef-81a7-70f443dfb033" width="120"> |  $99  |     1    |
| **Voltage Converter**   | [Mini360 DC-DC Converter](https://a.co/d/gaePhae)                    | <img src="https://github.com/user-attachments/assets/47b7fa63-0fb7-46a8-a4be-35bbdf58f11c" width="120"> |   $8  |     3    |

**Total Car Hardware Cost:** **$188 USD**

[↑ Back to top](#top)

---

<a id="sec-5"></a>

## 5. Gallery

### 5.1 Robot Views (Front / Side / Top)

| Front | Side | Top |
|:------:|:----:|:---:|
| <img src="https://github.com/user-attachments/assets/25f3d40a-215f-4ea2-bea8-40bcff1fbbed" width="250"> | <img src="https://github.com/user-attachments/assets/865b8981-3acf-4259-b72a-254d87498ab6" width="250"> | <img src="https://github.com/user-attachments/assets/d08c9ad8-2ab7-4a6a-b362-f7af385b26de" width="250"> |

[↑ Back to top](#top)

---

<a id="sec-7"></a>

## 6. Testing & Validation

### 6.1 Performance Videos 

| Video                                                                              | Description                                                                                                |
| :--------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------- |
| [🔗 Traction test](https://youtube.com/shorts/DLdXPhjLxYY)                         | Verifies the response of the NXT motor and DRV8871 driver in forward and reverse motion under PWM control. |
| [🔗 Colour detection](https://youtu.be/MnDNOvI_E2s)                                | Calibrates HSV and LAB masks to correctly identify red and green pillars.                                  |
| [🔗 Back-and-forth test](https://www.youtube.com/shorts/6pMi8gZCbPo)               | Evaluates encoder response and traction symmetry by moving forward and backward repeatedly.                |
| [🔗 Open Challenge clockwise](https://www.youtube.com/watch?v=ao2uUjXZPo4)         | First complete lap clockwise, tuning PD steering and lane centering.                                       |
| [🔗 Open Challenge counter-clockwise](https://www.youtube.com/watch?v=F6bJ-BMMgI8) | Counter-clockwise run, verifying control symmetry and detection consistency.                               |

### 6.2 Test Plan

| Nº | Test Name | Objective | Method / Description | Expected Result |
|:--:|:-----------|:-----------|:---------------------|:----------------|
| 1 | Traction Test | Verify motor and driver performance (DRV8871 + LEGO NXT) | Run forward/backward cycles at 50%, 75%, 100% PWM. Measure speed and smoothness. | Stable acceleration and braking without vibration. |
| 2 | Steering Calibration | Validate Ackermann servo geometry | Sweep MG995 servo (46°–140°) and check alignment vs wheel angles. | Smooth response; no servo jitter or over-travel. |
| 3 | PD Control Tuning | Optimize steering response for lane centering | Adjust P and D gains on Raspberry Pi until oscillations disappear. | Car maintains lane without overshoot or drift. |
| 4 | Vision Calibration | Verify camera detection (black / orange / red / green) | Adjust LAB and HSV masks under different lighting conditions. | Reliable color detection ≥ 95 % accuracy indoors. |
| 5 | Obstacle Challenge | Validate avoidance logic | Detect red/green pillars; trigger turn and resume path. | Passes obstacle without contact; re-centers quickly. |
| 6 | Endurance Test | Confirm battery life + thermal stability | Continuous run > 15 min measuring temperature and voltage. | Stable 5 V ± 0.05 V; no overheating or resets. |

---

### 6.3 Metrics

| Metric | Measured Value | Target | Result |
|:--------|:---------------|:--------|:--------|
| Average Speed | 0.86 m/s | ≥ 0.8 m/s | ✅ Pass |
| Steering Range | 94° (46°–140°) | ≥ 90° | ✅ Pass |
| Camera FPS | 28 fps | ≥ 25 fps | ✅ Pass |
| Power Stability | 5.02 V ± 0.05 V | 5.0 V ± 0.1 V | ✅ Pass |
| Runtime Batería Lipo OVONIC 3S 50C 5200 mAh 11.1 V) | 18 min | ≥ 15 min | ✅ Pass |

---

### 6.4 Results

> The Rulo Bot successfully completed both the Open and Obstacle challenges.  
> PD steering achieved stable trajectory tracking with quick correction after turns.  
> The Innomaker UVC 2.0 camera performed reliably once color masks were tuned for indoor lighting.  
> The 3D-printed chassis maintained rigidity during high-speed maneuvers, and electrical systems remained within nominal voltage limits.  
> For future improvements, the team plans to integrate an IMU for orientation feedback and automatic color calibration for dynamic environments.

[↑ Back to top](#top)

---

<a id="sec-8"></a>

## 7. Project Management

### 7.1 Official Roadmap

 <img width="1188" height="695" alt="image" src="https://github.com/user-attachments/assets/ab3ef720-c97d-4e07-9602-d79629d049f5" /> 


---

### 7.2 Work Plan —  RuloBot (Sept 22 → Nov 24 2025)

Sept 22 – 23 → Mechanical / Electrical / Programming Setup

Adjust camera mount; replace wheel supports (PLA → ABS).

Improve cable isolation and solder joints; test servo feedback.

Finalize Open Challenge code and begin Obstacle Challenge logic.

Upload updated photos and documentation to GitHub.

🔹 Sept 23 – 26 → Optimization & Documentation

Refine mechanical parts with tolerance corrections.

Optimize Open Challenge code and calibrate obstacle colors.

Prepare initial Obstacle Challenge commit and unified logic.

Complete hardware table and mechanical documentation.

🔹 Sept 27 → Testing Session #1 (Fogueo #1)

Full-track trial run (3 laps Open + Obstacle).

Collect timing and steering-error data.

🔹 Sept 28 → Post-Test Documentation

Edit and upload videos of Fogueo #1.

Update code commits with feedback results.

🔹 Sept 29 – Oct 3 → Reliability Preparation

Print spare parts; purchase backup servo and connectors.

Run timed tests at multiple speeds; document mechanical stability.

Update GitHub with full code explanation and wiring guide.

🔹 Oct 4 → Testing Session #2 (Fogueo #2)

Validate improvements vs Fogueo #1.

Measure consistency ≥ 70 % success rate per lap.

🔹 Oct 5 – 13 → Final Integration & Deadline

Validate ABS durability and motor endurance on full track.

Test battery autonomy and servo calibration under stress.

Validate autonomous parking (encoder + vision).

Perform final GitHub commits and upload photos/videos.

🔹 Oct 14 – 18 → Pre-Competition Week

Conduct final rehearsals on official track replica.

Perform final color calibration (HSV / LAB).

Check contingency kit (spare servo, motor, battery, cables).

Prepare travel materials, laptop, chargers, and tool kit.

🔹 Oct 18 – Nov 24 → Competition Day – Singapore 🇸🇬

Practice and final testing on official track.

Runs: Open Challenge + Obstacle Challenge.

Submit printed documentation and publicar repositorio.

---

### 7.3 Deliverables

- Robot fully operational and documented on GitHub.  
- Final videos of both challenges (≥ 30 s each).  
- Three commits minimum: Initial / Optimization / Release.  
- PDF reports (mechanical · electrical · software) uploaded under `/readme`.  

[↑ Back to top](#top)

