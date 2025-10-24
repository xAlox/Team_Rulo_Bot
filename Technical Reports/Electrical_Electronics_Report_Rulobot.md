# ⚡ Electrical and Electronics Documentation – Team Rulo Bot

The electrical and electronic system of **Team Rulo Bot** has been designed to provide stable power delivery, high-performance control, and reliable signal communication between all robot subsystems. This architecture supports **real-time vision**, **Ackermann steering**, and **precise traction control** during the [WRO Future Engineers 2025](https://wro-association.org/future-engineers/) competition.

The backbone of the system is a **3S 11.1 V LiPo battery**, which feeds a **main power switch** and then distributes power through three **Mini360 buck converters**:
- **5.2 V rail** → Raspberry Pi 5 + Arduino Nano + peripherals.  
- **6.0 V rail** → MG995 servo (steering).  
- **11.1 V rail (direct)** → DRV8871 motor driver.

This configuration ensures stable operation of each subsystem while isolating high-load actuators from sensitive logic components.

---

##  Main Control Units

### Raspberry Pi 5 — Main Processor
The **Raspberry Pi 5** was selected as the brain of the robot thanks to its:
- Quad-core ARM Cortex-A76 CPU @ 2.4 GHz and up to 8 GB RAM.
- Excellent USB 3.0 support for the vision system.
- Reliable **UART serial communication** with the Arduino Nano.
- High processing power to handle **real-time vision** and **navigation algorithms**.
 <img width="638" height="475" alt="image" src="https://github.com/user-attachments/assets/b95f61c8-c310-4c98-9eb4-3f81e7aecc04" />



Powered from the **5.2 V Mini360 rail**, the Pi processes the UVC camera feed, runs the navigation logic, and transmits control signals to the Arduino Nano with low latency. Its stable power supply and strong I/O capabilities make it the centerpiece of the control architecture.

---

### Arduino Nano — Auxiliary Controller
The **Arduino Nano** complements the Raspberry Pi by handling **time-critical, low-level tasks**:
- Reading NXT motor encoder pulses with interrupt precision.
- Generating PWM for the MG995 servo.
- Monitoring battery voltage and communicating alerts.
  <img width="600" height="406" alt="image" src="https://github.com/user-attachments/assets/4f6137b9-6139-4288-8654-c9b2d9b382e9" />


It was chosen for its **compact form factor**, **low power consumption**, and **ease of code deployment**, making iteration and testing fast. The Nano is powered from the same 5.2 V logic rail as the Pi.

---

##  Vision System

### Innomaker UVC 2.0 Camera
The **Innomaker UVC 2.0 Camera** is a plug-and-play USB module chosen for:
- **103° field of view**, ideal for detecting nearby walls and obstacle pillars.
- Full HD 1080p @ 30 FPS streaming over USB.
- UVC protocol — no extra drivers required on Raspberry Pi.
- Low power consumption (~0.25 W).
<img width="389" height="338" alt="image" src="https://github.com/user-attachments/assets/e139cbc9-2568-467b-87ca-0443ba08512f" />



Its wide angle enables the robot to perceive both **forward and lateral boundaries** of the track, improving situational awareness and path planning.

---

##  Actuation System

### MG995 Servo — Ackermann Steering
The **MG995 Servo** servo was chosen for:
- High torque (≈ 11 kg·cm @ 6 V)  
- Fast response (0.16 s/60°)  
- Good durability and cost-efficiency

  <img width="550" height="550" alt="image" src="https://github.com/user-attachments/assets/1ed23ec3-ba8d-4985-a6ec-21cd5eba71c0" />


It is powered from a **dedicated 6 V Mini360 buck**, ensuring that its current spikes do not interfere with the Pi or Nano. This allows **sharp, precise steering** under load without brownouts.

---

### LEGO NXT Motor with Encoder — Traction
The **LEGO NXT Motor** provides:
- Reliable traction power.
- **360 PPR encoder resolution** for accurate speed and distance feedback.
- Robust gearbox for continuous operation.
 
  <img width="510" height="324" alt="image" src="https://github.com/user-attachments/assets/3f8d4354-f023-4150-a941-f5090c0d55fd" />  <img width="503" height="242" alt="image" src="https://github.com/user-attachments/assets/26f98c4d-53af-4a06-aac7-54fe70405462" />



Although rated for 9 V nominal, it performs well at 11.1 V when controlled through PWM and current-limited with the driver, increasing speed while maintaining safe thermal limits.

---

##  Power Electronics

### DRV8871 Motor Driver
The **DRV8871 Motor Driver** driver is powered **directly from the battery’s 11.1 V line** and was selected because:
- It supports up to 45 V and 3.6 A peak.
- Has **built-in current limiting** and protection (UVLO, OCP, TSD).
- Is extremely **compact**, requiring no dedicated mounting space.

  <img width="470" height="397" alt="image" src="https://github.com/user-attachments/assets/e575d226-e0db-4cb5-bc38-2cacd30c1c1c" />


Its combination of power and small footprint allows clean integration with minimal wiring.

---

### Mini360 Buck Converters + Output Filtering
The **Mini360 Buck Converters** modules provide regulated rails for logic and servo power:
- 5.2 V for Raspberry Pi & Arduino.
- 6.0 V for the MG995 servo.
- 11.1 V pass-through for DRV8871.

  <img width="625" height="310" alt="image" src="https://github.com/user-attachments/assets/3a2c9345-befe-4edb-8385-49e41e29da36" />


The **servo rail includes a 47 µF electrolytic capacitor** mounted at the Mini360 output.  
🔸 **Why:** During early testing, the MG995 servo generated electrical noise and voltage dips that interfered with control signals and caused random resets.  
✅ **Solution:** Adding this capacitor provided a local energy reservoir and filtered out high-frequency spikes, stabilizing the rail and eliminating signal noise problems that persisted during the first development days.

<img width="365" height="152" alt="image" src="https://github.com/user-attachments/assets/3d7077bf-59d2-40eb-93a6-ef474e9a1a5d" />


These converters are **very compact** and mounted **inline with the wiring using thermo-shrink**, optimizing internal space and weight.

---

### Ovonic 3S 11.1 V 5200 mAh LiPo Battery
The **Ovonic 3S 11.1 V 5200 mAh LiPo Battery** is the **main energy source**:
- High capacity and discharge rating for sustained performance.
- Stable voltage for long runs.
- Connected to the **main power switch** and a 10 A fuse for safety.

  <img width="600" height="306" alt="image" src="https://github.com/user-attachments/assets/24b6a811-b43a-4d81-b3ee-2693108be3b1" />


Even though it’s physically large, its **high energy density** ensures stable power delivery during long testing sessions and competition rounds.

---

##  Power Topology

<img width="803" height="675" alt="image" src="https://github.com/user-attachments/assets/38bdb96b-2364-4b07-b104-274dd5b485fd" />



-The main power switch controls the entire power distribution, allowing the system to be turned on and off from a single point.

-The voltage rails are regulated independently, improving system stability and preventing fluctuations from affecting other subsystems.

-The 47 µF capacitor on the servo rail filters electrical noise and prevents signal distortion, increasing steering reliability.

-The star grounding topology minimizes noise and voltage drops, ensuring stable sensor readings and control signals.

-The modular power distribution design makes it easier to isolate faults and simplifies electrical diagnostics during testing or maintenance.

---

## 🧪 Protection & Safety Measures 
- 🔸 DRV8871 current limiting to protect motor and battery.  
- 🔸 Independent 6 V rail with filtering to isolate servo noise.  
- 🔸 Star ground topology prevents resets and ground loops.  
- 🔸 Safe power-up and shutdown sequence to protect the Pi.

---

##  Power Budget

| Component                    | Voltage | Avg Current | Peak Current | Notes                                              |
|------------------------------|---------|------------:|------------:|----------------------------------------------------|
| Raspberry Pi 5              | 5.2 V   | ~1 A        | 2–3 A       | Vision & decision logic                            |
| Arduino Nano               | 5.2 V   | 0.05 A      | 0.05 A      | Lightweight coprocessor                           |
| UVC Camera                 | 5.2 V   | 0.05 A      | 0.05 A      | Wide FOV, low power                               |
| MG995 Servo                | 6.0 V   | 0.1–0.5 A   | 2 A         | Isolated rail + 47 µF cap for noise suppression     |
| NXT Motor                  | 11.1 V  | 0.2–1 A     | 2 A         | PWM controlled through driver                     |
| DRV8871 Driver             | 11.1 V  | <0.01 A     | 3.6 A max   | Compact and protected                             |
| Ovonic LiPo Battery        | 11.1 V  | —           | —           | Long runtime, stable voltage                      |

---

## 🏁 Conclusion
The electrical system of **Team Rulo Bot** is designed to maximize **performance**, **stability**, and **noise immunity**:
- 🧠 **Raspberry Pi 5** handles demanding vision processing.  
- 🛠️ **Arduino Nano** provides fast low-level control and monitoring.  
- 📸 The **UVC camera** ensures wide and reliable visual coverage.  
- ⚡ **MG995 + DRV8871 + Mini360** deliver power and torque in a compact footprint.  
- 🔋 The **Ovonic 5200 mAh LiPo** ensures long runtime and consistent voltage.  
- 🧰 The **47 µF filter capacitor** on the servo rail solved early noise issues, improving overall system stability.

This power architecture allows the robot to operate with excellent energy stability, electrical isolation between subsystems, and built-in protection against failure scenarios. It is optimized for competition, reliability, and ease of maintenance.

---

## 📚 References
- Tower Pro MG995 Datasheet — sandorobotics.com.mx  
- DRV8871 Motor Driver — ti.com  
- LEGO NXT Motor 9842 Reference — lagos.udg.mx  
- Mini360 Buck Converter — components101.com  
- Raspberry Pi Power Guidelines — forums.raspberrypi.com  
- UVC Camera Specs — spanish.module-camera.com  
- Ovonic LiPo Battery — dynamoelectronics.com


