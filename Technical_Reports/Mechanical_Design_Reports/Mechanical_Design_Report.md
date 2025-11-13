# Hardware Documentation and Setup Guide
The Rulo Bot platform was designed to compete in WRO Future Engineers 2025.
It is a compact, modular Ackermann-steered vehicle integrating sensors, actuators, and
processing units.
Its architecture combines a Raspberry Pi 5 as the main computer and an Arduino Nano
for real-time traction and steering control.

# 1. System Overview

The robot is built on a 3D-printed Ackermann chassis, optimized for modularity,
structural rigidity, and ease of maintenance.
Power distribution is centralized and regulated to ensure stable supply under high-load
conditions.

A 11.1 V LiPo battery powers the entire system, while buck converters deliver stabilized
voltages to servos, logic, and peripherals.
The propulsion system uses a LEGO NXT Servo Motor, and steering is handled by an
MG995 servo.
Combined with the Ackermann geometry, this setup enables precise turns, smooth
trajectories, and compact maneuvers on narrow tracks.


# 2. Components List

| **Subsystem**          | **Component & Specifications**                                   | **Function**                                      |
| :---------------------- | :-------------------------------------------------------------- | :------------------------------------------------ |
| **Chassis**             | Custom 3D-printed Ackermann chassis (ABS/PLA)                   | Modular frame and structural support              |
| **Steering Servo**      | MG995 (4.8–6 V, metal gears, 24°–144°, center 84°)             | Controls front steering angle                     |
| **Traction Motor**      | LEGO NXT 12 V with encoder wheel                               | Provides drive and speed feedback                 |
| **Main Computer**       | Raspberry Pi 5 (8 GB)                                           | Vision processing and navigation control          |
| **Microcontroller**     | Arduino Nano                                                    | Deterministic PWM and servo control              |
| **Motor Driver**        | DRV8871                                                         | DC motor control                                 |
| **Power Regulation**    | Mini360 Buck Converter (11.1 V → 5 V) + 47 µF capacitor        | Voltage stabilization and noise filtering         |
| **Battery**             | LiPo 11.1 V / 5200 mAh                                          | Main power source                                |
| **Camera**              | Innomaker UVC USB 2.0 (1080p @ 30 fps, 640×480)                | Vision for track and obstacle detection          |
| **Main Switch**         | Toggle switch                                                  | Global power on/off                              |
| **Connection Board**    | Pre-perforated protoboard                                      | Power and signal distribution                    |


# 3. Assembly and Design Notes

```
 Dimensions: 173.89 × 282.71 × 245.56 mm
 Turning radius: ≈ 23 cm
 Weight: ≈ 1.06 kg
```
The Ackermann layout minimizes turning radius for improved cornering.
Power is managed through a custom distribution board to reduce voltage drops and
electrical noise.
A 470 μF capacitor on the servo line stabilizes signals and eliminated early instability
issues.

The Raspberry Pi handles perception and decision-making, while the Arduino executes
deterministic actions — improving responsiveness and system stability.

<img width="568" height="642" alt="image" src="https://github.com/user-attachments/assets/62c42ce7-9512-4b80-81bf-7594419de052" />
<img width="567" height="657" alt="image" src="https://github.com/user-attachments/assets/46db9882-ef4a-44bc-9a54-e136a88612a6" />
<img width="567" height="698" alt="image" src="https://github.com/user-attachments/assets/aa2c619f-72fc-436a-8ad2-8dd7e1440261" />



# 4. Navigation and Control


 Lane detection: LAB color filtering for identifying track lines, pillars, reference
marks, and parking zones.
 Obstacle avoidance: Innomaker UVC USB 2.0 camera detecting colors within a
defined metric range.
 Steering control: PD controller tuned for smooth and responsive turns.
 Serial protocol (Raspberry Pi ↔ Arduino):
 STEER:<angle>
 TURN_ON / TURN_OFF
 STOP

This modular architecture ensures precise lane centering and reliable maneuver execution
in dynamic environments.

# 4.1 Mobility Management

The Mobility Management subsystem defines how the robot interprets, plans, and
executes motion across different stages of the WRO Future Engineers challenge. It
bridges the vision-based control logic and the mechanical actuation system, ensuring that
navigation decisions are translated into precise, stable, and efficient movements.

a) Driving Modes
The robot supports multiple driving modes depending on mission context and control
logic:

 Autonomous Mode: Full vision-based navigation with PD steering control and
obstacle detection.

 Parking Mode: Low-speed precision control for entering or exiting target zones.
 Avoidance Mode: Reactive behavior triggered by obstacle recognition within the
front ROI (Region of Interest).

 Manual Debug Mode: Optional direct control for testing and calibration through
serial commands.

b) Motion Control and Trajectory Planning

Mobility control relies on a PD (Proportional–Derivative) steering system executed on
the Raspberry Pi, while the Arduino Nano handles deterministic PWM and servo
commands.

The motion planner dynamically adjusts steering and velocity based on:
 Lane curvature and relative lateral error.
 
 Obstacle proximity detected through camera color thresholds.
 
 Speed regulation to ensure stable turning radius under Ackermann geometry.
 
This configuration allows smooth transitions between straight segments, turns,
and obstacle-avoidance maneuvers.

c) Dynamic Coordination

The system maintains continuous coordination between sensing, decision, and actuation
layers:

1. Sensing: Camera and optional side sensors capture lane and color information in
    LAB/HSV space.
   
2. Decision: Raspberry Pi computes steering corrections and target velocities.
   
3. Actuation: Arduino converts commands into PWM signals for the DRV
    motor driver and servo steering control.


d) Stability and Correction

To enhance stability, the system integrates:

 Anti-Ackermann geometry, improving cornering behavior at medium-high
speeds.

 Dynamic PD gain adjustment to adapt to changing traction or lighting conditions.
 
 Cooldown intervals between steering updates to prevent oscillations or over-
corrections.

e) Integration and Testing

Mobility functions were validated during multiple track tests under various lighting and
surface conditions. Data from camera feedback and serial communication logs confirmed
consistent behavior in:

 Lane-centering accuracy within ±2° steering deviation.
 
 Obstacle detection response times below 250 ms.
 
 Smooth deceleration and steering recovery after avoidance maneuvers.

# 5. Direction and traction system

The vehicle employs an Anti-Ackermann geometry, configured so the outer wheel
describes a slightly smaller radius than the inner one during a turn.
This results in a larger outer-wheel steering angle, improving high-speed responsiveness
or performance on low-grip surfaces, at the cost of mild lateral slip at low speeds.

<img width="567" height="319" alt="image" src="https://github.com/user-attachments/assets/1e663507-1f43-4603-8b03-da646d2a40e8" />
<img width="568" height="426" alt="image" src="https://github.com/user-attachments/assets/4427b107-f310-49c8-adda-67fe48f31082" />

The wheelbase is 154.12 mm, and the steering angle of each wheel is determined using
the inverse Anti-Ackermann relationship, where the outer wheel turns slightly more than
the inner wheel. This configuration is modeled as:

$$ \theta =  arctan\left(\frac{H}{R}\right) $$

where:

 H = wheelbase (154.12 mm)
 
 R = desired turning radius (tunable parameter)

This Anti-Ackermann setup improves high-speed responsiveness and cornering stability
on low-friction surfaces, at the cost of slightly increased lateral slip at low speeds.

The steering assembly is fully 3D-printed in PLA.
The MG995 servo provides the necessary torque and precision for stable control.

Control modes:


 Fixed-angle turn: servo reaches a target angle according to navigation logic.
 Implemented in Arduino IDE + Python, relying purely on visual feedback (no
IMU sensors).


# 6. Traction and Electronics System

 Motor: LEGO NXT 12 V controlled via DRV8871 PWM.
 Power: 3S 11.1 V / 5200 mAh LiPo battery.
 Regulation: Three Mini360 modules for separate rails (motor, servo, logic).
 Architecture: Single-motor Ackermann drive without mechanical or electronic
differential — reduces weight and complexity.

# 7. Mechanical Design and Fabrication
<img width="567" height="425" alt="image" src="https://github.com/user-attachments/assets/5cfb79ce-0637-463d-b99a-c7bb4042c503" />
<img width="567" height="425" alt="image" src="https://github.com/user-attachments/assets/fe1c1784-9401-4d10-829d-23021f1c30a8" />


 Full CAD design of Ackermann system in PLA.
 Original LEGO NXT transmission axle used.
 Optimized for low friction without extra bearings.
 Custom 3D wheels adapted to MG995 servo.
 All CAD files available in 3D_Models/current_models.

This approach yields a strong, lightweight, functional chassis that meets competition
constraints and ensures compatibility with standard components.

# 8. 3D Part Design and Manufacturing Process
![WhatsApp Image 2025-11-09 at 22 01 40_abea0f08](https://github.com/user-attachments/assets/67f3d0dd-5303-4523-8c66-541120561b49)

8.1 Introduction

A core pillar of Team Rulo Bot development has been the design and fabrication of 3D-
printed parts.
The goal was not only a functional robot but also a light, modular, rigid, and easily
serviceable platform for WRO Future Engineers 2025.

Most structural and support components were custom-designed and manufactured by the
team, allowing precise fit, rapid iteration, and hardware adaptability.


8.2 Designed Parts

 Modular upper and lower bases
 DC motor mount (LEGO)
 Steering servo mount
 Adjustable camera bracket
 Front and rear wheel hubs
 Drive shafts and Raspberry Pi 5 mount
 Decorative and support braces

8.3 Tools and Materials

For the CAD design, we used SolidWorks (latest version). The printing process was
carried out using a Bambu Lab X1 Carbon, which allowed us to achieve high-resolution
parts, excellent repeatability, and short manufacturing times.

CAD: SolidWorks (latest version)

Printer: Bambu Lab X1 Carbon (high resolution & repeatability)

Material: PLA (main), ABS tested for front hubs but discarded due to minor wear benefit.

Infill: ≈ 10 % for non-structural parts; localized reinforcement in critical zones.

Layer height and orientation: optimized for expected loads and assembly ease.

Export: STL format for native slicer processing.

8.4 Step-by-Step Design Process

a) Identification of Requirements and Base Geometry

Each component began with a geometric analysis of the Ackermann platform and the
internal distribution of components. The evaluation included:

 Available space.

 Possible mechanical interferences.

 Anchor points and load directions.

Modularity was a key design criterion — the goal was to build a robot that could be easily
disassembled and upgraded without compromising structural rigidity.


b) Sketching and CAD Design

Once component positions were defined, initial sketches were created in SolidWorks,
followed by the development of the final 3D models. Each design was checked for:

 Relative dimensions between subsystems.

 Areas with potential stress concentrations.

 Compatibility with standard fasteners and custom wheel hubs.

The Ackermann chassis, fully designed by the team, was optimized to minimize
vibrations, maximize structural stiffness, and maintain an efficient turning radius.

c) Fabrication and Prototyping

All parts were 3D-printed in PLA. Although ABS was tested for some critical
components, the improvement did not justify its use over PLA, which provided better
dimensional stability and shorter print times.

The printing strategy prioritized:

 Low infill density in non-structural parts to reduce weight.

 Reinforcement and strategic print orientation for load-bearing components
(mounts, hubs, servo holder).

 Assemblies without complex post-processing requirements.

<img width="567" height="353" alt="image" src="https://github.com/user-attachments/assets/cfe18343-d1b0-4483-a6c5-51fe65c5f20f" />

d) Assembly and Validation

The printed parts were assembled according to the digital design, maintaining the
modularity between the two main platforms:


 Lower platform: LEGO motor, battery, voltage regulators, and main wiring.
 Upper platform: Raspberry Pi, Arduino Nano, camera mount, motor driver, and
auxiliary electronic components.

The mechanical design was directly integrated with the electrical and control layout,
reducing interference and optimizing maintenance accessibility.

e) Joining and Reinforcement Elements

The connection between the robot’s upper and lower platforms is achieved using four M
metal standoffs (50 mm length), ensuring precise spacing and high structural rigidity.

These standoffs maintain vertical alignment, improve access to internal wiring, and
contribute to center-of-gravity balance.
In 3D-printed parts exposed to repeated assembly and disassembly, M3 heat-set threaded
inserts were embedded to reinforce the joints and prevent PLA material wear.

This approach combines mechanical strength, modularity, and ease of maintenance —
essential features for quick servicing during testing and competition.

# 9. Evolution of Key Components
<img width="567" height="100" alt="image" src="https://github.com/user-attachments/assets/e3e91317-e7c0-4971-a84f-da6a615befbd" />


9.1. Camera Mount

The camera mount was one of the most iterative elements of the design — four versions
were developed before reaching the final configuration.

 Initial issues: uncertainty regarding the camera model and whether it should be
fixed or adjustable.
 Intermediate iterations: prototypes with limited motion, low rigidity, or inefficient
geometry.
 Final version: a mount with an adjustable angle and higher structural rigidity,
improving image stability and visual calibration.


9.2. Ackermann Chassis

The chassis is fully 3D-printed.
To achieve good dynamic stability, the team implemented:


 Increased infill density at load-bearing points.
 Calculation of the maximum turning area without interference or flexing.
 A compact structure to minimize vibration and maintain stiffness.

9.3. Hubs and Axles

The front wheel hubs were initially manufactured in ABS to reduce wear but were later
replaced with PLA, as the improvement was not significant.
The axles and couplings were designed to fit perfectly into the steering system without
the need for custom gears, ensuring smooth motion and compatibility.

# 10. Physical Integration and Architecture


 Modularity: The robot is built around two main platforms (upper and lower).
 Weight distribution: The battery and motor are located on the lower platform to
maintain a low center of gravity.
 Wiring and power layout: Organized according to the SolidWorks digital model,
minimizing interference and cable entanglement.
 Accessibility: Each module can be removed independently without requiring full
disassembly of the robot.

# 11. Conclusion

The 3D design and fabrication process has been a core element in the construction of Rulo
Bot. Thanks to this strategy:


 We reduced overall weight while increasing structural rigidity.
 We improved modularity and maintainability across all subsystems.
 We developed a mechanically customized robot perfectly aligned with the
electronic and control architecture.

The knowledge and experience gained through this process will enable us to iterate and
improve more efficiently in future stages of development and competition.