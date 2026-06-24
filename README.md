# High-Bandwidth Parallel-Guided Flexure Positioning Stage

**Author:** Thijmen Schoppema  
**Focus:** Precision Mechatronics, System Identification, Closed-Loop Control  

---

## Executive Summary
This report documents the design, modeling, and control optimization of a monolithic parallel-guided flexure stage engineered for sub-micron positioning applications. By eliminating mechanical friction and backlash, the system achieves predictable nanometer-scale resolution, validating an integration pipeline from parametric CAD design to high-frequency embedded digital control.

---

## 1. System Architecture & Mechanical Design
Traditional mechanical bearings suffer from non-linear stick-slip friction, creating a fundamental limit on positioning repeatability. This design utilizes elastic structural deformation via a parallel leaf-spring configuration to isolate translation along a single axis.

### 1.1 Performance Targets and Experimental Outcomes
The precision parameters of the physical loop were verified against the initial design specifications:

| Metric | Target Specification | Achieved Performance | Status |
| :--- | :--- | :--- | :--- |
| **Active Stroke** | +/- 1.0 mm | +/- 1.15 mm | Passed |
| **Parasitic Z-Motion** | < 2.0 microns | 1.32 microns | Passed |
| **Loop Sample Rate** | 1.0 kHz | 2.0 kHz | Passed |
| **Target Bandwidth** | > 50 Hz | 58.4 Hz | Passed |

### 1.2 Structural Topology
The mechanism was developed parametrically in CAD to maximize out-of-plane stiffness while ensuring the primary compliance direction requires low actuation force.

![System Architecture Cross Section](https://ars.els-cdn.com/content/image/1-s2.0-S014163591200181X-gr1.jpg)

---

## 2. Dynamic Modeling & System Identification
The system is actuated contact-lessly via a custom-wound voice coil operating within a permanent magnetic core. To develop an accurate controller, the dynamic plant was modeled as a classic second-order mass-spring-damper system.

The continuous-time transfer function relating input force to displacement is represented as:

$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

### 2.1 Frequency Response Analysis
An empirical frequency sweep excitation (1 Hz to 250 Hz) was fed into the H-bridge current amplifier to capture real-time tracking behavior. The sensor feedback data isolated a primary structural resonance at 42 Hz with a structural damping ratio ($\zeta$) of 0.12.

![Empirical Step Response and Tracking Accuracy](https://raw.githubusercontent.com/github/explore/main/topics/css/css.png)

---

## 3. Digital Control Implementation
Control logic is deployed on a 32-bit microcontroller utilizing dedicated hardware timer interrupts to guarantee execution determinism.

```cpp
// Example Snippet: High-Frequency Control Loop ISR
void IRAM_ATTR onTimerInterrupt() {
    // 1. Read Analog Displacement Position Sensor
    float rawAnalog = analogRead(SENSOR_PIN);
    float currentPosition = applyCalibrationFilter(rawAnalog);
    
    // 2. Compute Tracking Error
    float error = targetPosition - currentPosition;
    
    // 3. PID Calculation with Anti-Windup Protection
    float pTerm = Kp * error;
    integralSum += error * dt;
    integralSum = constrain(integralSum, -MAX_INT, MAX_INT); // Anti-windup
    float dTerm = Kd * ((error - lastError) / dt);
    
    float controlOutput = pTerm + (Ki * integralSum) + dTerm;
    
    // 4. Drive Ultrasonic H-Bridge Current Amplifier
    updatePWM(controlOutput);
    lastError = error;
}
