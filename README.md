# INDUSTRIAL CONTROL THEORY TOOLKIT — C++ / ROS2 / ISAAC SIM

---

## STATUS: **UNDER ACTIVE DEVELOPMENT (Original project (Python-based) for academia abandoned in early 2025)* **
**This is a personal, production-grade control theory library.  
Industrial-grade algorithms that actually run real robots and factory lines.  
Everything implemented from scratch in modern C++ — ROS2-ready, Isaac Sim connectable, and 100% auditable.**

---

## WHY THIS EXISTS

> 99% of industrial automation still runs on control theory, not “AI”.  
> This repo is my long-term, no-BS answer to people who don’t understand that.  

> Every core classical and modern control technique — from PID to H-infinity, from Smith predictors to state observers — in clean, modular, provable code.  

> No Python. No fragile scripts. No research prototypes.  
> Everything built for hard deployment, portfolio proof, and maximum technical leverage.

---

## WHAT’S INCLUDED (FULL STACK — NO FILLER)

### 1. PID & Variants  
- P, PI, PD, PID  
- Anti-windup (all types)  
- Gain scheduling & adaptive PID  
- Feedforward, cascade, Smith predictor  
- Disturbance observer, fractional-order PID  

### 2. Frequency-Domain Design  
- Root locus, Bode/Nyquist synthesis  
- Lead/lag, lead–lag, notch filter design  
- Loop shaping, resonance suppression  

### 3. State-Space & Linear Control  
- Full state-space toolkit (cont/discrete)  
- Controllability/observability  
- Pole placement (Ackermann)  
- LQR, LQG, output feedback, servo design  

### 4. Optimal & Robust Control  
- Pontryagin/Bellman dynamic programming  
- Min-time, min-energy, H₂/H∞, QFT, μ-synthesis  
- Bang-bang, LTR, loop transfer recovery  

### 5. Model Predictive Control  
- Linear/Nonlinear MPC, explicit MPC  
- Tube-based, robust and economic MPC  

### 6. Nonlinear & Adaptive Control  
- Feedback/exact linearization  
- Sliding mode (all variants), backstepping  
- Lyapunov/energy-based methods  
- MRAC, STR, gain-scheduled/adaptive pole placement  

### 7. Observers & Estimators  
- Luenberger, reduced-order, Kalman (EKF/UKF/MHE/UIO)  
- Moving horizon, unknown input observer  

### 8. System Identification  
- ARX, ARMAX, NARX  
- RLS, subspace, prediction error models  

### 9. Intelligent/Hybrid 
- Fuzzy logic, fuzzy-PID, ANFIS, NN-based  
- RL-based controllers — implementing -> https://github.com/1Kaustubh122/rlx-core

### 10. Special Topics  
- Disturbance rejection, IMC, ILC, event-triggered, passivity-based, repetitive/preview control

---

## HOW IT’S BUILT

- **Modern C++ (C++20 minimum, zero dynamic allocs in control loops)**
- **CMake, colcon, ROS2 native modules** (rclcpp, real launch files, ROS2 bagging/logging)
- **Isaac Sim USD support** for high-fidelity plant/sensor testing (optional)
- **CI-tested, gtest/unit-tested**; clean separation between core, interface, and simulation
- **Every algorithm modular, plug-and-play for hardware or sim**  
- **No dependencies on Python, Matlab, or toy libraries**

---

## WHO IT’S FOR

- Anyone who wants to see how real automation is actually built, without the academic clutter or “AI” hype
- Engineers needing deployable, auditable, high-performance C++ control code
- Portfolio reviewers: This is what top 1% hands-on automation looks like

