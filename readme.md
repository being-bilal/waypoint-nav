# ASV Navigation System

An autonomous surface vehicle navigation stack built around industrial-grade sensors and a dual-compute architecture.

---

## How It's Wired Together

```
Industrial RTK GPS ──► Jetson Orin Nano  ◄──  VN-100 (IMU/AHRS)
                             │            ◄──  PNI RM3100 (Magnetometer)
                             ▼
                       STM32H755xI
                             │
                           ESCs
```

The Jetson is the brain. It reads all three sensors, runs the navigation logic, and sends thrust commands down to the STM32 which drives the ESCs.

---

## Sensors

**VectorNav VN-100** — IMU and AHRS. Outputs calibrated acceleration, yaw, and yaw rate. It does *not* do GPS/INS fusion — that's the job of the Jetson. What it gives you is clean, high-rate attitude data without having to wrangle raw IMU noise.

**PNI RM3100** — The absolute heading reference. Far less noisy than consumer magnetometers. Mounted away from ESCs and power cables to keep motor interference out. Still needs hard/soft iron calibration done with everything powered on and motors running — don't skip this.

**Industrial RTK GPS** — Ground truth position. Fed directly into the Jetson alongside VN-100 data for sensor fusion.

---

## Position: GPS/INS Fusion on the Jetson

The VN-100 is an AHRS, not an INS — it has no GPS input. So the loosely-coupled GPS/INS Kalman filter runs on the Jetson itself. RTK GPS gives absolute position at ~5–10 Hz, VN-100 fills in the gaps at high rate. GPS corrects IMU drift on every fix. IMU dead-reckons between fixes.

---

## Heading

RM3100 is the primary heading source — absolute magnetic heading after declination correction. VN-100 yaw rate feeds the PID derivative term for smooth damping. Two sensors, two jobs.

---

## Jetson → STM32

Binary packets over UART, validated with CRC on every message:

```
[START] [thrust_left : int16] [thrust_right : int16] [mode] [CRC] [END]
```

STM32 runs a watchdog — if no valid packet arrives within N ms, thrust cuts to zero. The M4 core handles parsing and PWM generation, M7 is free for anything else.

---

## Update Rates

| Source | Rate |
|---|---|
| VN-100 attitude / yaw rate | up to 800 Hz |
| RM3100 heading | 50–100 Hz |
| RTK GPS | 5–10 Hz |
| Navigation loop | 10–50 Hz |