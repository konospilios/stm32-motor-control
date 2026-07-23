# Low-Speed Field-Oriented Control of a Direct-Drive PMSM (STM32 + SVPWM)

Real-time **Field-Oriented Control (FOC)** of a permanent-magnet synchronous motor
(PMSM), commutated with **Space-Vector PWM (SVPWM)** and executed in C on an
**STM32F411RE**. The goal: drive a low-KV PMSM **gearlessly** (direct-drive) at low
speed with high torque — as a candidate replacement for the geared brushed-DC motor
in LEGO's *Powered Up* sets.

> **MSc group project** — 8th semester, Electro-Mechanical System Design, Aalborg
> University (group EMSD2-4.115, Feb–May 2024). Six authors; the work was shared.
> **My focus was the motor modeling and the controller design.** I also know and
> have reviewed the firmware. Full report (60 pages + appendices) is in
> [`docs/report.pdf`](docs/report.pdf).

## What it does

- Runs **FOC + SVPWM** in a 10 kHz control loop on the STM32, taking a low-KV,
  7-pole-pair PMSM (CubeMars GL35) direct-drive — no gearbox.
- Closes **cascaded control loops**: inner d/q current PI controllers, an outer
  speed controller, and an optional position loop, all with **anti-windup**.
- Reads rotor position from an **incremental encoder**, computes and low-pass
  filters speed, and logs signals back to a PC over **UART** for analysis.

## Results

Benchmarked against the motor it aims to replace (LEGO Powered Up Technic Large DC
motor):

- **Higher torque** and **higher efficiency** under load.
- **Faster transient response** and **lower steady-state error** (the DC motor's
  error comes from its high friction / gearing).
- **Smoother low-speed operation** (down to ~1 rad/s) — the payoff of a brushless,
  gearless drive.

Conclusion: FOC + SVPWM on a direct-drive PMSM is a viable, more-efficient
alternative to the geared DC motor for this class of application.

## How it was built

| Stage | What was done |
|---|---|
| **Modeling** | Electrical (3-phase + back-EMF → d-q) and mechanical (friction, inertia) PMSM model; ideal-inverter model; validated against open-loop and closed-loop bench tests. |
| **Control design** | s-plane model, loop shaping, d-q decoupling, discretization, anti-windup; tuned for high inertia across a wide operating range. |
| **Implementation** | FOC + SVPWM in C on the STM32 (HAL); fixed-point sine/cosine lookup; SVPWM sector selection driving the timer compare registers. |
| **Testing** | Mass-on-string load tests, inertia-disk transient tests, low-speed tracking, and the LEGO-motor comparison. |

## Hardware & tools

- **MCU:** STM32F411RE (Nucleo) · **Inverter:** X-NUCLEO-IHM07M1 · **Motor:**
  CubeMars GL35 PMSM (low KV, 7 pole pairs) · **Feedback:** incremental encoder
  (a 2-axis Hall sensor was tested but not used in the final setup).
- **Software:** C / STM32 HAL (STM32CubeIDE, CubeMX, CubeMonitor), MATLAB & Simulink
  (modeling and control design), SolidWorks, LaTeX.

## Repository layout

```
stm32-motor-control/
├── docs/report.pdf     Full project report (60 pages + 10 appendices)
└── firmware/
    ├── main.c          The control firmware (see firmware/README.md)
    └── README.md       What's authored vs auto-generated, and a map of the code
```

> **Note on the code:** `firmware/main.c` is reconstructed from the report's code
> appendix and is provided for reading, not building — see
> [`firmware/README.md`](firmware/README.md) for details.
