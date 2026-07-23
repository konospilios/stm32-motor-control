# Firmware — `main.c`

This is the control firmware for the project, running on the STM32F411RE.

## Important: what this file is (and isn't)

- **Source:** `main.c` is **reconstructed from Appendix H of the report**
  (`../docs/report.pdf`, pages 83–110), which is a printout of the actual `main.c`
  submitted with the project. It is faithful to that printout, line-for-line.
- **It is for reading, not building.** This was an STM32CubeIDE project, so a full
  build also needs the CubeMX project (`.ioc`), the generated HAL headers, and the
  `SinTable` / `CosTable` lookup-table data — none of which survive in the report.
  Treat this as *the control implementation to read*, not a compilable project.
- **Minor artifacts of PDF extraction:** a few very long lines that the printout
  wrapped have approximate whitespace/indentation. No logic was changed.

## Authored control logic vs auto-generated boilerplate

Like any CubeMX/HAL project, `main.c` mixes two kinds of code:

- **Auto-generated boilerplate** (CubeMX): `SystemClock_Config`, and the
  `MX_*_Init` functions (`MX_GPIO_Init`, `MX_DMA_Init`, `MX_ADC1_Init`, the timer
  and SPI init, etc.). These configure the peripherals; they are not the interesting
  part.
- **The team's authored control code** lives in the `USER CODE` sections. The work
  on this project was shared across the six-person group; my own focus was the
  motor modeling and controller design.

## Map of the interesting code (line numbers match `main.c`)

| Lines | What it is |
|---|---|
| **848** | `HAL_TIM_PeriodElapsedCallback` — the **10 kHz control ISR**: reads the encoder, computes & filters speed, then runs the whole FOC loop below. |
| **914** | **Park transform** (abc → d-q), d-axis aligned. |
| **925–968** | **Cascaded controllers** — outer position loop → speed loop → d/q current **PI controllers with anti-windup** and reference saturation. |
| **970–999** | Inner **current controllers** and vector-magnitude saturation. |
| **1001** | **Inverse Park** (d-q → αβ). |
| **1075–1148** | **SVPWM sector selection** — the `switch(N)` that picks the sector and writes the on-times to `TIM1->CCR1/2/3`. |
| **1161 / 1173** | `getSine` / `getCoSine` — **fixed-point sine/cosine lookup** (quarter-wave table). |
| **1187** | `FilterSpeed` — first-order **low-pass** on the speed estimate. |
| **1191** | `readAngleFromMA730` — SPI read of the magnetic Hall-sensor angle (tested, not used in the final setup). |
| **1035–1061** | **UART data logging** — streams the recorded signal arrays + controller gains to a PC for analysis. |

For the theory behind all of this — the motor model, the controller derivations,
and the test results — see [`../docs/report.pdf`](../docs/report.pdf).
