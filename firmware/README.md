# Firmware — `main.c`

The control firmware running on the STM32F411RE: the 10 kHz Field-Oriented Control
loop with SVPWM commutation, plus the peripheral configuration.

## Map of the control logic

| Lines | What it is |
|---|---|
| **848** | `HAL_TIM_PeriodElapsedCallback` — the **10 kHz control ISR**: reads the encoder, computes and filters speed, then runs the FOC loop below. |
| **914** | **Park transform** (abc → d-q), d-axis aligned. |
| **925–968** | **Cascaded controllers** — outer position loop → speed loop → d/q current **PI controllers with anti-windup** and reference saturation. |
| **970–999** | Inner **current controllers** and vector-magnitude saturation. |
| **1001** | **Inverse Park** (d-q → αβ). |
| **1075–1148** | **SVPWM sector selection** — the `switch(N)` that picks the sector and writes the on-times to `TIM1->CCR1/2/3`. |
| **1161 / 1173** | `getSine` / `getCoSine` — **fixed-point sine/cosine lookup** (quarter-wave table). |
| **1187** | `FilterSpeed` — first-order **low-pass** on the speed estimate. |
| **1191** | `readAngleFromMA730` — SPI read of the magnetic Hall-sensor angle. |
| **1035–1061** | **UART data logging** — streams the recorded signals and controller gains to a PC. |

The peripheral setup (`SystemClock_Config`, the `MX_*_Init` functions) is standard
CubeMX/HAL configuration; the control work lives in the `USER CODE` sections mapped
above.

For the motor model, controller derivations, and test results, see
[`../docs/report.pdf`](../docs/report.pdf).
