# ESC1 current and bus-voltage scaling (from the arm-current-sense session, 2026-11-13)

Sources: NuttX B-G431B-ESC1 board page (values from ST's board config), SimpleFOC forum, ST community. Not yet checked against the MB1419 schematic PDF itself.

| Constant | Value | Note |
|---|---|---|
| Shunt R | 0.003 ohm per phase, 3 shunts, low side | |
| Current-sense gain G | 9.14 (= 64/7), INVERTING | OPAMP PGA set to 16, external network brings it to 9.14 |
| Bias (zero-current voltage) | about mid-rail, NOT hard-coded | measured per channel at boot (offset calibration) |
| ADC | 12-bit, Vref 3.3 V | |
| Amps per count k | 3.3 / (4096 * 9.14 * 0.003) = 0.02937 A/count | I = -(raw - offset) * k |
| Measurable span | about +/-60 A at the ADC; board rated 40 A peak (about 1362 counts from offset) | |
| VBUS divider | 18k / (18k + 169k), ratio 10.39 | VBUS on PA0 = ADC1_IN1 |
| VBUS | V = raw * 3.3 / 4096 * 10.39; 24.0 V = raw 2867; 25.2 V = raw 3011; 34.3 V = full scale | |

- Two ADCs sample two phases at the same instant; the third is reconstructed as -(sum of the other two). ST's firmware switches OPAMP3's internal output dynamically to choose which pair.
- Sample while the low-side FETs are on (center of the low-side window); coordinate trigger timing with `arm-pwm`.
- Pins (UM2516 Rev 4, Table 4, pp.16-17): PA0 VBUS; PA1/PA3 phase1 +/-, PA2 OP1_OUT; PA7/PA5 phase2 +/-, PA6 OP2_OUT; PB0/PB2 phase3 +/- (OPAMP3 output internal); VREF+ = 3V3; PB14 NTC; TIM1 CH1/2/3 = PA8/PA9/PA10, CH1N/2N/3N = PC13/PA12/PB15. PA4/PC4/PB11 are BEMF1/2/3, not current sense.
- UM2516 Rev 4 has NO shunt/gain/bias values and its schematics were removed in Rev 3; R, G and bias must be confirmed from the MB1419 schematic (st.com product page, CAD Resources).
- Consumers: `current_loop` (amps), `faults` (overcurrent, OV/UV from VBUS), `foc_math.clarke()`.
