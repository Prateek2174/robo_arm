# B-G431B-ESC1 board facts (from UM2516 Rev 4, read by arm-pwm session 2026-11-11)

Source: ST UM2516 Rev 4, sections 1, 5.3-5.6, Tables 4-5. st.com timed out, so read from a distributor mirror of the same ST PDF.

## Supply
- Input: LiPo 3S-6S on J5/J6 ("min 3S - max 6S"). 6S full charge = 25.2 V.
- UM2516 gives NO absolute-maximum bus voltage. MOSFETs are 60 V (STL180N6F7), L6387 driver is a 600 V part, so neither is the limit. The real ceiling depends on capacitor and regulator ratings on the MB1419 schematic, not checked yet.
- Treat 25.2 V as the working maximum; software overvoltage threshold just above it; avoid regen into an open contactor/BMS.
- VBUS sense: PA0 (divider ratio is on the schematic, not in UM2516).
- Board has OCP/OVP hardware and an NTC (temperature feedback on PB14).

## Encoder / Hall pads (J8), the open question from the main session
- J8 = 5 V, GND, and three signals: PB6 = A+/H1, PB7 = B+/H2, PB8 = Z+/H3.
- No SPI is available on PB6-PB8 (SPI1 pins PA5-7/PB3-5 are used by op-amps, BEMF, UART). Confirms the concern: the MT6701 can't use hardware SPI/SSI on J8.
- Realistic options (verify alternate functions in the STM32G431 datasheet DS12589 AF table):
  1. ABZ into TIM4 encoder mode (PB6 TIM4_CH1, PB7 TIM4_CH2, PB8 index). No absolute angle at boot: needs an alignment step every power-up, or an index search.
  2. I2C1 on PB7/PB8 for the 14-bit absolute angle. Slow (tens of us per read), so fine for slow loops/boot, marginal at 20 kHz.
  3. Bit-banged SSI on PB6-PB8 GPIO. J8 is a "Hall or encoder" input circuit, so check the schematic for pull-ups/RC filters that would limit clock speed.
  4. Hardware SPI1 on the J3 pads (PB3 SCK, PB4 MISO) at the cost of USART2, which is likely the ST-LINK virtual COM port. Needs checking.
- Affects: encoder_mt6701 (#4), calibration (#5, offset handling at boot).

## TIM1 power stage pins
- CH1 PA8 / CH1N PC13, CH2 PA9 / CH2N PA12, CH3 PA10 / CH3N PB15.
- Gate driver: L6387(E), outputs in phase with inputs, interlock (both outputs can't be high together) but NO programmable dead time. Propagation ~110 ns on / 105 ns off, rise 50 ns / fall 30 ns at 1 nF (datasheet AC table). TIM1 must supply the dead time.
- High side is bootstrapped: duty can't sit at 100%; low side must turn on regularly.

## Current sensing
- 3-shunt, internal op-amps: phase 1 on PA1(+)/PA3(-), output PA2 (OP1_OUT) [corrected 2026-11-17 from UM2516 Rev 4 Table 4: PA3 = Curr_fdbk1_OPAmp-, PA4 = BEMF1]; phase 2 PA7(+)/PA5(-), output PA6 (OP2_OUT); phase 3 PB0(+)/PB2(-), OPAMP3 internal. Shunt value and gain are on the schematic, not in UM2516.

## Other
- HSE crystal 8 MHz (PF0/PF1). 170 MHz needs PLL: /M, xN, /R.
- CAN: onboard transceiver, PA11 RX / PB9 TX, CAN_SHDN on PC11, firmware-switchable 120 ohm termination on PC14 (CAN_TERM; which level turns it on is in UM2516 Table 3, whose layout didn't survive text extraction, so read the PDF before relying on it).
- USART2 PB3 TX / PB4 RX on J3; PWM input PA15; button PC10; status LED PC6; pot PB12.
