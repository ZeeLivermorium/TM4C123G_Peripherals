# Pulse Width Modulation

| Name | Pin Assignment |
| :---: | :---: |
| M0PWM0 | PB6 |
| M0PWM1 | PB7 |
| M0PWM2 | PB4 |
| M0PWM3 | PB5 |
| M0PWM4 | PE4 |
| M0PWM5 | PE5 |
| M0PWM6 | PC4, PD0 |
| M0PWM7 | PC5, PD1 |
| M1PWM0 | PD0 |
| M1PWM1 | PD1 |
| M1PWM2 | PA6, PE4 |
| M1PWM3 | PA7, PE5 |
| M1PWM4 | PF0 |
| M1PWM5 | PF1 |
| M1PWM6 | PF2 |
| M1PWM7 | PF3 |




Some key set up for PWM as below:

PWM0_0_CTL_R &= ~PWM_0_CTL_MODE;          // re-loading down-counting mode

`PWM0_0_CTL_R` is M0PWM0 Conttrol Register, here we set the `MODE` bit to 0 to enable Count-Down mode.
`PWM0_0_CTL_R` is declared in [tm4c123gh6pm.h](../../tm4c123gh6pm.h) line 742:

#define PWM0_0_CTL_R            (*((volatile uint32_t *)0x40028040))

Here we use `PWM_0_CTL_MODE` , it is defined in line 3457 in [tm4c123gh6pm.h](../../tm4c123gh6pm.h).

#define PWM_0_CTL_MODE          0x00000002  // Counter Mode

The complete bit field definitions for PWM0_0_CTL_R are between line 3430 to line 3458 in [tm4c123gh6pm.h](../../tm4c123gh6pm.h).
To learn more about PWM0_0_CTL_R, please refer to [TM4C123GH6PM Data Sheet](http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf) page 1266.


PWM0_0_GENA_R |= PWM_0_GENA_ACTLOAD_ZERO; // PB6 goes low on LOAD
PWM0_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ONE; // PB6 goes high on CMPA down


PWM0_0_LOAD_R = period - 1;               // cycles needed to count down to 0


PWM0_0_CMPA_R = duty - 1;                 // count value when output rises


PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;         // start PWM0


PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;       // enable M0PWM0/PB6
