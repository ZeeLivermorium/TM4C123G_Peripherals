# M0PWM0

M0PWM0 uses PB6, it has some key set up for PWM as below

- PWM0_0_CTL_R:

        PWM0_0_CTL_R &= ~PWM_0_CTL_MODE;          // re-loading down-counting mode
    
    `PWM0_0_CTL_R` is M0PWM0 Control Register, `PWM0_0_CTL_R` is declared in [tm4c123gh6pm.h line 742](../../tm4c123gh6pm.h#L742):

        #define PWM0_0_CTL_R            (*((volatile uint32_t *)0x40028040))
    
    We use `PWM_0_CTL_MODE` to modify its `MODE` bit, it is defined in [tm4c123gh6pm.h line 3457](../../tm4c123gh6pm.h#L3457).
    
        #define PWM_0_CTL_MODE          0x00000002  // Counter Mode
    
    The complete bit field definitions for PWM0_0_CTL_R are in [tm4c123gh6pm.h between line 3430 to line 3458](../../tm4c123gh6pm.h#L3430-L3458).

- PWM0_0_GENA_R:

        PWM0_0_GENA_R |= PWM_0_GENA_ACTLOAD_ZERO; // PB6 goes low on LOAD
        PWM0_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ONE; // PB6 goes high on CMPA down
    
- PWM0_0_LOAD_R:

        PWM0_0_LOAD_R = period - 1;               // cycles needed to count down to 0
    
- PWM0_0_CMPA_R:

        PWM0_0_CMPA_R = duty - 1;                 // count value when output rises
    
- PWM0_0_CTL_R:
    
        PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;         // start PWM0
    
- PWM0_ENABLE_R:
    
        PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;       // enable M0PWM0/PB6
