/*
 * M1PWM0.c
 * Use M1PWM0/PD0 to generate pulse-width modulated outputs.
 * ----------
 * Inspired by the PWM example in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Dec 26, 2017
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"   // put tm4c123gh6pm.h in your project folder or change this line

/*
 * Function: void M1PWM0_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M1PWM0/PD0.
 */
void M1PWM0_Init(uint16_t period, uint16_t duty){
    /* PWM Module and Port Set Up */
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;                 // activate PWM Module 1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;               // activate Port D
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0){};    // allow time to finish activating
    
    /* Port D Set Up */
    GPIO_PORTD_AFSEL_R |= 0x01;                            // enable alt funct on PD0
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;                 // clear bit fields for PD0
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_M1PWM0;             // configure PD0 as M1PWM0
    GPIO_PORTD_AMSEL_R &= ~0x01;                           // disable analog functionality on PD0
    GPIO_PORTD_DEN_R |= 0x01;                              // enable digital I/O on PD0
    
    /* System Control Run-Mode Clock Configuration (RCC) Set Up */
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;                  // use PWM divider
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                  // clear PWM divider field
    SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;                   // configure for /2 divider
    
    /* M1PWM0 Set Up */
    PWM1_0_CTL_R &= ~PWM_0_CTL_MODE;                       // re-loading down-counting mode
    PWM1_0_GENA_R |= PWM_0_GENA_ACTLOAD_ZERO;              // PD0 goes low on LOAD
    PWM1_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ONE;              // PD0 goes high on CMPA down
    PWM1_0_LOAD_R = period - 1;                            // cycles needed to count down to 0
    PWM1_0_CMPA_R = duty - 1;                              // count value when output rises
    PWM1_0_CTL_R |= PWM_0_CTL_ENABLE;                      // enable Generator 0 for PWM Module 1
    PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;                    // enable M1PWM0 output
}

/*
 * Function: void M1PWM0_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M1PWM0_Set_Duty(uint16_t duty){
    PWM1_0_CMPA_R = duty - 1;                              // count value when output rises
}

