/*
 * M1PWM2.c
 * Use M1PWM2/PA6 to generate pulse-width modulated outputs.
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
 * Function: void M1PWM2_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M1PWM2/PA6.
 */
void M1PWM2_Init(uint16_t period, uint16_t duty){
    /* PWM Module and Port Set Up */
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;                 // activate PWM Module 1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;               // activate Port A
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0){};    // allow time to finish activating
    
    /* Port A Set Up */
    GPIO_PORTA_AFSEL_R |= 0x40;                            // enable alt funct on PA6
    GPIO_PORTA_PCTL_R &= ~GPIO_PCTL_PA6_M;                 // clear bit fields for PA6
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA6_M1PWM2;             // configure PA6 as M1PWM2
    GPIO_PORTA_AMSEL_R &= ~0x40;                           // disable analog functionality on PA6
    GPIO_PORTA_DEN_R |= 0x40;                              // enable digital I/O on PA6
    
    /* System Control Run-Mode Clock Configuration (RCC) Set Up */
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;                  // use PWM divider
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                  // clear PWM divider field
    SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;                   // configure for /2 divider
    
    /* M1PWM2 Set Up */
    PWM1_1_CTL_R &= ~PWM_1_CTL_MODE;                       // re-loading down-counting mode
    PWM1_1_GENA_R |= PWM_1_GENA_ACTLOAD_ZERO;              // PA6 goes low on LOAD
    PWM1_1_GENA_R |= PWM_1_GENA_ACTCMPAD_ONE;              // PA6 goes high on CMPA down
    PWM1_1_LOAD_R = period - 1;                            // cycles needed to count down to 0
    PWM1_1_CMPA_R = duty - 1;                              // count value when output rises
    PWM1_1_CTL_R |= PWM_1_CTL_ENABLE;                      // enable Generator 0 for PWM Module 1
    PWM1_ENABLE_R |= PWM_ENABLE_PWM2EN;                    // enable M1PWM2 output
}

/*
 * Function: void M1PWM2_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M1PWM2_Set_Duty(uint16_t duty){
    PWM1_1_CMPA_R = duty - 1;                              // count value when output rises
}

