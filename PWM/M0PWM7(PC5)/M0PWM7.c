/*
 * M0PWM7.c
 * Use M0PWM7/PC5 to generate pulse-width modulated outputs.
 * ----------
 * Inspired by the PWM example in ValvanoWare by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find VavanoWare at http://users.ece.utexas.edu/~valvano/arm/downloadmsp432.html
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Dec 21, 2017
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"   // put tm4c1w23gh6pm.h in your project folder or change this line

/*
 * Function: void M0PWM7_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M0PWM7/PC5.
 */
void M0PWM7_Init(uint16_t period, uint16_t duty){
    /* PWM Module and Port Set Up */
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;                 // activate PWM Module 0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;               // activate Port C
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2) == 0){};    // allow time to finish activating
    
    /* Port C Set Up */
    GPIO_PORTC_AFSEL_R |= 0x20;                            // enable alt funct on PC5
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_M;                 // clear bit fields for PC5
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_M0PWM7;             // configure PC5 as M0PWM7
    GPIO_PORTC_AMSEL_R &= ~0x20;                           // disable analog functionality on PC5
    GPIO_PORTC_DEN_R |= 0x20;                              // enable digital I/O on PC5
    
    /* System Control Run-Mode Clock Configuration (RCC) Set Up */
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;                  // use PWM divider
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                  // clear PWM divider field
    SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;                   // configure for /2 divider
    
    /* M0PWM7 Set Up */
    PWM0_3_CTL_R &= ~PWM_3_CTL_MODE;                       // re-loading down-counting mode
    PWM0_3_GENB_R |= PWM_3_GENB_ACTLOAD_ZERO;              // PC5 goes low on LOAD
    PWM0_3_GENB_R |= PWM_3_GENB_ACTCMPBD_ONE;              // PC5 goes high on CMPB down
    PWM0_3_LOAD_R = period - 1;                            // cycles needed to count down to 0
    PWM0_3_CMPB_R = duty - 1;                              // count value when output rises
    PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;                      // enable Generator 2 for PWM Module 0
    PWM0_ENABLE_R |= PWM_ENABLE_PWM7EN;                    // enable M0PWM7 output
}

/*
 * Function: void M0PWM7_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M0PWM7_Set_Duty(uint16_t duty){
    PWM0_3_CMPB_R = duty - 1;                              // count value when output rises
}

