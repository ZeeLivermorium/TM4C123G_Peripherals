/*
 * PWM.h
 * ----------
 * Inspired by the PWM example in ValvanoWare by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find VavanoWare at http://users.ece.utexas.edu/~valvano/arm/downloadmsp432.html
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Dec 20, 2017
 */

/****************************************
 *                                      *
 *                M0PWM0                *
 *                                      *
 ****************************************/
/*
 * Function: void M0PWM0_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M0PWM0/PB6.
 */
void M0PWM0_Init(uint16_t period, uint16_t duty);

/*
 * Function: void M0PWM0_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M0PWM0_Set_Duty(uint16_t duty);


/****************************************
 *                                      *
 *                M0PWM1                *
 *                                      *
 ****************************************/
/*
 * Function: void M0PWM1_Init(uint16_t period, uint16_t duty)
 * Parameters:
 *   - period: period of the PWM signal
 *   - duty: duty cycle of the PWM signal
 * Discription: Initialize M0PWM1/PB7.
 */
void M0PWM1_Init(uint16_t period, uint16_t duty);

/*
 * Function: void M0PWM1_Set_Duty(uint16_t duty)
 * Parameters:
 *   - duty: new duty cycle to be set.
 * Discription: Set duty cycle of the PWM signal to a new value.
 */
void M0PWM1_Set_Duty(uint16_t duty);


/****************************************
 *                                      *
 *                M0PWM2                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M0PWM3                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M0PWM4                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M0PWM5                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M0PWM6                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M0PWM7                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM0                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM1                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM2                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM3                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM4                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM5                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM6                *
 *                                      *
 ****************************************/


/****************************************
 *                                      *
 *                M1PWM7                *
 *                                      *
 ****************************************/
