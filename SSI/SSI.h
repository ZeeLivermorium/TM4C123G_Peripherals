/*
 * SSI.h
 * ----------
 * Inspired by examples in ValvanoWareTM4C123 by Dr. Jonathan Valvano
 * as well as his book Embedded Systems: Real-Time Interfacing to Arm Cortex-M Microcontrollers
 * You can find ValvanoWareTM4C123 at http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip?dl=1
 * You can find his book at https://www.amazon.com/gp/product/1463590156/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
 * You can find more of his work at http://users.ece.utexas.edu/~valvano/
 * ----------
 * Zee Livermorium
 * Apr 14, 2018
 */

/****************************************************
 *                                                  *
 *             Setting Parameter Types              *
 *                                                  *
 ****************************************************/

typedef enum {
    Regular, Bit_Banging
} SSI_SS_Mode;

typedef enum {
    Master, Slave
} SSI_MS_Mode;

typedef enum {
    MOTO,     // Freescale SPI Frame Format
    TI,       // Synchronous Serial Frame Format
    NMW       // MICROWIRE Frame Format
} SSI_Frame_Select;

typedef enum {
    LSB, MSB
} Bit_Order;

/****************************************************
 *                                                  *
 *                  SSI Functions                   *
 *                                                  *
 ****************************************************/




///**
// * SSI1_F_Init
// * ----------
// * @brief initialize SSI1 on Port F with corresponding setting parameters.
// */
//void SSI1_F_Init (
//                  SSI_SS_Mode SSI_SS_mode,      // SS is trigered regularly or by bit banging (GPIO)
//                  SSI_MS_Mode SSI_ms_mode,      // Master/Slave mode
//                  uint8_t CPSDVSR,              // must be an even number from 2 to 254
//                  uint16_t SCR,                 // SSI Serial Clock Rate, a value from 0 to 255
//                  uint8_t SSI_clk_mode,         // SSI Clock Mode, from 0 to 3
//                  SSI_Frame_Select SSI_FRF,     // SSI Frame format
//                  uint8_t SSI_DS,               // from 4 bit to 16 bit
//                  Bit_Order bit_order           // LSB or MSB
//);
//
///**
// * SSI2_B_Init
// * ----------
// * @brief initialize SSI2 on Port B with corresponding setting parameters.
// */
//void SSI2_B_Init (
//                  SSI_SS_Mode SSI_SS_mode,      // SS is trigered regularly or by bit banging (GPIO)
//                  SSI_MS_Mode SSI_ms_mode,      // Master/Slave mode
//                  uint8_t CPSDVSR,              // must be an even number from 2 to 254
//                  uint16_t SCR,                 // SSI Serial Clock Rate, a value from 0 to 255
//                  uint8_t SSI_clk_mode,         // SSI Clock Mode, from 0 to 3
//                  SSI_Frame_Select SSI_FRF,     // SSI Frame format
//                  uint8_t SSI_DS,               // from 4 bit to 16 bit
//                  Bit_Order bit_order           // LSB or MSB
//);
//
///**
// * SSI3_D_Init
// * ----------
// * @brief initialize SSI3 on Port D with corresponding setting parameters.
// */
//void SSI3_D_Init (
//                  SSI_SS_Mode SSI_SS_mode,      // SS is trigered regularly or by bit banging (GPIO)
//                  SSI_MS_Mode SSI_ms_mode,      // Master/Slave mode
//                  uint8_t CPSDVSR,              // must be an even number from 2 to 254
//                  uint16_t SCR,                 // SSI Serial Clock Rate, a value from 0 to 255
//                  uint8_t SSI_clk_mode,         // SSI Clock Mode, from 0 to 3
//                  SSI_Frame_Select SSI_FRF,     // SSI Frame format
//                  uint8_t SSI_DS,               // from 4 bit to 16 bit
//                  Bit_Order bit_order           // LSB or MSB
//);
//
//
///**
// * SSI2_read
// * ----------
// * @return: date read from another device.
// */
//static uint16_t SSI2_read (void);
//
///**
// * SSI2_write
// * ----------
// * @param  data  byte of data to be written.
// */
//static void SSI2_write(uint16_t data);
//
///**
// * SSI3_read
// * ----------
// * @return: date read from another device.
// */
//static uint16_t SSI3_read (void);
//
///**
// * SSI3_write
// * ----------
// * @param  data  byte of data to be written.
// */
//static void SSI3_write(uint16_t data);


/****************************************************
 *                                                  *
 *                 Helper Functions                 *
 *                                                  *
 ****************************************************/

/**
 * reverseBitOrder
 * ----------
 * Discription: to output in the order of LSB first, we need to reverse all bits.
 */
uint16_t reverseBitOrder(uint16_t data, uint8_t size) {
    uint16_t result = 0;
    uint16_t bitMask = 0x0001;
    
    /* shifting bits */
    for(int i = 0; i < size; i++) {
        if (2 * i + 1 < size) result += (data & bitMask) << (size - 2 * i - 1);
        else if (size == 2 * i + 1) continue;
        else result += (data & bitMask) >> (2 * i + 1 - size);
        bitMask <<= 1;
    }
    
    return result;
}
