/******************************************************************************

 @file       urfc.c

 @brief User configurable variables for the Micro BLE Stack Radio.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2009-2017, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "urfc.h"
#include <board.h>

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

/* # of TX power entries in the table */
#define UB_NUM_TX_POWER_VALUE (sizeof(ubTxPowerVal) / sizeof(ubTxPowerVal_t))

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/* RF patch function pointers */
const RF_Mode ubRfMode =
{
  .rfMode      = RF_MODE_BLE,
#if defined(CC26XX_R2)
  .cpePatchFxn = 0,
  .mcePatchFxn = 0,
  .rfePatchFxn = 0,
#else /* !CC26XX_R2 */
  .cpePatchFxn = &rf_patch_cpe_ble,
  .mcePatchFxn = 0,
  .rfePatchFxn = &rf_patch_rfe_ble,
#endif /* CC26XX_R2 */
};

uint32 ubRfRegOverride[] = {
#if defined(CC26XX_R2)
  // RFC_RFE:SPARE0. Select R1-style gain table
  HW_REG_OVERRIDE(0x6084, 0x05F8),
  0x04280243, // add 10us to RF SYNTH calibration
#elif defined(CC26XX)
  #if defined(CC2650EM_7ID) || defined(CC2650EM_5XD) || defined(CC2650M5A)
  0x00001007,
  0x00354038,
  0x4001402D,
  0x00608402,
  0x4001405D,
  0x1801F800,
  0x000784A3,
  0xA47E0583,
  0xEAE00603,
  0x00010623,
  0x02010403,
  0x40014035,
  0x177F0408,
  0x38000463,
  0x00456088,
  0x013800C3,
  0x036052AC,
  0x01AD02A3,
  0x01680263,
  #elif defined(CC2650EM_4XS) || defined(CC2650EM_4IS)
  0x00001007,
  0x00354038,
  0x4001402D,
  0x00608402,
  0x4001405D,
  0x1801F800,
  0x000784A3,
  0xA47E0583,
  0xEAE00603,
  0x00010623,
  0x02010403,
  0x40014035,
  0x177F0408,
  0x38000463,
  0x000288A3,
  0x00456088,
  0x013800C3,
  0x036052AC,
  0x01AD02A3,
  0x01680263,
  #else /* unknown device package */
    #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#elif defined(CC13XX)
  #if defined(CC1350LP_7XD) || defined(CC2650EM_7ID)
  0x00001007,
  0x849f0002,
  0xc7440002,
  0x00344038,
  0x00456088,
  0x05fd6084,
  0x7f004020,
  0x00404064,
  0x4001405d,
  0x18000000,
  0x013800c3,
  0x000784a3,
  0xb1070503,
  0x05330523,
  0xa47e0583,
  0xeae00603,
  0x00010623,
  0x00038883,
  0x00f388a3,
  0x04b00243,
  #elif defined(CC1350STK_7XS)
  // override_use_patch_ble_1mbps.xml
  // PHY: Use MCE ROM, RFE RAM patch
  MCE_RFE_OVERRIDE(0,0,0,1,0,0),
  // override_synth_ble_1mbps.xml
  // Synth: Set recommended RTRIM to 4
  HW_REG_OVERRIDE(0x4038,0x0034),
  // Synth: Set Fref to 3.43 MHz
  (uint32_t)0x000784A3,
  // Synth: Configure fine calibration setting
  HW_REG_OVERRIDE(0x4020,0x7F00),
  // Synth: Configure fine calibration setting
  HW_REG_OVERRIDE(0x4064,0x0040),
  // Synth: Configure fine calibration setting
  (uint32_t)0xB1070503,
  // Synth: Configure fine calibration setting
  (uint32_t)0x05330523,
  // Synth: Set loop bandwidth after lock to 80 kHz
  (uint32_t)0xA47E0583,
  // Synth: Set loop bandwidth after lock to 80 kHz
  (uint32_t)0xEAE00603,
  // Synth: Set loop bandwidth after lock to 80 kHz
  (uint32_t)0x00010623,
  // Synth: Configure PLL bias
  HW32_ARRAY_OVERRIDE(0x405C,1),
  // Synth: Configure PLL bias
  (uint32_t)0x18000000,
  // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
  ADI_REG_OVERRIDE(1,4,0x9F),
  // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
  ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
  // override_phy_ble_1mbps.xml
  // Tx: Configure symbol shape for BLE frequency deviation requirements
  (uint32_t)0x013800C3,
  // Rx: Configure AGC reference level
  HW_REG_OVERRIDE(0x6088, 0x0045),
  // Rx: Configure AGC gain level
  HW_REG_OVERRIDE(0x6084, 0x05FD),
  // Rx: Configure LNA bias current trim offset
  (uint32_t)0x00038883,
  // override_frontend_xd.xml
  // Rx: Set RSSI offset to adjust reported RSSI by +13 dB
  (uint32_t)0x00F388A3,

  ADI_HALFREG_OVERRIDE(0, 16, 0x7, 1),
  #else /* unknown device package */
    #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#else /* unknown platform */
  #error "MICRO RF CONFIG BUILD ERROR: Unknown platform!"
#endif

#ifdef CACHE_AS_RAM
  0x00018063,
#endif /* CACHE_AS_RAM */

  0xFFFFFFFF
};

const ubTxPowerVal_t ubTxPowerVal[] = {
#if defined(CC26XX)
  #if defined(CC2650EM_7ID) || defined(CC2650EM_5XD) || defined(CC2650M5A)
/* Tx Power Values (dBm, TC, GC, IB) */
  { TX_POWER_MINUS_21_DBM, TX_POUT( 0x07, 3, 0x0C ) },
  { TX_POWER_MINUS_18_DBM, TX_POUT( 0x09, 3, 0x0C ) },
  { TX_POWER_MINUS_15_DBM, TX_POUT( 0x0B, 3, 0x0C ) },
  { TX_POWER_MINUS_12_DBM, TX_POUT( 0x0B, 1, 0x14 ) },
  { TX_POWER_MINUS_9_DBM,  TX_POUT( 0x0E, 1, 0x19 ) },
  { TX_POWER_MINUS_6_DBM,  TX_POUT( 0x12, 1, 0x1D ) },
  { TX_POWER_MINUS_3_DBM,  TX_POUT( 0x18, 1, 0x25 ) },
  { TX_POWER_0_DBM,        TX_POUT( 0x21, 1, 0x31 ) },
  { TX_POWER_1_DBM,        TX_POUT( 0x14, 0, 0x42 ) },
  { TX_POWER_2_DBM,        TX_POUT( 0x18, 0, 0x4E ) },
  { TX_POWER_3_DBM,        TX_POUT( 0x1C, 0, 0x5A ) },
  { TX_POWER_4_DBM,        TX_POUT( 0x24, 0, 0x93 ) },
  { TX_POWER_5_DBM,        TX_POUT( 0x30, 0, 0x93 ) },
  #elif defined(CC2650EM_4XS) || defined(CC2650EM_4IS)
  { TX_POWER_MINUS_21_DBM, TX_POUT( 0x07, 3, 0x0C ) },
  { TX_POWER_MINUS_18_DBM, TX_POUT( 0x09, 3, 0x10 ) },
  { TX_POWER_MINUS_15_DBM, TX_POUT( 0x0B, 3, 0x14 ) },
  { TX_POWER_MINUS_12_DBM, TX_POUT( 0x0E, 3, 0x14 ) },
  { TX_POWER_MINUS_9_DBM,  TX_POUT( 0x0F, 1, 0x21 ) },
  { TX_POWER_MINUS_6_DBM,  TX_POUT( 0x14, 1, 0x29 ) },
  { TX_POWER_MINUS_3_DBM,  TX_POUT( 0x1C, 1, 0x35 ) },
  { TX_POWER_0_DBM,        TX_POUT( 0x2C, 1, 0x56 ) },
  { TX_POWER_1_DBM,        TX_POUT( 0x1F, 0, 0x6A ) },
  { TX_POWER_2_DBM,        TX_POUT( 0x29, 0, 0x9C ) },
  #elif defined(CC2640R2EM_CXS)
  { TX_POWER_MINUS_21_DBM, TX_POUT( 0xC3, 3, 0x0C ) },
  { TX_POWER_MINUS_18_DBM, TX_POUT( 0xC5, 3, 0x10 ) },
  { TX_POWER_MINUS_15_DBM, TX_POUT( 0xC7, 3, 0x10 ) },
  { TX_POWER_MINUS_12_DBM, TX_POUT( 0xC9, 3, 0x10 ) },
  { TX_POWER_MINUS_9_DBM,  TX_POUT( 0xCC, 3, 0x19 ) },
  { TX_POWER_MINUS_6_DBM,  TX_POUT( 0x4E, 1, 0x25 ) },
  { TX_POWER_MINUS_3_DBM,  TX_POUT( 0x54, 1, 0x2D ) },
  { TX_POWER_0_DBM,        TX_POUT( 0x10, 0, 0x46 ) },
  { TX_POWER_1_DBM,        TX_POUT( 0x16, 0, 0x62 ) },
  { TX_POWER_2_DBM,        TX_POUT( 0x25, 0, 0xBC ) },
  #else /* unknown device package */
    #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#elif defined(CC13XX)
  #if defined(CC1350LP_7XD) || defined(CC2650EM_7ID)
  { TX_POWER_MINUS_21_DBM, 0x0DC8 },
  { TX_POWER_MINUS_18_DBM, 0x0DCB },
  { TX_POWER_MINUS_15_DBM, 0x15CE },
  { TX_POWER_MINUS_12_DBM, 0x19D4 },
  { TX_POWER_MINUS_9_DBM,  0x1DDA },
  { TX_POWER_MINUS_6_DBM,  0x25E3 },
  { TX_POWER_MINUS_3_DBM,  0x2DEF },
  { TX_POWER_0_DBM,        0x5B29 },
  { TX_POWER_1_DBM,        0x6321 },
  { TX_POWER_2_DBM,        0x6F26 },
  { TX_POWER_3_DBM,        0x7F2C },
  { TX_POWER_4_DBM,        0x7734 },
  { TX_POWER_5_DBM,        0x5F3C },
  #elif defined(CC1350STK_7XS)
  { TX_POWER_MINUS_21_DBM, 0x0DC8 },
  { TX_POWER_MINUS_18_DBM, 0x0DCB },
  { TX_POWER_MINUS_15_DBM, 0x15CE },
  { TX_POWER_MINUS_12_DBM, 0x19D4 },
  { TX_POWER_MINUS_9_DBM,  0x1DDA },
  { TX_POWER_MINUS_6_DBM,  0x25E3 },
  { TX_POWER_MINUS_3_DBM,  0x2DEF },
  { TX_POWER_0_DBM,        0x5B29 },
  { TX_POWER_1_DBM,        0x6321 },
  { TX_POWER_2_DBM,        0x6F26 },
  { TX_POWER_3_DBM,        0x7F2C },
  { TX_POWER_4_DBM,        0x7734 },
  { TX_POWER_5_DBM,        0x5F3C },
  #else /* unknown device package */
    #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#else /* unknown platform */
  #error "MICRO RF CONFIG BUILD ERROR: Unknown platform!"
#endif
};

/* RF Frontend Mode and Bias */
const uint8 ubFeModeBias =
#if defined(CC26XX)
  #if defined(CC2650EM_7ID)
  RF_FE_DIFFERENTIAL | RF_FE_INT_BIAS;
  #elif defined(CC2650EM_5XD) || defined(CC2650EM_4XD) || defined(CC2650M5A)
  RF_FE_DIFFERENTIAL | RF_FE_EXT_BIAS;
  #elif defined(CC2650EM_4XS)
  RF_FE_SINGLE_ENDED_RFP | RF_FE_EXT_BIAS;
  #elif defined(CC2640R2EM_CXS)
  RF_FE_SINGLE_ENDED_RFN | RF_FE_EXT_BIAS;
  #else /* unknown device package */
    #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#elif defined(CC13XX)
  #if defined(CC2650EM_7ID)
  /* TEMP : Proper values for CC1350DK_7XD */
  RF_FE_DIFFERENTIAL | RF_FE_EXT_BIAS;
  #elif defined(CC1350EM_7ID)
  RF_FE_DIFFERENTIAL | RF_FE_INT_BIAS;
  #elif defined(CC1350LP_7XD)
  RF_FE_DIFFERENTIAL | RF_FE_EXT_BIAS;
  #elif defined(CC1350STK_7XS)
  RF_FE_SINGLE_ENDED_RFP | RF_FE_EXT_BIAS;
  #else /* unknown device package */
  #error "MICRO RF CONFIG BUILD ERROR: Unknown package type!"
  #endif
#else /* unknown platform */
  #error "MICRO RF CONFIG BUILD ERROR: Unknown platform!"
#endif

/* Tx Power Table */
const ubTxPowerTable_t ubTxPowerTable = { ubTxPowerVal, UB_NUM_TX_POWER_VALUE };

/******************************************************************************
 */

