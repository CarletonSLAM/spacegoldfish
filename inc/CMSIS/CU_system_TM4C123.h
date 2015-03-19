/* CU **********************************
	File:   CU_system_TM4C123.c	Version 1.0

	This file is a modified version of system_TM4C129.c distributed with
	the Tiva C Series software pack. File modified for use in the Carleton
	SLAM sessions. All modification are documented in comments like these
	that have "CU" clearly indicated in the comment block.
	
	Revision History:
		July 14, 2014:	version 1.0: convert from TM4C129 to TM4C123	
	
CU  */

// CU  (mostly) ignore original documentation header
/**************************************************************************//**
 * @file     system_LM4F.h
 * @brief    CMSIS Cortex-M4 Device Peripheral Access Layer Header File for
 *           TI Tiva TM4C123 Class Devices
 * @version  V3.1
 * @date     15. May 2013
 *
 * @note
 * Copyright (C) 2010-2011 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef CU_SYSTEM_TM4C123_H
#define CU_SYSTEM_TM4C123_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* CU_SYSTEM_TM4C123_H */
