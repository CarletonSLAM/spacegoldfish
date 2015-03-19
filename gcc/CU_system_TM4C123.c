/* CU **********************************
	File:   CU_system_TM4C123.c	Version 1.0

	This file is a modified version of system_TM4C129.c distributed with
	the Tiva C Series software pack. File modified by Trevor Pearce for use
	in SYSC4906 course. All modification are documented in comments like these
	that have "CU" clearly indicated in the comment block.
	
	Revision History:
		July 14, 2014:	version 1.0: convert from TM4C129 to TM4C123	
	
CU  */

// CU ignore original header
/**************************************************************************//**
 * @file     system_TM4C129.c
 * @brief    CMSIS Device System Source File for
 *           TI TIVA TM4C129 Snowflake Device Series
 * @version  V1.00
 * @date     15. May 2013
 *
 * @note
 *
 *                                                             modified by Keil
 ******************************************************************************/

#include <stdint.h>
// CU  OLD: #include "TM4C129.h"
#include "CU_TM4C123.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __SYSTEM_CLOCK    (16000000ul)


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK; /*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* TODO: Updates required to fully work with TM4C123 series devices */
  SystemCoreClock = __SYSTEM_CLOCK;

}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  /* TODO: Updates required to fully work with TM4C123 series devices */
  #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
  #endif

#ifdef UNALIGNED_SUPPORT_DISABLE
  SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

  SystemCoreClock = __SYSTEM_CLOCK;

}
