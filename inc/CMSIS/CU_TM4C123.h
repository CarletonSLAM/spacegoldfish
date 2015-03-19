/* CU **********************************
	File:   CU_TM4C123.h	Version 1.0

	This file is a modified version of TM4C123.h distributed with
	the Tiva C Series software pack. File modified for use in the Carleton
	SLAM sessions. All modification are documented in comments like these
	that have "CU" clearly indicated in the comment block.
	
	Revision History:
		July 14, 2014:	version 1.0: convert from TM4C123.h to CU_TM4C123.h	
	
CU  */

// CU old documentation header:
/**************************************************************************//**
 * @file     TM4C123.h
 * @brief    CMSIS Device System Header File for
 *           Texas Instruments Tiva TM4C123 Device Series
 * @version  V10956
 * @date     05. June 2013
 *
 * @note
 *
 ******************************************************************************/

#ifndef CU_TM4C123_H
#define CU_TM4C123_H

/* CU  snip out all but TM4C123GH6Pm.h 
CU */
 
#if defined(TM4C123GH6PM)
  #include "CU_TM4C123GH6PM.h"
#else
#error "TM4C123.h: TM4C123 Device NOT specified"
#endif
#endif /* CU_TM4C123_H */
