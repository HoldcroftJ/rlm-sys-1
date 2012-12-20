/* -*- C -*- ****************************************************************
 *
 *  System        : 
 *  Module        : 
 *  Object Name   : $RCSfile$
 *  Revision      : $Revision$
 *  Date          : $Date$
 *  Author        : $Author$
 *  Created By    : Russ Magee
 *  Created       : Tue Jun 5 21:42:42 2012
 *  Last Modified : <120605.2219>
 *
 *  Description	
 *
 *  Notes
 *
 *  History
 *	
 ****************************************************************************
 *
 *  Copyright (c) 2012 Russ Magee.
 * 
 *  All Rights Reserved.
 * 
 * This  document  may  not, in  whole  or in  part, be  copied,  photocopied,
 * reproduced,  translated,  or  reduced to any  electronic  medium or machine
 * readable form without prior written consent from Russ Magee.
 *
 ****************************************************************************/

#ifndef __TIMER_1MS_H
#define __TIMER_1MS_H

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

/** Install ms-resolution timer
 * 
 * @param[out] tid - handle for installed timer
 * @param[in] cb - callback function to execute on each tick
 * @param[in] arg - optional argument to callback
 * @return int32_t - TRUE if timer installed OK; FALSE otherwise
 */
int32_t installMsTimer(uint32_t *tid, void* cb, uint32_t arg);

/** Uninstall ms-resolution timer
 *
 * @param[in] tid - timer handle used to install timer
 * @return int32_t - TRUE if timer was uninstalled; FALSE otherwise
 */
int32_t uninstallMsTimer(uint32_t tid);


#if defined(__cplusplus) || defined(c_plusplus)
}
#endif
#endif /* __TIMER_1MS_H */
