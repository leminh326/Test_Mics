/*
 * winky_debug.h
 *
 *  Created on: 21 janv. 2020
 *      Author: clement
 */

#ifndef WINKY_DEBUG_H_
#define WINKY_DEBUG_H_

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------
#include "app_conf.h"
// --------------------------------------------------------------------------------------------------------------------
// ----- public constant macros
// --------------------------------------------------------------------------------------------------------------------
const char *DbgTraceGetFileName(const char *fullpath);

#if (CFG_WINKY_TRACE != 0)
#define APP_DBG_MSG(...)                 do{printf("\r\n [%s][%s][%d] ", DbgTraceGetFileName(__FILE__),__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0);
#else
#define APP_DBG_MSG(...)
#endif

// --------------------------------------------------------------------------------------------------------------------
// ----- public function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------

void winky_debug_init();

#endif /* WINKY_DEBUG_H_ */
