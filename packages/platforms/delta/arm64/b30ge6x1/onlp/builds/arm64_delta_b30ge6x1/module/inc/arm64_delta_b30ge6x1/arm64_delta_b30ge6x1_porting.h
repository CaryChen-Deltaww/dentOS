/********************************************************//**
 *
 * @file
 * @brief arm64_delta_b30ge6x1 Porting Macros.
 *
 * @addtogroup arm64_delta_b30ge6x1-porting
 * @{
 *
 ***********************************************************/
#ifndef __ARM64_DELTA_B30GE6X1_PORTING_H__
#define __ARM64_DELTA_B30GE6X1_PORTING_H__


/* <auto.start.portingmacro(ALL).define> */
#if ARM64_DELTA_B30GE6X1_CONFIG_PORTING_INCLUDE_STDLIB_HEADERS == 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <memory.h>
#endif

#ifndef ARM64_DELTA_B30GE6X1_MALLOC
    #if defined(GLOBAL_MALLOC)
        #define ARM64_DELTA_B30GE6X1_MALLOC GLOBAL_MALLOC
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_MALLOC malloc
    #else
        #error The macro ARM64_DELTA_B30GE6X1_MALLOC is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_FREE
    #if defined(GLOBAL_FREE)
        #define ARM64_DELTA_B30GE6X1_FREE GLOBAL_FREE
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_FREE free
    #else
        #error The macro ARM64_DELTA_B30GE6X1_FREE is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_MEMSET
    #if defined(GLOBAL_MEMSET)
        #define ARM64_DELTA_B30GE6X1_MEMSET GLOBAL_MEMSET
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_MEMSET memset
    #else
        #error The macro ARM64_DELTA_B30GE6X1_MEMSET is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_MEMCPY
    #if defined(GLOBAL_MEMCPY)
        #define ARM64_DELTA_B30GE6X1_MEMCPY GLOBAL_MEMCPY
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_MEMCPY memcpy
    #else
        #error The macro ARM64_DELTA_B30GE6X1_MEMCPY is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_VSNPRINTF
    #if defined(GLOBAL_VSNPRINTF)
        #define ARM64_DELTA_B30GE6X1_VSNPRINTF GLOBAL_VSNPRINTF
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_VSNPRINTF vsnprintf
    #else
        #error The macro ARM64_DELTA_B30GE6X1_VSNPRINTF is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_SNPRINTF
    #if defined(GLOBAL_SNPRINTF)
        #define ARM64_DELTA_B30GE6X1_SNPRINTF GLOBAL_SNPRINTF
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_SNPRINTF snprintf
    #else
        #error The macro ARM64_DELTA_B30GE6X1_SNPRINTF is required but cannot be defined.
    #endif
#endif

#ifndef ARM64_DELTA_B30GE6X1_STRLEN
    #if defined(GLOBAL_STRLEN)
        #define ARM64_DELTA_B30GE6X1_STRLEN GLOBAL_STRLEN
    #elif ARM64_DELTA_B30GE6X1_CONFIG_PORTING_STDLIB == 1
        #define ARM64_DELTA_B30GE6X1_STRLEN strlen
    #else
        #error The macro ARM64_DELTA_B30GE6X1_STRLEN is required but cannot be defined.
    #endif
#endif

/* <auto.end.portingmacro(ALL).define> */


#endif /* __ARM64_DELTA_B30GE6X1_PORTING_H__ */
/* @} */
