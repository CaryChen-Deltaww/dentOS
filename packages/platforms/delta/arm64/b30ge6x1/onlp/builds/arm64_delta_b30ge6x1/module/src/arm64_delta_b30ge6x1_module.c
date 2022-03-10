/**************************************************************************//**
 *
 *
 *
 *****************************************************************************/
#include <arm64_delta_b30ge6x1/arm64_delta_b30ge6x1_config.h>

#include "arm64_delta_b30ge6x1_log.h"

static int
datatypes_init__(void)
{
#define ARM64_DELTA_B30GE6X1_ENUMERATION_ENTRY(_enum_name, _desc)     AIM_DATATYPE_MAP_REGISTER(_enum_name, _enum_name##_map, _desc,                               AIM_LOG_INTERNAL);
#include <arm64_delta_b30ge6x1/arm64_delta_b30ge6x1.x>
    return 0;
}

void __arm64_delta_b30ge6x1_module_init__(void)
{
    AIM_LOG_STRUCT_REGISTER();
    datatypes_init__();
}

int __onlp_platform_version__ = 1;
