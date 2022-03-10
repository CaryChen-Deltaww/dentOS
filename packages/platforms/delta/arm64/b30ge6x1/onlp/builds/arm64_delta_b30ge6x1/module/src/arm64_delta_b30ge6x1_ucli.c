/**************************************************************************//**
 *
 *
 *
 *****************************************************************************/
#include <arm64_delta_b30ge6x1/arm64_delta_b30ge6x1_config.h>

#if ARM64_DELTA_B30GE6X1_CONFIG_INCLUDE_UCLI == 1

#include <uCli/ucli.h>
#include <uCli/ucli_argparse.h>
#include <uCli/ucli_handler_macros.h>

static ucli_status_t
arm64_delta_b30ge6x1_ucli_ucli__config__(ucli_context_t* uc)
{
    UCLI_HANDLER_MACRO_MODULE_CONFIG(arm64_delta_b30ge6x1)
}

/* <auto.ucli.handlers.start> */
/******************************************************************************
 *
 * These handler table(s) were autogenerated from the symbols in this
 * source file.
 *
 *****************************************************************************/
static ucli_command_handler_f arm64_delta_b30ge6x1_ucli_ucli_handlers__[] =
{
    arm64_delta_b30ge6x1_ucli_ucli__config__,
    NULL
};
/******************************************************************************/
/* <auto.ucli.handlers.end> */

static ucli_module_t
arm64_delta_b30ge6x1_ucli_module__ =
    {
        "arm64_delta_b30ge6x1_ucli",
        NULL,
        arm64_delta_b30ge6x1_ucli_ucli_handlers__,
        NULL,
        NULL,
    };

ucli_node_t*
arm64_delta_b30ge6x1_ucli_node_create(void)
{
    ucli_node_t* n;
    ucli_module_init(&arm64_delta_b30ge6x1_ucli_module__);
    n = ucli_node_create("arm64_delta_b30ge6x1", NULL, &arm64_delta_b30ge6x1_ucli_module__);
    ucli_node_subnode_add(n, ucli_module_log_node_create("arm64_delta_b30ge6x1"));
    return n;
}

#else
void*
arm64_delta_b30ge6x1_ucli_node_create(void)
{
    return NULL;
}
#endif
