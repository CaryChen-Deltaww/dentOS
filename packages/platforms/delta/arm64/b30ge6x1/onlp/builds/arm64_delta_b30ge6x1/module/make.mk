############################################################
#
#
#
############################################################
THIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
arm64_delta_b30ge6x1_INCLUDES := -I $(THIS_DIR)inc
arm64_delta_b30ge6x1_INTERNAL_INCLUDES := -I $(THIS_DIR)src
arm64_delta_b30ge6x1_DEPENDMODULE_ENTRIES := init:arm64_delta_b30ge6x1 ucli:arm64_delta_b30ge6x1
