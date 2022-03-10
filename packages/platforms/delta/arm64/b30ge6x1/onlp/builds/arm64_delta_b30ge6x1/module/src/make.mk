############################################################
#
#
#
############################################################

LIBRARY := arm64_delta_b30ge6x1
$(LIBRARY)_SUBDIR := $(dir $(lastword $(MAKEFILE_LIST)))
#$(LIBRARY)_LAST := 1
include $(BUILDER)/lib.mk
