#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




ifeq ($(is_in_docker),)
# is_in_docker
# return : empty if in docker
define is_in_docker
$(shell echo `[ ! -f /.dockerenv ]` $$? | grep -v 1)
endef
endif


# Echo Config

ifeq ($(call is_in_docker),)
ECHO						?= echo -e
else
ECHO						?= echo
endif



# Global Verbose Config

V							?=


# Target Verbose Config
TV							?=


Q							:= @
MAKE						:= make -s
SCONS						:= scons

TQ							:= $(Q)


ifneq ($(V),)

TV							:= 1
Q							:= 
TQ							:= $(Q)

MAKE						:= make
SCONS						:= scons --verbose

else

ifneq ($(TV),)
TQ							:= 
endif

endif


override MAKEFLAGS=



# Target Build Verbose

TARGET_BUILD_INFO_ENABLE	?=

TARGET_BUILD_STATUS_ENABLE	?=

TARGET_STAGE_DONE_ENABLE	?=



GUEST_METHOD_INFO_ENABLE	?=


GUEST_MODULE_INFO_ENABLE	?= 1

GUEST_MODULE_STATUS_ENABLE	?=

GUEST_MODULE_DONE_ENABLE	?=



