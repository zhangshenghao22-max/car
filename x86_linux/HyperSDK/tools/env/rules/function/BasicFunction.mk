#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




ifeq ($(origin file_is_exist),undefined)
# file_is_exist
# $(1) file
# return empty if exist.
define file_is_exist
$(shell ls $(1) > /dev/null 2>&1;echo $$? | grep -v 0)
endef
endif



ifeq ($(origin rule_include),undefined)
# rule_include
# $(1) rule file path
define rule_include
$(if $(call file_is_exist,$(1)),$(error Rule File "$(1)" Not Exist !!!),include $(1))
endef
endif

#$(eval $(call rule_include,xxxx.mk))



# newline
define newline

endef



# is_in_docker
# return : empty if in docker
define is_in_docker
$(strip $(shell echo -n `[ ! -f /.dockerenv ]` $$? | grep -v 1))
endef



