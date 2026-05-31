#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#



.PHONY	: all


all	: build


PROJECT_REPO_TYPE			:= usersdk

PROJECT_ROOT				:= $(PWD)

PROJECT_RULE_PATH			:= $(PROJECT_ROOT)/tools/env/rules

HYPERSDK_CONFIG				:= $(PROJECT_ROOT)/.hypersdk.mk



ifeq ($(PROJECT_REPO_TYPE),usersdk)
ifeq ($(call path_is_exist,$(HYPERSDK_CONFIG)),)

include $(HYPERSDK_CONFIG)

else

$(error Cannot Found HyperSDK Config File "$(HYPERSDK_CONFIG)")

endif # ($(call path_is_exist,$(HYPERSDK_CONFIG)),)
endif # ($(PROJECT_REPO_TYPE),usersdk)



include $(PROJECT_RULE_PATH)/HyperRule.mk



