#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




###############################################################################
# Environment Check


ifeq ($(PROJECT_ROOT),)
$(error "PROJECT_ROOT has not been defined.")
endif


ifeq ($(PROJECT_RULE_PATH),)
$(error "PROJECT_RULE_PATH has not been defined.")
endif


ifeq ($(PROJECT_REPO_TYPE),)
$(error "PROJECT_REPO_TYPE has not been defined.")
endif




###############################################################################
# Function Rules

# file_is_exist
# $(1) file
# return empty if exist.
define file_is_exist
$(shell ls $(1) > /dev/null 2>&1;echo $$? | grep -v 0)
endef


# rule_include
# $(1) rule file path
define rule_include
$(if $(call file_is_exist,$(1)),$(error Rule File "$(1)" Not Exist !!!),include $(1))
endef


$(eval $(call rule_include,$(PROJECT_RULE_PATH)/function/BasicFunction.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/function/EchoFunction.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/function/StringFunction.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/function/FileFunction.mk))




###############################################################################
# Default Configs



ifneq ($(PROJECT_REPO_TYPE),usersdk)

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/CustomConfig.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/LicenceConfig.mk))

endif


$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/DefaultConfig.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/VerboseConfig.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/DockerConfig.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/BuildConfig.mk))




