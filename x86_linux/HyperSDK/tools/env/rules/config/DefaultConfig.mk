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
# Default Variables


PROJECT_DOCS_ROOT			:= $(PROJECT_ROOT)/docs

PROJECT_FLAVOR_ROOT			:= $(PROJECT_ROOT)/flavor

PROJECT_TARGET_ROOT			:= $(PROJECT_ROOT)/target

PROJECT_GUEST_ROOT			:= $(PROJECT_ROOT)/guest

PROJECT_BUILD_ROOT			:= $(PROJECT_ROOT)/build

PROJECT_DOWNLOAD_ROOT		:= $(PROJECT_ROOT)/download


PROJECT_TOOLS_ROOT			:= $(PROJECT_ROOT)/tools

PROJECT_ENV_ROOT			:= $(PROJECT_TOOLS_ROOT)/env

PROJECT_RULES_ROOT			:= $(PROJECT_ENV_ROOT)/rules

PROJECT_DOCKER_ROOT			:= $(PROJECT_ENV_ROOT)/docker

PROJECT_SCRIPTS_ROOT		:= $(PROJECT_ENV_ROOT)/scripts

PROJECT_IMAGE_ROOT			:= $(PROJECT_TOOLS_ROOT)/image



ifneq ($(PROJECT_REPO_TYPE),main)

PROJECT_SOURCE_ROOT			:= $(PROJECT_ROOT)/sources

PROJECT_VMRTT_ROOT			:= $(PROJECT_SOURCE_ROOT)/vmrt-thread

else

PROJECT_SOURCE_ROOT			:= "error"

PROJECT_VMRTT_ROOT			:= $(PROJECT_ROOT)

endif



# Image Various

HYPERIMAGE_NAME				:= hi

HYPERIMAGE_BIN				:= $(PROJECT_IMAGE_ROOT)/$(HYPERIMAGE_NAME)


# Target Various

VMRTTHREAD_NAME				:= vmRT-Thread

HYPERBOOT_NAME				:= HyperBoot


TARGET_TYPE_DEFAULT			:= *.bin *.img *.tar *.tar.gz


# Release Various

RELEASE_INFO_NAME			:= ReleaseInfo


# Flavor Various

FLAVOR_CONFIG_FILE_NAME		:= HyperConfig.yaml




###############################################################################
# Default Configs


# Global Build And Running Configs

PLAT_DEFAULT				:= qemu

PLAT						?= $(if $(DOCKER_RUN_PLAT),$(DOCKER_RUN_PLAT),$(PLAT_DEFAULT))


BOARD_DEFAULT				:= virt-aarch64

BOARD						?= $(if $(DOCKER_RUN_BOARD),$(DOCKER_RUN_BOARD),$(BOARD_DEFAULT))


FLAVOR						?= default


TYPE						?= test


CUSTOM						?= Business

DESC						?= "\"Copyright (c) 2022-$(shell date +"%Y"), Shanghai Real-Thread Electronic Technology Co., Ltd., All right reserved.\""




