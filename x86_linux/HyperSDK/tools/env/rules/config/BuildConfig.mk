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
# Build Version Config


PROJECT_VERSION_DIGITS		?= 2

# version_convert
# $(1) version number
# $(2) default version number
define version_convert
$(shell version=`echo $(1) | sed 's/[^0-9]*//g' | cut -c -$(PROJECT_VERSION_DIGITS) | tr -d " "`; if [ x$${version} != x ]; then echo \"$${version}\"; else echo $(2); fi)
endef


ifeq ($(PROJECT_VERSION),)
ifeq ($(PROJECT_SUBVERSION),)
ifeq ($(PROJECT_REVISION),)


PROJECT_VERSION				?= 0
PROJECT_SUBVERSION			?= 7
PROJECT_REVISION			?= 1


ifeq ($(call file_is_exist,$(PROJECT_ROOT)/.git),)

PROJECT_VERSION_FULL		:= $(shell git --git-dir=$(PROJECT_ROOT)/.git describe --tags --always --dirty="-dirty")
PROJECT_VERSION_FULL		:= $(if $(PROJECT_VERSION_FULL),$(PROJECT_VERSION_FULL),v$(PROJECT_VERSION).$(PROJECT_SUBVERSION).$(PROJECT_REVISION))


PROJECT_VERSION_MAIN		:= $(shell echo $(PROJECT_VERSION_FULL) | awk -F "-" '{ print $$1}' | tr -d "v")
PROJECT_VERSION_MAIN		:= $(if $(PROJECT_VERSION_MAIN),$(PROJECT_VERSION_MAIN),$(PROJECT_VERSION).$(PROJECT_SUBVERSION).$(PROJECT_REVISION))


PROJECT_VERSION				:= $(shell echo $(PROJECT_VERSION_MAIN) | awk -F "." '{ print $$1}')
PROJECT_VERSION				:= $(call version_convert,$(PROJECT_VERSION),$(PROJECT_VERSION))


PROJECT_SUBVERSION			:= $(shell echo $(PROJECT_VERSION_MAIN) | awk -F "." '{ print $$2}')
PROJECT_SUBVERSION			:= $(call version_convert,$(PROJECT_SUBVERSION),$(PROJECT_SUBVERSION))


PROJECT_REVISION			:= $(shell echo $(PROJECT_VERSION_MAIN) | awk -F "." '{ print $$3}')
PROJECT_REVISION			:= $(call version_convert,$(PROJECT_REVISION),$(PROJECT_REVISION))


PROJECT_SCMVERSION			:= $(shell echo $(PROJECT_VERSION_FULL) | awk -F "-" '{print $3}' | sed -e 's/^g//g')
PROJECT_SCMVERSION			:= $(if $(PROJECT_SCMVERSION),$(PROJECT_SCMVERSION)," ")


endif # ($(call file_is_exist,$(PROJECT_ROOT)/.git,)

endif # ($(PROJECT_REVISION),)
endif # ($(PROJECT_SUBVERSION),)
endif # ($(PROJECT_VERSION),)




###############################################################################
# Build Type Config


PROJECT_BUILD_TYPE_STR		?= $(TYPE)

ifeq ($(PROJECT_BUILD_TYPE_STR),release)
PROJECT_BUILD_TYPE			?= 0
endif

ifeq ($(PROJECT_BUILD_TYPE_STR),debug)
PROJECT_BUILD_TYPE			?= 1
endif

ifeq ($(PROJECT_BUILD_TYPE_STR),perf)
PROJECT_BUILD_TYPE			?= 2
endif

ifeq ($(PROJECT_BUILD_TYPE_STR),test)
PROJECT_BUILD_TYPE			?= 3
endif

ifeq ($(PROJECT_BUILD_TYPE_STR),trial)
PROJECT_BUILD_TYPE			?= 4
endif




###############################################################################
# Build Path Config


ifneq ($(PROJECT_REPO_TYPE),usersdk)

PROJECT_FLAVOR_PATH			:= $(PROJECT_FLAVOR_ROOT)/$(PLAT)/$(BOARD)

PROJECT_TARGET_PATH			:= $(PROJECT_TARGET_ROOT)/$(PLAT)/$(BOARD)


ifneq ($(call file_is_exist,$(PROJECT_FLAVOR_PATH)/$(FLAVOR)),)
$(error invalid flavor name "$(FLAVOR)")
endif


ifneq ($(call file_is_exist,$(PROJECT_TARGET_PATH)),)
$(error invalid platform "$(PLAT)" or target "$(BOARD)" name)
endif


else # ($(PROJECT_REPO_TYPE),usersdk)

PROJECT_FLAVOR_PATH			:= $(PROJECT_FLAVOR_ROOT)

PROJECT_TARGET_PATH			:= $(PROJECT_TARGET_ROOT)

endif # ($(PROJECT_REPO_TYPE),usersdk)



PROJECT_BUILD_PATH			:= $(PROJECT_BUILD_ROOT)/$(PLAT)/$(BOARD)




###############################################################################
# Target Build Configs


TARGET_USE_PREBUILTS		?= 1


TARGET_PREBUILTS_ROOT		:= $(PROJECT_TARGET_PATH)/Prebuilts

TARGET_BUILD_ROOT			:= $(PROJECT_TARGET_PATH)/Build



ifneq ($(PROJECT_REPO_TYPE),usersdk)
TARGET_INSTALL_ROOT			:= $(PROJECT_TARGET_PATH)/Output
else
TARGET_INSTALL_ROOT			:= $(PROJECT_TARGET_PATH)
endif

TARGET_PACKAGE_ROOT			:= $(TARGET_INSTALL_ROOT)


VMRTTHREAD_BIN				:= $(TARGET_INSTALL_ROOT)/$(VMRTTHREAD_NAME).bin

VMRTTHREAD_ELF				:= $(TARGET_INSTALL_ROOT)/$(VMRTTHREAD_NAME).elf

HYPERBOOT_RAW_BIN			:= $(TARGET_INSTALL_ROOT)/$(HYPERBOOT_NAME).bin
HYPERBOOT_AES_BIN			:= $(TARGET_INSTALL_ROOT)/$(HYPERBOOT_NAME).aes
HYPERBOOT_FIT_BIN			:= $(TARGET_INSTALL_ROOT)/$(HYPERBOOT_NAME).itb
HYPERBOOT_FIT_AES_BIN		:= $(TARGET_INSTALL_ROOT)/$(HYPERBOOT_NAME)_itb.aes


TARGET_FIT_ITS_FILE			?= None
TARGET_FIT_KEY_DIR			?= None
TARGET_FIT_MKIMAGE			?= None
TARGET_FIT_PUB_KEY			?= None

TARGET_AES_KEY				?= None
TARGET_AES_IV				?= None
TARGET_AES_LEN				?= 128


ifeq ($(VERIFY),fit)

ifeq ($(ENCRYPT),aes)

HYPERBOOT_DST_BIN 			:= $(HYPERBOOT_FIT_AES_BIN)

else # ($(ENCRYPT),aes)

HYPERBOOT_DST_BIN 			:= $(HYPERBOOT_FIT_BIN)

endif # ($(ENCRYPT),aes)

else ifeq ($(ENCRYPT),aes)

HYPERBOOT_DST_BIN 			:= $(HYPERBOOT_AES_BIN)

else # ($(ENCRYPT),aes)

HYPERBOOT_DST_BIN 			:= $(HYPERBOOT_RAW_BIN)

endif # ($(ENCRYPT),aes)


ifeq ($(BOARD),virt-aarch64)

TARGET_FIT_KERNEL_LOAD_ADDR		?= 0x40200000
TARGET_FIT_DTB_ADDR				?= 0x48000000

else 

ifeq ($(BOARD),rockpi5b)

TARGET_FIT_KERNEL_LOAD_ADDR		?= 0x400000
TARGET_FIT_DTB_ADDR				?= 0xb000000

endif

endif


###############################################################################
# Flavor Files Config


FLAVOR_ARM_GIC_VER			?= 3

FLAVOR_VMDTS_DEFINE			:= ARM_GIC=$(FLAVOR_ARM_GIC_VER)



FLAVOR_CONFIG_FILE			:= $(PROJECT_FLAVOR_PATH)/$(FLAVOR)/$(FLAVOR_CONFIG_FILE_NAME)

ifeq ($(call file_is_exist,$(FLAVOR_CONFIG_FILE)),)
FLAVOR_VM_NUM				?= $(shell awk -F "," 'BEGIN {x=-1} $$1=="vm:" {x=NR+1} NR==x {print NF}' $(FLAVOR_CONFIG_FILE))
else
#$(warning Flavor Config File "$(FLAVOR_CONFIG_FILE)" Is Not Exist !!!)
FLAVOR_VM_NUM				?= 2
endif



FLAVOR_VMDTS_BIND_PATH		:= $(PROJECT_VMRTT_ROOT)/adpt/hal/pal
FLAVOR_VMDTS_BIND_PATH		+= $(PROJECT_FLAVOR_PATH)/common

FLAVOR_VMDTS_BUILD_PATH		:= $(PROJECT_FLAVOR_PATH)/common

FLAVOR_VMDTS_FILES			:= $(wildcard $(PROJECT_FLAVOR_PATH)/$(FLAVOR)/*.dts)
FLAVOR_VMDTB_FILES			:= $(patsubst %.dts,%.dtb,$(FLAVOR_VMDTS_FILES))

FLAVOR_VMITSO_FILES			:= $(wildcard $(PROJECT_FLAVOR_PATH)/$(FLAVOR)/*.itso)
FLAVOR_VMITS_FILES			:= $(patsubst %.itso,%.its,$(FLAVOR_VMITSO_FILES))




###############################################################################
# Release File Config


RELEASE_CHECK_FORCE			?= 1

RELEASE_INFO_TXT			?= $(TARGET_INSTALL_ROOT)/$(RELEASE_INFO_NAME).txt




###############################################################################
# Download Various


DOWNLOAD_URL				?= https://git.rt-thread.com/vmrt-thread/hyperdownload.git

DOWNLOAD_ROOT				?= $(PROJECT_DOWNLOAD_ROOT)



