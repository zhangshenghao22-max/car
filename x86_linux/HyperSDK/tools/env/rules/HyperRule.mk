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
# Include Basic Rules


include $(PROJECT_RULE_PATH)/BasicRule.mk




###############################################################################
# Project Environment Varibles Export


PROJECT_ENV_EXPORT			= RT_PLAT=$(PLAT) RT_BOARD=$(BOARD) RT_FLAVOR=$(FLAVOR) V=$(V) \
	PROJECT_ROOT=$(PROJECT_ROOT) PROJECT_REPO_TYPE==$(PROJECT_REPO_TYPE) PROJECT_RULE_PATH=$(PROJECT_RULE_PATH) \
	RT_VERSION=$(PROJECT_VERSION) RT_SUBVERSION=$(PROJECT_SUBVERSION) RT_REVISION=$(PROJECT_REVISION) \
	RT_SCMVERSION=$(PROJECT_SCMVERSION) RT_TYPE=$(PROJECT_BUILD_TYPE) RT_TYPE_STR=$(PROJECT_BUILD_TYPE_STR) \
	RT_TRIAL_WORLD_TIME=$(PROJECT_TRIAL_WORLD_TIME) RT_TRIAL_RUN_TIME=$(PROJECT_TRIAL_RUN_TIME) \
	RT_CUSTOM=$(CUSTOM) RT_DESC=$(DESC) TARGET_USE_PREBUILTS=$(TARGET_USE_PREBUILTS)




###############################################################################
# Include Extra Rules


$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildDtb.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildITS.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildBoot.mk))


ifeq ($(PROJECT_REPO_TYPE),main)

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildVMRTT.mk))

else # ($(PROJECT_REPO_TYPE),main)

vmrtt :

vmrtt_clean : 

endif # ($(PROJECT_REPO_TYPE),main)



target_output : $(TARGET_INSTALL_ROOT)



ifneq ($(PROJECT_REPO_TYPE),usersdk)

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildRelease.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildTarget.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildInfo.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildDownload.mk))

endif # ($(PROJECT_REPO_TYPE),usersdk)



ifeq ($(PROJECT_REPO_TYPE),devsdk)

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/build/BuildSDK.mk))

endif # ($(PROJECT_REPO_TYPE),devsdk)




###############################################################################
# Target Output Images


TARGET_GUEST_PREBUILTS		:= $(foreach type,$(TARGET_TYPE_DEFAULT),$(wildcard $(TARGET_PREBUILTS_ROOT)/$(type)) )

TARGET_GUEST_OUTPUT			:= $(patsubst $(TARGET_PREBUILTS_ROOT)/%,$(TARGET_INSTALL_ROOT)/%,$(TARGET_GUEST_PREBUILTS))



$(TARGET_GUEST_OUTPUT) : $(TARGET_INSTALL_ROOT)/% : $(TARGET_PREBUILTS_ROOT)/%
	$(Q)cp -fv $< $@



target_prebuilts : $(TARGET_INSTALL_ROOT) $(TARGET_GUEST_OUTPUT)



ifneq ($(TARGET_USE_PREBUILTS),0)

target_output : 
	$(Q)$(MAKE) target_prebuilts


else # ($(TARGET_USE_PREBUILTS),0)

target_output : 
	$(Q)$(MAKE) target_install


endif # ($(TARGET_USE_PREBUILTS),0)




###############################################################################
# Docker Rules


ifneq ($(PROJECT_REPO_TYPE),usersdk)

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/docker/DockerImg.mk))

endif # ($(PROJECT_REPO_TYPE),usersdk)


$(eval $(call rule_include,$(PROJECT_RULE_PATH)/docker/DockerRun.mk))




###############################################################################
# Emulator Rules


ifeq ($(PLAT),qemu)

.PHONY	: run

run		: qemu_run

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/config/QemuConfig.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/emulator/VirtNetwork.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/emulator/VirtImage.mk))

$(eval $(call rule_include,$(PROJECT_RULE_PATH)/emulator/QemuRun.mk))

endif # ($(PLAT),qemu)




###############################################################################
# Default Targets


ifneq ($(PROJECT_REPO_TYPE),usersdk)

PROJECT_TARGETS					:= $(shell ls $(PROJECT_ROOT)/target/*/*/DeviceConfig.mk | awk -F "/" '{print $$(NF-2) "_" $$(NF-1)}')
PROJECT_TARGETS_PATTERN			:= $(foreach target,$(PROJECT_TARGETS),$(target)_%)

#$(warning PROJECT_TARGETS="$(PROJECT_TARGETS)")
#$(warning PROJECT_TARGETS_PATTERN="$(PROJECT_TARGETS_PATTERN)")


PROJECT_TARGETS_SHORT			:= $(shell ls $(PROJECT_ROOT)/target/*/*/DeviceConfig.mk | awk -F "/" '{print $$(NF-1)}')
PROJECT_TARGETS_SHORT_PATTERN	:= $(foreach target,$(PROJECT_TARGETS_SHORT),$(target)_%)

endif



.PHONY	: build clean help FORCE $(PROJECT_TARGETS) $(PROJECT_TARGETS_PATTERN) $(PROJECT_TARGETS_SHORT)



build	: hyperboot



clean	: hyperboot_clean



help : 
	$(Q)$(call echox_prompt_title,	"vmRT-Thread Help",$(LIGHTBLUE))
	$(Q)$(call echox,$(LIGHTRED),"  make build [FLAVOR=flavor name]             - Build HyperBoot image.")
	$(Q)$(call echox,$(LIGHTRED),"  make clean [FLAVOR=flavor name]             - Clean all temporary and target files.")
	$(Q)$(call echox,$(LIGHTRED),"  make vmrtt                                  - Build vmRT-Thread image only.")
ifeq ($(PLAT),qemu)
	$(Q)$(call echox,$(LIGHTRED),"  make run   [QEMU_TERM_TYPE=none/gnome/tmux] - Run Qemu emulator,(AArch64 Only).")
endif
	$(Q)$(call echox_prompt_line,$(LIGHTBLUE))




###############################################################################
# Shortcut for Targets


PROJECT_SHORTCUT_ENV_EXPORT	:= TARGET_USE_PREBUILTS=$(TARGET_USE_PREBUILTS)


# Example : bst_ipu02c qemu_virt-aarch64 rockchip_hd3568 rockchip_ok3588 rockchip_onebox rockchip_rockpi5b

$(PROJECT_TARGETS) : 
	$(Q)export PLAT=`echo $@ | awk -F "_" '{print $$1}'` && \
		export BOARD=`echo $@ | awk -F "_" '{print $$2}'` && \
		$(call echox,$(BLUE),"Build Platform \"$${PLAT}\" Board \"$${BOARD}\"\n") && \
		$(MAKE) $(PROJECT_SHORTCUT_ENV_EXPORT) PLAT=$${PLAT} BOARD=$${BOARD}



# Example : ipu02c virt-aarch64 hd3568 ok3588 onebox rockpi5b

$(PROJECT_TARGETS_SHORT) : 
	$(Q)export PLAT=$(shell echo $(filter %$@,$(PROJECT_TARGETS)) | awk -F "_" '{print $$1}') && \
		export BOARD=$@ && \
		$(call echox,$(BLUE),"Build Platform \"$${PLAT}\" Board \"$${BOARD}\"\n") && \
		$(MAKE) $(PROJECT_SHORTCUT_ENV_EXPORT) PLAT=$${PLAT} BOARD=$${BOARD}



# Example : bst_ipu02c_% qemu_virt-aarch64_% rockchip_hd3568_% rockchip_ok3588_% rockchip_onebox_% rockchip_rockpi5b_%

$(PROJECT_TARGETS_PATTERN) : 
	$(Q)export PLAT=`echo $@ | awk -F "_" '{print $$1}'` && \
		export BOARD=`echo $@ | awk -F "_" '{print $$2}'` && \
		export LENGTH=`echo $${PLAT}_$${BOARD}_ | wc -c` && \
		export TARGET=`echo $@ | cut -b $${LENGTH}-` && \
		$(call echox,$(BLUE),"Build Platform \"$${PLAT}\" Board \"$${BOARD}\" Target \"$${TARGET}\"\n") && \
		$(MAKE) $(PROJECT_SHORTCUT_ENV_EXPORT) PLAT=$${PLAT} BOARD=$${BOARD} $${TARGET}



# Example : ipu02c_% virt-aarch64_% hd3568_% ok3588_% onebox_% rockpi5b_%

$(PROJECT_TARGETS_SHORT_PATTERN) : 
	$(Q)export BOARD=`echo $@ | awk -F "_" '{print $$1}'` && \
		export PLAT=`echo $(PROJECT_TARGETS) | tr " " "\n" | grep $${BOARD} | awk -F "_" '{print $$1}'` && \
		export LENGTH=`echo $${BOARD}_ | wc -c` && \
		export TARGET=`echo $@ | cut -b $${LENGTH}-` && \
		$(call echox,$(BLUE),"Build Platform \"$${PLAT}\" Board \"$${BOARD}\" Target \"$${TARGET}\"\n") && \
		$(MAKE) $(PROJECT_SHORTCUT_ENV_EXPORT) PLAT=$${PLAT} BOARD=$${BOARD} $${TARGET}



