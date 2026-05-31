#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




# docker_self_id
# return current container self id
define docker_self_id
$(shell cat /proc/self/mountinfo | awk '$$5=="/etc/resolv.conf" {print $$4}' | awk -F "/" '{print $$(NF-1)}')
endef



# docker_host_temp_root
# return current container temp root directory path in host
define docker_host_temp_root
$(shell docker inspect $(call docker_self_id) -f "{{.GraphDriver.Data.MergedDir}}")
endef



# docker_host_mount_root
# $(1) path in current container
# return mount path in host
define docker_host_mount_root
$(shell docker inspect $(call docker_self_id) -f '{{range .Mounts}}{{if eq .Destination "$(1)" }}{{printf "%s" .Source}}{{end}}{{end}}')
endef



# docker_host_map_root
# $(1) path in current container
# return map root in host
define docker_host_map_root
$(if $(call docker_host_mount_root,$(1)),$(call docker_host_mount_root,$(1)),$(call docker_host_temp_root)$(1))
endef



# docker_host_map_path
# $(1) path in current container
# return map path in host
define docker_host_map_path
$(if $(CI_BUILDS_DIR),$(call docker_host_map_root,$(CI_BUILDS_DIR))/$(patsubst $(CI_BUILDS_DIR)%,%,$(1)),$(if $(call is_in_docker),$(1),$(call docker_host_map_root,$(1))))
endef




DOCKER_IMAGE				:= $(patsubst %,image_%,$(DOCKER_TAG))

DOCKER_RUN					:= $(patsubst %,run_%,$(DOCKER_TAG))

DOCKER_RUND					:= $(patsubst %,rund_%,$(DOCKER_TAG))

DOCKER_CMD					:= $(patsubst %,cmd_%,$(DOCKER_TAG))

DOCKER_EXEC					:= $(patsubst %,exec_%,$(DOCKER_TAG))

DOCKER_BASH					:= $(patsubst %,bash_%,$(DOCKER_TAG))

DOCKER_ZSH					:= $(patsubst %,zsh_%,$(filter-out base,$(DOCKER_TAG)))

DOCKER_SSH					:= $(patsubst %,ssh_%,$(DOCKER_TAG))

DOCKER_START				:= $(patsubst %,start_%,$(DOCKER_TAG))

DOCKER_STOP					:= $(patsubst %,stop_%,$(DOCKER_TAG))

DOCKER_RM					:= $(patsubst %,rm_%,$(DOCKER_TAG))

DOCKER_RMI					:= $(patsubst %,rmi_%,$(DOCKER_TAG))

DOCKER_IP					:= $(patsubst %,ip_%,$(DOCKER_TAG))



# For Device File System And Kernel Module

DOCKER_RUN_ARGS				:= --privileged


DOCKER_RUN_ARGS				+= -v /dev:/dev



# Docker In Docker Arguments

DOCKER_RUN_ARGS				+= -e DOCKER_RUNNING_NESTED=1


DOCKER_RUN_ARGS				+= -v /etc/localtime:/etc/localtime:ro


DOCKER_RUN_ARGS				+= -v /var/run/docker.sock:/var/run/docker.sock



# Terminal Arguments

DOCKER_RUN_ARGS				+= $(DOCKER_TERM_OPTS)



# Debug Arguments

DOCKER_RUN_ARGS				+= -e DOCKER_RUN_DEBUG=$(DOCKER_RUN_DEBUG)



# Image Arguments

DOCKER_RUN_ARGS				+= \
		-e DOCKER_IMG_GROUP=$(DOCKER_IMG_GROUP) \
		-e DOCKER_IMG_USER=$(DOCKER_IMG_USER) \
		-e DOCKER_IMG_GID=$(DOCKER_IMG_GID) \
		-e DOCKER_IMG_UID=$(DOCKER_IMG_UID) \
		-u $(DOCKER_IMG_UID):$(DOCKER_IMG_GID)


# Container Arguments

DOCKER_RUN_ARGS				+= \
		-e DOCKER_RUN_GROUP=$(DOCKER_RUN_GROUP) \
		-e DOCKER_RUN_USER=$(DOCKER_RUN_USER) \
		-e DOCKER_RUN_GID=$(DOCKER_RUN_GID) \
		-e DOCKER_RUN_UID=$(DOCKER_RUN_UID)



# HyperEnv Initrc Arguments

DOCKER_RUN_ARGS				+= \
		-e DOCKER_INITRC=$(DOCKER_INITRC) \
		-e DOCKER_INITCFG=$(DOCKER_INITCFG)



# HyperEnv Authorization Arguments

ifeq ($(call is_in_docker),)

#$(warning PROJECT_ROOT=$(PROJECT_ROOT), MAP_ROOT=$(call file_dst,$(DIR_MAP)))
ifeq ($(PROJECT_ROOT),$(call file_dst,$(DIR_MAP)))

ifneq ($(call file_is_exist,$(PROJECT_ROOT)/$(DOCKER_AUTH_FILE)),)
$(error HyperEnv Authorization File Not Exist !)
endif

endif # ($(PROJECT_ROOT),$(call file_dst,$(DIR_MAP)))


else # ($(call is_in_docker),)

ifneq ($(PROJECT_REPO_TYPE),usersdk)

ifneq ($(call file_is_exist,$(PROJECT_ROOT)/$(DOCKER_AUTH_FILE)),)
$(error HyperEnv Authorization File Not Exist !)
endif

endif # ($(PROJECT_REPO_TYPE),usersdk)


endif # ($(call is_in_docker),)



DOCKER_RUN_ARGS				+= \
		-e DOCKER_AUTH_FILE=$(DOCKER_AUTH_FILE)



ifeq ($(PROJECT_REPO_TYPE),usersdk)

$(shell touch $(PROJECT_ROOT)/$(DOCKER_AUTH_FILE))

endif



ifeq ($(DOCKER_RUNNING_NESTED),1)

DOCKER_RUN_ARGS				+= \
		--mount type=bind,source=$(call docker_host_map_path,/etc/$(DOCKER_AUTH_FILE)),destination=/etc/$(DOCKER_AUTH_FILE)

else # ($(DOCKER_RUNNING_NESTED),1)

DOCKER_RUN_ARGS				+= \
		--mount type=bind,source=$(call docker_host_map_path,$(PROJECT_ROOT)/$(DOCKER_AUTH_FILE)),destination=/etc/$(DOCKER_AUTH_FILE)

endif # ($(DOCKER_RUNNING_NESTED),1)



# User Work Directory Map Arguments

DOCKER_RUN_ARGS				+= \
		-e DOCKER_RUN_PATH=$(call file_dst,$(DIR_MAP)) \
		--mount type=bind,source=$(call docker_host_map_path,$(call file_src,$(DIR_MAP))),destination=$(call file_dst,$(DIR_MAP))



# SSH Key Map Arguments

DOCKER_RUN_ARGS				+= \
		-e DOCKER_SSH_MAP_KEY=$(DOCKER_SSH_MAP_KEY)

ifneq ($(DOCKER_SSH_MAP_KEY),0)

DOCKER_RUN_ARGS				+= \
		-v ~/.ssh/id_rsa:/home/$(DOCKER_IMG_USER)/.ssh/id_rsa \
		-v ~/.ssh/id_rsa.pub:/home/$(DOCKER_IMG_USER)/.ssh/id_rsa.pub

endif



# define docker_run_plat
# $(1) docker image tag
# return platform
define docker_run_plat
$(shell x=$$(echo $(filter %$(1),$(PROJECT_TARGETS)) | awk -F "_" '{print $$1}'); [ -z $$x ] && echo $(PLAT_DEFAULT) || echo $${x})
endef



# define docker_run_board
# $(1) running platform
# $(2) docker image tag
# return board
define docker_run_board
$(shell [ $(1) = $(PLAT_DEFAULT) ] && echo $(BOARD_DEFAULT) || echo $(2))
endef



# define docker_run_env
# $(1) docker image tag
# return env set scripts
define docker_run_env
export DOCKER_RUN_PLAT=$(call docker_run_plat,$(1)) && \
export DOCKER_RUN_BOARD=$$([ $${DOCKER_RUN_PLAT} = $(PLAT_DEFAULT) ] && echo $(BOARD_DEFAULT) || echo $(1))
endef



# define docker_run_cmd
# $(1) docker image tag
# $(2) run cmd
define docker_run_cmd
$(call docker_run_env,$(1)) && \
docker run --rm \
		-e DOCKER_RUN_PLAT=$${DOCKER_RUN_PLAT} \
		-e DOCKER_RUN_BOARD=$${DOCKER_RUN_BOARD} \
		$(DOCKER_CMD_ARGS) \
		$(DOCKER_RUN_ARGS) \
		$(DOCKER_NAME):$(1) \
		$(if $(2), sh -c $(2)); \
exit 0
endef



# docker_daemon_name
# $(1) docker image tag
define docker_daemon_name
$(DOCKER_NAME)_$(1)_$(shell whoami)_$(shell echo $(PROJECT_ROOT) | md5sum | cut -f1 -d " " | head -c6)
endef



# define docker_rund_cmd
# $(1) docker image tag
define docker_rund_cmd
$(call docker_run_env,$(1)) && \
docker run -d --restart=always \
		--name $(call docker_daemon_name,$(1)) \
		-e DOCKER_RUN_PLAT=$${DOCKER_RUN_PLAT} \
		-e DOCKER_RUN_BOARD=$${DOCKER_RUN_BOARD} \
		$(DOCKER_CMD_ARGS) \
		$(DOCKER_RUN_ARGS) \
		$(DOCKER_NAME):$(1)
endef



# docker_daemon_id
# $(1) docker image tag
define docker_daemon_id
$(shell docker ps -a | awk '$$NF=="$(call docker_daemon_name,$(1))" {print $$1}' | head -n1)
endef



# docker_image_id
# $(1) docker image tag
define docker_image_id
$(shell docker images | awk '$$1=="$(DOCKER_NAME)" && $$2=="$(1)" {print $$3}')
endef



# docker_ip
# $(1) docker image tag
define docker_ip
$(shell docker exec $(DOCKER_TERM_OPTS) $(call docker_daemon_name,$(1)) ip addr show eth0 | tr "/" " " | awk '$$1=="inet" {print $$2}')
endef



# docker_start
# $(1) docker image tag
define docker_start
$(if $(call docker_daemon_id,$(1)),docker start $(call docker_daemon_name,$(1)) > /dev/null && $(ECHO) "start $(call docker_daemon_name,$(1)) ok")
endef



# docker_stop
# $(1) docker image tag
define docker_stop
$(if $(call docker_daemon_id,$(1)),ssh-keygen -f "$${HOME}/.ssh/known_hosts" -R "$(call docker_ip,$(1))" > /dev/null 2>&1;docker stop -t0 $(call docker_daemon_name,$(1)) > /dev/null && $(ECHO) "stop $(call docker_daemon_name,$(1)) ok";)
endef



# docker_rm
# $(1) docker image tag
define docker_rm
$(if $(call docker_daemon_id,$(1)),docker rm $(call docker_daemon_name,$(1)) > /dev/null && $(ECHO) "remove $(call docker_daemon_name,$(1)) ok")
endef



# docker_rmi
# $(1) docker image tag
define docker_rmi
$(if $(call docker_image_id,$(1)),docker rmi $(DOCKER_NAME):$(1) > /dev/null && $(ECHO) "remove $(DOCKER_NAME):$(1) ok")
endef



.PHONY : $(DOCKER_IMAGE) $(DOCKER_RUN) $(DOCKER_RUND) $(DOCKER_EXEC) $(DOCKER_BASH) $(DOCKER_ZSH) \
		$(DOCKER_SSH) $(DOCKER_START) $(DOCKER_STOP) $(DOCKER_RM) $(DOCKER_RMI) $(DOCKER_IP)


USR_SCRIPT					:= "USR_CMD=$(if $(USR_CMD),\"$(USR_CMD);exit\") . $(DOCKER_INITRC)"



ifeq ($(PROJECT_REPO_TYPE),usersdk)

$(DOCKER_IMAGE) : 

else # ($(PROJECT_REPO_TYPE),usersdk)


ifeq ($(DOCKER_IMG_SOURCE),remote)

$(DOCKER_IMAGE) : image_% : pull_%

else # ($(DOCKER_IMG_SOURCE),remote)


ifeq ($(DOCKER_IMG_SOURCE),local)

$(DOCKER_IMAGE) : image_% : build_%

else # ($(DOCKER_IMG_SOURCE),local)


$(error Invalid Docker Image Type : "$(DOCKER_IMG_SOURCE)")

endif # ($(DOCKER_IMG_SOURCE),local)

endif # ($(DOCKER_IMG_SOURCE),remote)

endif # ($(PROJECT_REPO_TYPE),usersdk)




$(DOCKER_RUN) : run_% : image_%
	$(Q)$(call docker_run_cmd,$(subst image_,,$<),$(USR_SCRIPT))



$(DOCKER_RUND) : rund_% : image_%
	$(Q)if [ -z $(DOCKER_RUNNING_NESTED) ]; then $(if $(call docker_daemon_id,$(subst image_,,$<)),$(call echox,$(RED),Container \"$(DOCKER_NAME)_$(subst image_,,$<)\" Has Been Started),$(call docker_rund_cmd,$(subst image_,,$<))); fi



$(DOCKER_CMD) : cmd_% : image_%
	$(Q)-ssh -AY $(DOCKER_RUN_USER)@$(call docker_ip,$(subst image_,,$<)) -t "$(USR_CMD)"; exit 0



$(DOCKER_EXEC) : exec_% : image_%
	$(Q)-docker exec $(DOCKER_TERM_OPTS) $(call docker_daemon_id,$(subst image_,,$<)) sh -c $(USR_SCRIPT); exit 0



$(DOCKER_BASH) : bash_% : image_%
	$(Q)-docker exec $(DOCKER_TERM_OPTS) $(call docker_daemon_id,$(subst image_,,$<)) bash; exit 0



$(DOCKER_ZSH) : zsh_% : image_%
	$(Q)-ssh -AY -t $(DOCKER_RUN_USER)@$(call docker_ip,$(subst image_,,$<)) "LC_ALL=en_US.UTF-8 exec dbus-run-session -- zsh -l"; exit 0



$(DOCKER_SSH) : ssh_% : image_%
	$(Q)-ssh -AY -t $(DOCKER_RUN_USER)@$(call docker_ip,$(subst image_,,$<)) "LC_ALL=en_US.UTF-8 exec dbus-run-session -- bash -l"; exit 0



$(DOCKER_START) : start_% : image_%
	$(Q)$(call docker_start,$(subst image_,,$<))



$(DOCKER_STOP) : 
	$(Q)$(call docker_stop,$(patsubst stop_%,%,$@))



$(DOCKER_RM) : rm_% : stop_%
	$(Q)$(call docker_rm,$(patsubst rm_%,%,$@))



$(DOCKER_RMI) : rmi_% : rm_%
	$(Q)$(call docker_rmi,$(patsubst rmi_%,%,$@))



$(DOCKER_IP) : ip_% : image_%
	$(Q)echo -n $(call docker_ip,$(subst image_,,$<))



DOCKER_HELP_TARGET		+= "  RUN   : " $(DOCKER_RUN) "\n\n"
DOCKER_HELP_TARGET		+= "  RUND  : " $(DOCKER_RUND) "\n\n"
DOCKER_HELP_TARGET		+= "  CMD   : " $(DOCKER_CMD) "\n\n"
DOCKER_HELP_TARGET		+= "  EXEC  : " $(DOCKER_EXEC) "\n\n"
DOCKER_HELP_TARGET		+= "  BASH  : " $(DOCKER_BASH) "\n\n"
DOCKER_HELP_TARGET		+= "  ZSH   : " $(DOCKER_ZSH) "\n\n"
DOCKER_HELP_TARGET		+= "  SSH   : " $(DOCKER_SSH) "\n\n"
DOCKER_HELP_TARGET		+= "  START : " $(DOCKER_START) "\n\n"
DOCKER_HELP_TARGET		+= "  STOP  : " $(DOCKER_STOP) "\n\n"
DOCKER_HELP_TARGET		+= "  RM    : " $(DOCKER_RM) "\n\n"
DOCKER_HELP_TARGET		+= "  RMI   : " $(DOCKER_RMI) "\n\n"
DOCKER_HELP_TARGET		+= "  IP    : " $(DOCKER_IP) "\n\n"

DOCKER_HELP_VARIABLE	+= "  " DIR_MAP=\"$(DIR_MAP)\" "\n\n"
DOCKER_HELP_VARIABLE	+= "  " USR_CMD=\"$(USR_CMD)\" "\n\n"

