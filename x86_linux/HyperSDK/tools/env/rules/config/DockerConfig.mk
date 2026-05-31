#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




# Docker Config

DOCKER_REPO_BASE			?= harbor.rt-thread.com/vmrtt


DOCKER_NAME					?= hyperenv


DOCKER_GROUP				?= $(DOCKER_NAME)

DOCKER_USER					?= $(DOCKER_NAME)


DOCKER_GID					?= $(shell id -g)

DOCKER_UID					?= $(shell id -u)



DOCKER_IMG_SOURCE			?= remote


DOCKER_IMG_GROUP			?= $(DOCKER_GROUP)

DOCKER_IMG_USER				?= $(DOCKER_USER)


DOCKER_IMG_GID				?= 165535

DOCKER_IMG_UID				?= 165535



DOCKER_RUN_DEBUG			?= 0


DOCKER_RUN_GROUP			?= vmrtt

DOCKER_RUN_USER				?= vmrtt


DOCKER_RUN_GID				?= $(DOCKER_GID)

DOCKER_RUN_UID				?= $(DOCKER_UID)



DOCKER_SSH_MAP_KEY			?= 0

DOCKER_SSH_MAP_GID			?= $(DOCKER_IMG_GID)

DOCKER_SSH_MAP_UID			?= $(DOCKER_IMG_UID)



DOCKER_INITRC				?= /etc/hyperenvrc

DOCKER_INITCFG				?= /etc/hyperenvcfg


DOCKER_AUTH_ID				?= hyperenvauth
DOCKER_AUTH_FILE			?= .$(DOCKER_AUTH_ID)



ifneq ($(PROJECT_REPO_TYPE),usersdk)
DOCKER_TAG   				?= $(notdir $(shell find $(PROJECT_DOCKER_ROOT)/* -maxdepth 0 -type d -name "[a-zA-Z0-9]*"))
else
DOCKER_TAG   				?= $(BOARD)
endif


ifeq ($(CI_BUILDS_DIR),)
DOCKER_TERM_OPTS			?= -it
else
DOCKER_TERM_OPTS			?= -i
endif


DOCKER_IMAGE_COMPR			?= 1


DIR_MAP						?= "$(PWD):/hyper"

USR_CMD						?= bash;exit




