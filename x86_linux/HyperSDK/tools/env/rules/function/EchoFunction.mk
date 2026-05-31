#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#




# Echo Colors

NOCOLOR						:= \033[0m
RED							:= \033[0;31m
GREEN						:= \033[0;32m
ORANGE						:= \033[0;33m
BLUE						:= \033[0;34m
PURPLE						:= \033[0;35m
CYAN						:= \033[0;36m
LIGHTGRAY					:= \033[0;37m
DARKGRAY					:= \033[1;30m
LIGHTRED					:= \033[1;31m
LIGHTGREEN					:= \033[1;32m
YELLOW						:= \033[1;33m
LIGHTBLUE					:= \033[1;34m
LIGHTPURPLE					:= \033[1;35m
LIGHTCYAN					:= \033[1;36m
WHITE						:= \033[1;37m



# Echo Config

ECHOX_PROMPT_KEY_SHIFT		?= 4
ECHOX_PROMPT_KEY_WIDTH		?= 30
ECHOX_PROMPT_TOTAL_WIDTH	?= $(shell echo `TERM=xterm tput cols` - 2 | bc)

ECHOX_PROMPT_FILLED_CHAR	?= '='

ECHOX_PROMPT_TITLE_COLOR	?= $(LIGHTRED)
ECHOX_PROMPT_LINE_COLOR		?= $(LIGHTRED)
ECHOX_PROMPT_INFO_COLOR		?= $(GREEN)
ECHOX_PROMPT_VAULE_COLOR	?= $(CYAN)



# echox
# $(1) fonts
# $(2)...$(6) string
define echox
$(ECHO) "$(if $(1),$(1))" $(2) $(3) $(4) $(5) $(6) "$(if $(1),$(NOCOLOR))"
endef



# echox_prompt_chars
# $(1) width
# $(2) char
define echox_prompt_chars
"$(call char_repeat,$(if $(2),$(2),$(ECHOX_PROMPT_FILLED_CHAR)),$(if $(1),$(1),$(ECHOX_PROMPT_TOTAL_WIDTH)))"
endef



# echox_prompt_line
# $(1) color
# $(2) char
# $(3) width
define echox_prompt_line
$(call echox,$(if $(1),$(1),$(ECHOX_PROMPT_LINE_COLOR)),"\n "$(call echox_prompt_chars,$(if $(3),$(3),$(ECHOX_PROMPT_TOTAL_WIDTH)),$(if $(2),$(2),$(ECHOX_PROMPT_FILLED_CHAR)))" \n")
endef



# echox_prompt_string_left_len
# $(1) string
# $(2) width
define echox_prompt_string_left_len
$(shell echo "($(if $(2),$(2),$(ECHOX_PROMPT_TOTAL_WIDTH)) - 2 - $(call string_len,$(1))) / 2" | bc)
endef



# echox_prompt_string_right_len
# $(1) string
# $(2) width
# $(3) length of left
define echox_prompt_string_right_len
$(shell echo "$(if $(2),$(2),$(ECHOX_PROMPT_TOTAL_WIDTH)) - 2 - $(3) - $(call string_len,$(1))" | bc)
endef



# echox_prompt_string_center
# $(1) string
# $(2) width
# $(3) char
# notice : char 'X' for space.
define echox_prompt_string_center
$(call echox_prompt_chars,$(call echox_prompt_string_left_len,$(1),$(2)),$(3))" $(1) "$(call echox_prompt_chars,$(call echox_prompt_string_right_len,$(1),$(2),$(call echox_prompt_string_left_len,$(1),$(2))),$(3))
endef



# echox_prompt_string_left
# $(1) string
# $(2) width of left
# $(3) char
# notice : char 'X' for space.
define echox_prompt_string_left
$(call echox_prompt_chars,$(ECHOX_PROMPT_KEY_SHIFT),$(if $(3),$(3),X))$(1)$(call echox_prompt_chars,$(shell echo "$(2) - $(ECHOX_PROMPT_KEY_SHIFT) - $(call string_len,$(1))" | bc),$(if $(3),$(3),X))
endef



# echox_prompt_title
# $(1) string
# $(2) color
# $(3) char
# $(4) width
# notice : char 'X' for space.
define echox_prompt_title
$(call echox,$(if $(2),$(2),$(ECHOX_PROMPT_TITLE_COLOR)),"\n "$(call echox_prompt_string_center,$(strip $(1)),$(4),$(3))" \n")
endef



# echox_prompt_info
# $(1) string
# $(2) color
# $(3) width
# $(4) char
# notice : char 'X' for space.
define echox_prompt_info
$(call echox,$(if $(2),$(2),$(ECHOX_PROMPT_INFO_COLOR)),"\n "$(call echox_prompt_string_center,$(strip $(1)),$(3),$(if $(4),$($(4)),X))" \n")
endef



# echox_prompt_value
# $(1) key
# $(2) value
# $(3) color
# $(4) width of key
# $(5) char
# notice : char 'X' for space.
define echox_prompt_value
$(call echox,$(if $(3),$(3),$(ECHOX_PROMPT_VAULE_COLOR)),$(call echox_prompt_string_left,$(strip $(1)),$(if $(4),$(4),$(ECHOX_PROMPT_KEY_WIDTH)),$(if $(5),$(5),X))" : "$(strip $(2)))
endef

define echox_prompt_notice
$(call echox,$(if $(2),$(2),$(ECHOX_PROMPT_INFO_COLOR)),""$(call echox_prompt_string_left,$(strip $(1)),$(3),$(if $(4),$($(4)),X))"")
endef



