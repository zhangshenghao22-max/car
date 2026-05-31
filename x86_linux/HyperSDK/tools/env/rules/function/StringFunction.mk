#
#                                 vmRT-Thread
#
# Copyright (c) 2022-2024, Shanghai Real-Thread Electronic Technology Co., Ltd.
# All right reserved.
#
# Republication, copying or redistribution of this source code by any means is
# expressly prohibited without a prior written permission.
#



# string_len
# $(1) string
# return : length of the string
define string_len
$(shell echo -n "$(1)" | wc -m)
endef



# string_to_lower
# $(1) string
define string_to_lower
$(shell echo $(1) | tr '[:upper:]' '[:lower:]')
endef



# string_to_upper
# $(1) string
define string_to_upper
$(shell echo $(1) | tr '[:lower:]' '[:upper:]')
endef



# string_to_camel
# $(1) string
define string_to_camel
$(shell echo $(1) | sed 's/.*/\u&/')
endef



# char_repeat
# $(1) char to repeat
# $(2) repeat number
# notice : char 'X' for space.
define char_repeat
$(shell echo $(foreach n,$(shell seq -s ' ' 1 $(2)),$(1)) | tr -d ' ' | tr 'X' ' ')
endef



