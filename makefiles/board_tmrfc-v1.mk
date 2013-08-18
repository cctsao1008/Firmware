#
# Board-specific definitions for the TMRFC
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = TMRFC_V1

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
