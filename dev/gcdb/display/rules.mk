LOCAL_DIR := $(GET_LOCAL_DIR)

INCLUDES += -I$(LOCAL_DIR)/include

OBJS += \
    $(LOCAL_DIR)/gcdb_display.o \
    $(LOCAL_DIR)/gcdb_display_param.o \
    $(LOCAL_DIR)/panel_display.o \
    $(LOCAL_DIR)/gcdb_autopll.o

ifeq ($(DSI2DPI_TC358762), 1)
OBJS += \
    $(LOCAL_DIR)/mipi_tc358762_dsi2dpi.o
endif
