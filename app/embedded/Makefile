SDK_PATH=qorc-sdk
HAL_PATH=$(SDK_PATH)/HAL
BSP_PATH=$(SDK_PATH)/BSP
LIB_SDK_PATH=$(SDK_PATH)/Libraries
FREERTOS_PATH=$(SDK_PATH)/FreeRTOS
FREERTOS_GATEWARE_PATH=$(SDK_PATH)/freertos_gateware
LIB_PATH_CMSIS=$(LIB_SDK_PATH)/CMSIS

DEVICE=ARM_CM4F_quicklogic_s3XX
LDSCRIPT=src/ql-qtplus2.ld

OUTPUT_FILE = test

BUILD_DIR = build
# OBCOPY = /opt/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-objcopy
# CC = /opt/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc
OBCOPY = arm-none-eabi-objcopy
CC = arm-none-eabi-gcc
LD = $(CC)

OPTIMIZATION = 0

STANDARD = c99

FLAGS_DEBUG = \
	-gdwarf-4

FLAGS_ARM = \
	$(FLAGS_DEBUG)			\
	-O$(OPTIMIZATION)		\
	-mcpu=cortex-m4			\
	-mthumb					\
	-mlittle-endian			\
	-mfloat-abi=hard		\
	-mfpu=fpv4-sp-d16

FLAGS_DEFINES = \
	-D__FPU_USED=1								\
	-D__FPU_USED=1								\
	-D__M4_DEBUG								\
	-D__EOSS3_CHIP								\
	-D__RTOS									\
	-D__GNU_SOURCE								\
	-D_DEFAULT_SOURCE							\
	-DARM_MATH_CM4								\
	-DFFE_NEWARCH								\
	-DARM_MATH_MATRIX_CHECK						\
	-DARM_MATH_ROUNDING							\
	-D__FPU_PRESENT								\
	-DconfigUSE_STATS_FORMATTING_FUNCTIONS		\
	-DconfigUSE_TRACE_FACILITY					\
	-DGCC										\
	-DNDEBUG									\
	-DGCC_MAKE

FLAG_LIB = \
	-lm						\
	-L$(LIB_PATH_CMSIS)

REMAINING_LIB_FLAG=-larm_cortexM4 -lf_math

AFLAGS = \
	$(FLAGS_ARM)			\
	-fmessage-length=0		\
	-fsigned-char			\
	-ffunction-sections		\
	-fdata-sections			\
	-MMD					\
	-MP

CFLAGS = \
	$(FLAGS_ARM)				\
	$(FLAGS_DEFINES)			\
	$(FLAG_LIB)					\
	-std=$(STANDARD)			\
	-fmerge-constants			\
	-fomit-frame-pointer		\
	-fcrossjumping				\
	-fexpensive-optimizations	\
	-ftoplevel-reorder			\
	-MD

LDFLAGS = \
	$(AFLAGS)							\
	$(FLAG_LIB)							\
	-T "$(LDSCRIPT)"					\
	-Xlinker --gc-sections				\
	-Wall								\
	-Werror								\
	-Wl,--fatal-warnings				\
	-Wl,-Map,"$(OUTPUT_FILE).map"		\
	--specs=nano.specs					\
	--specs=nosys.specs					\
	-Wl,--no-wchar-size-warning

SRCS_HAL = \
	$(HAL_PATH)/startup/startup_EOSS3B_GCC.c	\
	$(HAL_PATH)/startup/vectors_CM4F_gcc.c		\
	$(HAL_PATH)/src/s3x_clock_hal.c				\
	$(HAL_PATH)/src/eoss3_hal_gpio.c			\
	$(HAL_PATH)/src/eoss3_hal_adc.c            	\
	$(HAL_PATH)/src/eoss3_hal_ffe.c            	\
	$(HAL_PATH)/src/eoss3_hal_i2c.c            	\
	$(HAL_PATH)/src/eoss3_hal_pad_config.c     	\
	$(HAL_PATH)/src/eoss3_hal_pkfb.c           	\
	$(HAL_PATH)/src/eoss3_hal_rtc.c            	\
	$(HAL_PATH)/src/eoss3_hal_sdma.c           	\
	$(HAL_PATH)/src/eoss3_hal_spi.c            	\
	$(HAL_PATH)/src/eoss3_hal_timer.c          	\
	$(HAL_PATH)/src/eoss3_hal_uart.c           	\
	$(HAL_PATH)/src/eoss3_hal_wb.c             	\
	$(HAL_PATH)/src/eoss3_hal_wdt.c				\
	$(HAL_PATH)/src/eoss3_hal_i2s.c            	\
	$(HAL_PATH)/src/eoss3_hal_i2s_master_assp.c	\
	$(HAL_PATH)/src/eoss3_hal_i2s_slave_assp.c

HAL_NOT_WORKING = \
	$(HAL_PATH)/src/eoss3_hal_audio.c
INC_SDK = \
	$(LIB_SDK_PATH)/Audio/inc 				\
	$(LIB_SDK_PATH)/CMSIS/inc    			\
	$(LIB_SDK_PATH)/cli/inc    				\
	$(LIB_SDK_PATH)/Power/inc    			\
	$(LIB_SDK_PATH)/SysFlash/inc    		\
	$(LIB_SDK_PATH)/Utils/inc    			\
	$(LIB_SDK_PATH)/FPGA/inc    			\
	$(LIB_SDK_PATH)/DatablockManager/inc	\
	$(LIB_SDK_PATH)

SRCS_LIB_SDK = \
	$(LIB_SDK_PATH)/Power/src/hearable_pwrcfg.c			\
	$(LIB_SDK_PATH)/Power/src/s3x_clock.c				\
	$(LIB_SDK_PATH)/Power/src/s3x_qos.c					\
	$(LIB_SDK_PATH)/Power/src/s3x_cpuload.c				\
	$(LIB_SDK_PATH)/Power/src/s3x_dfs.c					\
	$(LIB_SDK_PATH)/Power/src/s3x_lpm.c					\
	$(LIB_SDK_PATH)/Power/src/s3x_pi.c					\
	$(LIB_SDK_PATH)/Utils/src/dbg_uart.c				\
	$(LIB_SDK_PATH)/FPGA/src/fpga_loader.c

NOT_WIRKING_SRCS_LIB=\
	$(LIB_SDK_PATH)/Power/src/s3x_pwrcfg_prototype.c	\
	$(LIB_SDK_PATH)/Power/src/s3x_fsm_interface.c		\

INC_FREERTOS =	\
	$(FREERTOS_PATH)/include			\
	$(FREERTOS_PATH)/portable/GCC/$(DEVICE)	\
	$(FREERTOS_GATEWARE_PATH)/inc

SRCS_FREERTOS =	\
	$(FREERTOS_PATH)/portable/GCC/$(DEVICE)/port.c		\
	$(FREERTOS_PATH)/croutine.c							\
	$(FREERTOS_PATH)/event_groups.c						\
	$(FREERTOS_PATH)/freertos_static_memory.c			\
	$(FREERTOS_PATH)/list.c								\
	$(FREERTOS_PATH)/queue.c							\
	$(FREERTOS_PATH)/stream_buffer.c					\
	$(FREERTOS_PATH)/tasks.c							\
	$(FREERTOS_PATH)/timers.c							\
	$(FREERTOS_PATH)/portable/MemMang/heap_1.c			\
	$(FREERTOS_GATEWARE_PATH)/src/eoss3_hal_fpga_clk_sync.c			\
	$(FREERTOS_GATEWARE_PATH)/src/eoss3_hal_fpga_gpio.c				\
	$(FREERTOS_GATEWARE_PATH)/src/eoss3_hal_fpga_sdma.c				\
	$(FREERTOS_GATEWARE_PATH)/src/eoss3_hal_fpga_uart.c				\
	$(FREERTOS_GATEWARE_PATH)/src/eoss3_hal_fpga_usbserial.c


INC_HAL = \
	$(HAL_PATH)/inc

INC_BSP = \
	$(BSP_PATH)/quickfeather/inc

SRCS = \
	$(SRCS_HAL) \
	$(SRCS_LIB_SDK) \
	$(SRCS_FREERTOS) \
	src/main.c \
	src/board/pincfg_table.c \
	src/board/s3x_pwrcfg.c\
	src/board/sec_debug.c\
	src/board/exceptions.c\
	src/board/qorc_hardwaresetup.c

INC = \
	$(INC_SDK) \
	$(INC_HAL) \
	$(INC_BSP) \
	$(INC_FREERTOS) \
	src \
	src/board

INCS := $(foreach _INC,$(INC),-I$(_INC))
OBJS = $(SRCS:.c=.o)
BUILDOBJS = $(foreach OB,$(OBJS),$(BUILD_DIR)/$(notdir $(OB)))

%.o: %.c
	@echo [CC] $<
	@$(CC) $(CFLAGS) $(INCS) -c $< -o $(BUILD_DIR)/$(notdir $@)

$(BUILD_DIR)/$(OUTPUT_FILE).elf: builddir $(OBJS)
	@echo [LD] $(notdir $(BUILDOBJS))
	@$(LD) $(LDFLAGS) -o $(BUILD_DIR)/$(OUTPUT_FILE).elf $(BUILDOBJS)

$(BUILD_DIR)/$(OUTPUT_FILE).hex: $(BUILD_DIR)/$(OUTPUT_FILE).elf
	@echo [OBCOPY] $(OUTPUT_FILE).hex
	@$(OBCOPY) -O ihex $(BUILD_DIR)/$(OUTPUT_FILE).elf $(BUILD_DIR)/$(OUTPUT_FILE).hex

$(BUILD_DIR)/$(OUTPUT_FILE).bin: $(BUILD_DIR)/$(OUTPUT_FILE).hex
	@echo [OBCOPY] $(OUTPUT_FILE).bin
	@$(OBCOPY) -I ihex $(BUILD_DIR)/$(OUTPUT_FILE).hex -O binary $(BUILD_DIR)/$(OUTPUT_FILE).bin

all: $(BUILD_DIR)/$(OUTPUT_FILE).bin

builddir:
	mkdir -p $(BUILD_DIR)
