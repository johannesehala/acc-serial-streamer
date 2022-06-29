# This is the main makefile for acc-serial-streamer firmware

# _______________________ User overridable configuration _______________________

PROJECT_NAME            ?= streamer

VERSION_MAJOR           ?= 1
VERSION_MINOR           ?= 0
VERSION_PATCH           ?= 0
VERSION_DEVEL           ?= "-dev"

DEFAULT_RADIO_CHANNEL   ?= 26

# Set device address at compile time for cases where a signature is not present
DEFAULT_AM_ADDR         ?= 1

# No bootloader, app starts at 0
APP_START               = 0

# Common build options - some of these should be moved to targets/boards
CFLAGS                  += -Wall -std=c99
CFLAGS                  += -ffunction-sections -fdata-sections -ffreestanding -fsingle-precision-constant -Wstrict-aliasing=0
CFLAGS                  += -DconfigUSE_TICKLESS_IDLE=0
CFLAGS                  += -D__START=main -D__STARTUP_CLEAR_BSS
CFLAGS                  += -DVTOR_START_LOCATION=$(APP_START) -Wl,--section-start=.text=$(APP_START)

LDFLAGS                 += -nostartfiles -Wl,--gc-sections -Wl,--relax -Wl,-Map=$(@:.elf=.map),--cref -Wl,--wrap=atexit -specs=nosys.specs
LDLIBS                  += -lgcc -lm
INCLUDES                += -Xassembler -I$(BUILD_DIR) -I.

# The CMSIS RTOS2 wrapper for FreeRTOS now requires this flag to actually import the components 
CFLAGS                  += -D_RTE_=1

# If set, disables asserts and debugging, enables optimization
RELEASE_BUILD           ?= 0

USE_LLL_LOGGING         ?= 0

ifeq ($(USE_LLL_LOGGING),1)
    # Set the lll verbosity base level
    #CFLAGS                  += -DBASE_LOG_LEVEL=0xFFFF # Everything
    #CFLAGS                  += -DBASE_LOG_LEVEL=0      # Nothing
    CFLAGS                  += -DBASE_LOG_LEVEL=LOG_INFO3
    #  CFLAGS                  += -DBASE_LOG_LEVEL=0x0050 # (LOG_INFO2 | LOG_INFO_4)
    #CFLAGS                  += -DBASE_LOG_LEVEL=0x0088  # (LOG_DEBUG1 | LOG_INFO1)
endif

# Enable debug messages
VERBOSE                 ?= 0
# Disable info messages
#SILENT                  ?= 1

# This project contains several Makefiles that reference the project root
ROOT_DIR                ?= $(abspath ../..)
ZOO                     ?= $(ROOT_DIR)/zoo
# Destination for build results
BUILD_BASE_DIR          ?= build
# Mark the default target
DEFAULT_BUILD_TARGET    ?= $(PROJECT_NAME)

# Configure how image is programmed to target device
PROGRAM_IMAGE           ?= $(BUILD_DIR)/$(PROJECT_NAME).bin
PROGRAM_DEST_ADDR       ?= $(APP_START)

# Silabs SDK location and version, due to licensing terms, the SDK is not
# distributed with this project and must be installed with Simplicity Studio.
# The variable needs to point at the subdirectory with the version number, set
# it in Makefile.private or through the environment.
SILABS_SDKDIR           ?= $(HOME)/SimplicityStudio_v5/developer/sdks/gecko_sdk_suite/v3.2

# Pull in the developer's private configuration overrides and settings
-include Makefile.private

# _______________________ Non-overridable configuration _______________________

BUILD_DIR                = $(BUILD_BASE_DIR)/$(BUILD_TARGET)
BUILDSYSTEM_DIR         := $(ZOO)/thinnect.node-buildsystem/make
PLATFORMS_DIRS          := $(ZOO)/thinnect.node-buildsystem/make $(ZOO)/thinnect.dev-platforms/make $(ZOO)/thinnect.smenete-platforms/make
PHONY_GOALS             := all clean
TARGETLESS_GOALS        += clean
UUID_APPLICATION        := d709e1c5-496a-4d31-8957-f389d7fdbb71

VERSION_BIN             := $(shell printf "%02X" $(VERSION_MAJOR))$(shell printf "%02X" $(VERSION_MINOR))$(shell printf "%02X" $(VERSION_PATCH))
VERSION_STR             := "$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)"$(VERSION_DEVEL)
SW_MAJOR_VERSION        := $(VERSION_MAJOR)
SW_MINOR_VERSION        := $(VERSION_MINOR)
SW_PATCH_VERSION        := $(VERSION_PATCH)
BUILD_TIMESTAMP         := $(shell date '+%s')
IDENT_TIMESTAMP         := $(BUILD_TIMESTAMP)

# NODE_PLATFORM_DIR is used by targets to add components to INCLUDES and SOURCES
NODE_PLATFORM_DIR       := $(ZOO)/thinnect.node-platform

# ______________ Build components - sources and includes _______________________

INCLUDES += -I./inc/i2c \
            -I./inc/gpio/ \
            -I./inc/ldma/ \
            -I./inc/mma8653fc/

# Project sources
SOURCES += src/i2c/i2c_config.c \
           src/gpio/gpio_config.c \
           src/mma8653fc/mma8653fc_driver.c

#TODO a nice lll version would be good
ifeq ($(USE_LLL_LOGGING),1)
    SOURCES += main_ldma.c
else
    SOURCES += main_ldma.c \
               src/ldma/ldma_config.c \
               src/ldma/ldma_descriptors.c
endif

# FreeRTOS
FREERTOS_DIR ?= $(ZOO)/FreeRTOS-Kernel
FREERTOS_INC = -I$(FREERTOS_DIR)/include \
               -I$(ZOO)/thinnect.cmsis-freertos/CMSIS_5/CMSIS/RTOS2/Include \
               -I$(ZOO)/thinnect.cmsis-freertos/CMSIS-FreeRTOS/CMSIS/RTOS2/FreeRTOS/Include \
               -I$(ZOO)/thinnect.cmsis-freertos/$(MCU_ARCH)

FREERTOS_SRC = $(wildcard $(FREERTOS_DIR)/*.c) \
               $(ZOO)/thinnect.cmsis-freertos/CMSIS-FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/cmsis_os2.c

INCLUDES += $(FREERTOS_PORT_INC) $(FREERTOS_INC)
SOURCES += $(FREERTOS_PORT_SRC) $(FREERTOS_SRC)

# CMSIS_CONFIG_DIR is used to add default CMSIS and FreeRTOS configs to INCLUDES
CMSIS_CONFIG_DIR ?= $(ZOO)/thinnect.cmsis-freertos/$(MCU_ARCH)/config

# Silabs EMLIB, RAIL, radio
INCLUDES += \
    -I$(SILABS_SDKDIR)/hardware/kit/common/drivers \
    -I$(SILABS_SDKDIR)/platform/halconfig/inc/hal-config \
    -I$(SILABS_SDKDIR)/platform/emlib/inc \
    -I$(SILABS_SDKDIR)/platform/emdrv/sleep/inc \

SOURCES += \
    $(SILABS_SDKDIR)/util/silicon_labs/silabs_core/queue/circular_queue.c \
    $(SILABS_SDKDIR)/hardware/kit/common/drivers/retargetserial.c \
    $(SILABS_SDKDIR)/hardware/kit/common/drivers/retargetio.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_system.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_core.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_emu.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_cmu.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_rmu.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_gpio.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_i2c.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_adc.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_iadc.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_ldma.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_usart.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_msc.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_rtcc.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_timer.c \
    $(SILABS_SDKDIR)/platform/emlib/src/em_wdog.c \
    $(SILABS_SDKDIR)/platform/emdrv/sleep/src/sleep.c \
    $(NODE_PLATFORM_DIR)/common/sys_panic.c \

ifeq ($(USE_LLL_LOGGING),1)
    # logging
    CFLAGS += -DLOGGER_TIMESTAMP
    CFLAGS  += -DLOGGER_FWRITE
    SOURCES += $(NODE_PLATFORM_DIR)/silabs/logger_fwrite.c
    CFLAGS  += -DLOGGER_LDMA_BUFFER_LENGTH=16384
    CFLAGS  += -DLOGGER_LDMA
    SOURCES += $(NODE_PLATFORM_DIR)/silabs/logger_ldma.c
    SOURCES += $(ZOO)/thinnect.lll/logging/loggers_ext.c
    INCLUDES += -I$(ZOO)/thinnect.lll/logging
else
    # This needs to be included because some source files have included it (though they shouldn't)
    INCLUDES += -I$(ZOO)/thinnect.lll/logging
endif


# device signature
INCLUDES += -I$(ZOO)/thinnect.device-signature/signature \
            -I$(ZOO)/thinnect.device-signature/area
SOURCES  += $(ZOO)/thinnect.device-signature/signature/DeviceSignature.c \
            $(ZOO)/thinnect.device-signature/area/silabs/SignatureArea.c

# Generally useful external tools
INCLUDES += -I$(ZOO)/lammertb.libcrc/include \
            -I$(ZOO)/jtbr.endianness \
            -I$(ZOO)/graphitemaster.incbin
SOURCES += $(ZOO)/lammertb.libcrc/src/crcccitt.c

# platform stuff - watchdog, io etc...
INCLUDES += -I$(NODE_PLATFORM_DIR)/include

# ------------------------------------------------------------------------------

# Pull in the grunt work
include $(BUILDSYSTEM_DIR)/Makerules
# ------------------------------------------------------------------------------

# Print some build parameters
$(info DEFAULT_AM_ADDR=$(DEFAULT_AM_ADDR))
$(info DEFAULT_RADIO_CHANNEL=$(DEFAULT_RADIO_CHANNEL))
$(info DEFAULT_PAN_ID=$(DEFAULT_PAN_ID))
$(info DEFAULT_RFPOWER_DBM=$(DEFAULT_RFPOWER_DBM))
# ------------------------------------------------------------------------------

$(call passVarToCpp,CFLAGS,VERSION_MAJOR)
$(call passVarToCpp,CFLAGS,VERSION_MINOR)
$(call passVarToCpp,CFLAGS,VERSION_PATCH)
$(call passVarToCpp,CFLAGS,VERSION_STR)
$(call passVarToCpp,CFLAGS,SW_MAJOR_VERSION)
$(call passVarToCpp,CFLAGS,SW_MINOR_VERSION)
$(call passVarToCpp,CFLAGS,SW_PATCH_VERSION)
$(call passVarToCpp,CFLAGS,IDENT_TIMESTAMP)

$(call passVarToCpp,CFLAGS,DEFAULT_AM_ADDR)
$(call passVarToCpp,CFLAGS,DEFAULT_RADIO_CHANNEL)
$(call passVarToCpp,CFLAGS,DEFAULT_PAN_ID)

UUID_APPLICATION_BYTES = $(call uuidToCstr,$(UUID_APPLICATION))
$(call passVarToCpp,CFLAGS,UUID_APPLICATION_BYTES)

$(call passVarToCpp,CFLAGS,BASE_LOG_LEVEL)

# _______________________________ Project rules _______________________________

all: $(BUILD_DIR)/$(PROJECT_NAME).bin

# header.bin should be recreated if a build takes place
$(OBJECTS): $(BUILD_DIR)/header.bin

$(BUILD_DIR)/$(PROJECT_NAME).elf: Makefile | $(BUILD_DIR)

$(BUILD_DIR)/header.bin: Makefile | $(BUILD_DIR)
	$(call pInfo,Creating application header block [$@])
	$(HEADEREDIT) -c -v softtype,1 -v firmaddr,$(APP_START) -v firmsizemax,$(APP_MAX_LEN) \
	    -v version,$(VERSION_STR) -v versionbin,$(VERSION_BIN) \
	    -v uuid,$(UUID_BOARD) -v uuid2,$(UUID_PLATFORM) -v uuid3,$(UUID_APPLICATION) \
	    -v timestamp,$(BUILD_TIMESTAMP) \
	    -v name,$(PROJECT_NAME) \
	    -v size -v crc "$@"

$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJECTS)
	$(call pInfo,Linking [$@])
	$(HIDE_CMD)$(CC) $(CFLAGS) $(INCLUDES) $(OBJECTS) $(LDLIBS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(call pInfo,Exporting [$@])
	$(HIDE_CMD)$(TC_SIZE) --format=Berkeley $<
	$(HIDE_CMD)$(TC_OBJCOPY) --strip-all -O binary "$<" "$@"
	$(HIDE_CMD)$(HEADEREDIT) -v size -v crc $@

$(PROJECT_NAME): $(BUILD_DIR)/$(PROJECT_NAME).bin

# _______________________________ Utility rules ________________________________

$(BUILD_DIR):
	$(call pInfo,Creating [$@])
	@mkdir -p "$@"

clean:
	$(call pInfo,Nuking everything in [$(BUILD_BASE_DIR)])
	@-rm -rf "$(BUILD_BASE_DIR)"

.PHONY: $(PHONY_GOALS)
