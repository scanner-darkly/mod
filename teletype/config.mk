#
# Copyright (c) 2009-2010 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop


# project name
THIS = teletype

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ../xdk-asf-3.17.0

# Target CPU architecture: ap, ucr1, ucr2 or ucr3
# ARCH = ucr1
ARCH = ucr2

# Target part: none, ap7xxx or uc3xxxxx
# PART = uc3b064
# PART = uc3b0256
PART = uc3b0512

# Target device flash memory details (used by the avr32program programming
# tool: [cfi|internal]@address
FLASH = internal@0x80000000

# Clock source to use when programming; xtal, extclk or int
PROG_CLOCK = int

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = $(THIS).elf

# List of C source files.
CSRCS = \
       ../$(THIS)/main.c    \
       ../$(THIS)/table.c    \
       ../$(THIS)/teletype.c    \
       ../system/adc.c     \
       ../system/events.c     \
       ../system/fix.c     \
       ../system/font.c     \
       ../system/i2c.c \
       ../system/init.c \
       ../system/interrupts.c \
       ../system/kbd.c \
       ../system/monome.c \
       ../system/region.c \
       ../system/screen.c \
       ../system/timers.c \
       ../system/usb.c \
       ../system/util.c \
       ../system/usb/ftdi/ftdi.c \
       ../system/usb/ftdi/uhi_ftdi.c \
       ../system/usb/hid/hid.c \
       ../system/usb/hid/uhi_hid.c \
       avr32/drivers/adc/adc.c                            \
       avr32/drivers/flashc/flashc.c                      \
       avr32/drivers/gpio/gpio.c                          \
       avr32/drivers/intc/intc.c                          \
       avr32/drivers/pm/pm.c                              \
       avr32/drivers/pm/pm_conf_clocks.c                  \
       avr32/drivers/pm/power_clocks_lib.c                \
       avr32/drivers/spi/spi.c                            \
       avr32/drivers/tc/tc.c                              \
       avr32/drivers/twi/twi.c                            \
       avr32/drivers/usart/usart.c                        \
       avr32/drivers/usbb/usbb_host.c                     \
       avr32/services/fs/fat/fat.c                        \
       avr32/services/fs/fat/fat_unusual.c                \
       avr32/services/fs/fat/file.c                       \
       avr32/services/fs/fat/navigation.c                 \
       avr32/utils/debug/print_funcs.c                    \
       common/services/storage/ctrl_access/ctrl_access.c  \
       common/services/usb/class/msc/host/uhi_msc.c       \
       common/services/usb/class/msc/host/uhi_msc_mem.c   \
       common/services/spi/uc3_spi/spi_master.c           \
       common/services/usb/uhc/uhc.c                      \
       common/services/clock/uc3b0_b1/sysclk.c  

# List of assembler source files.
ASSRCS = \
       avr32/utils/startup/trampoline_uc3.S               \
       avr32/drivers/intc/exception.S                     \
              

# List of include paths.
INC_PATH = \
       ../$(THIS)           \
       ../system                                        \
       ../system/conf      \
       ../system/usb \
       ../system/usb/ftdi \
       ../system/usb/hid \
       avr32/boards                                       \
       avr32/drivers/cpu/cycle_counter                    \
       avr32/drivers/flashc                               \
       avr32/drivers/gpio                                 \
       avr32/drivers/intc                                 \
       avr32/drivers/pm                                   \
       avr32/drivers/spi                                  \
       avr32/drivers/tc                                   \
       avr32/drivers/twi                                  \
       avr32/drivers/usart                                \
       avr32/drivers/usbb                                 \
       avr32/utils                                        \
       avr32/utils/debug                                  \
       avr32/utils/preprocessor                           \
       avr32/services/fs/fat                              \
       common/boards                                      \
       common/boards/user_board                         \
       common/services/storage/ctrl_access                \
       common/services/clock                              \
       common/services/delay                              \
       common/services/usb/                             \
       common/services/usb/uhc                          \
       common/services/clock                              \
       common/services/storage/ctrl_access                \
       common/services/usb/class/msc                      \
       common/services/usb/class/msc/host                 \
       common/services/usb/class/hid                      \
       common/services/spi/uc3_spi                      \
       common/utils                                       

# Additional search paths for libraries.
LIB_PATH = 

# List of libraries to use during linking.
LIBS = 

# Path relative to top level directory pointing to a linker script.
# LINKER_SCRIPT = avr32/utils/linker_scripts/at32uc3b/0256/gcc/link_uc3b0256.lds
# LINKER_SCRIPT = avr32/drivers/flashc/flash_example/at32uc3b0256_evk1101/link_uc3b0256.lds
# LINKER_SCRIPT = ../system/link_uc3b0256.lds
LINKER_SCRIPT = ../system/link_uc3b0512.lds


# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -Os

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = -fshort-enums

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
      -D BOARD=USER_BOARD -D UHD_ENABLE                             

# Extra flags to use when linking
LDFLAGS = \
        -Wl,-e,_trampoline

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 