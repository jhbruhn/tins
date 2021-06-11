# System specifications
F_CRYSTAL      = 8000000L
F_CPU          = 168000000L
SYSCLOCK       = SYSCLK_FREQ_168MHz
FAMILY         = f4xx
# USB            = enabled

#APPLICATION_LARGE = TRUE
BOOTLOADER        = tins_bootloader

# Preferred upload command
UPLOAD_COMMAND  = upload_jtag_erase_first

# Packages to build
TARGET         = tins
PACKAGES       = tins \
		tins/drivers \
		stmlib/dsp \
		stmlib/utils \
		stmlib/system
RESOURCES      = tins/resources

TOOLCHAIN_PATH ?= /usr/

include stmlib/makefile.inc

