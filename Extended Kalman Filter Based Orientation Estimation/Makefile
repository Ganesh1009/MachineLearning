export DEVKIT=pca10059
export SOFTDEVICE=mbr
export PROJ_DIR := $(PWD)
export OUTPUT_DIRECTORY := $(PROJ_DIR)/_build

.checkSDK:
	test -f $(PROJ_DIR)/../.setupdone

all: .checkSDK
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc all

help:
	@echo Top Level Makefile that builds following configuration:
	@echo  - $(DEVKIT)
	@echo  - $(SOFTDEVICE)
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc help

clean: .checkSDK
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc clean

# Flash the program
flash: .checkSDK
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc flash

flash_softdevice: .checkSDK
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc flash_softdevice

erase: .checkSDK
	$(MAKE) -C $(DEVKIT)/$(SOFTDEVICE)/armgcc erase
