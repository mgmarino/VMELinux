include $(PWD)/../config.mk

obj-m := universe.o 

OSSUFFIX := $(shell $(PWD)/detOS.sh)
KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)
BASENAME = $(basename $(obj-m))
INSTALLDIR = /lib/modules/$(shell uname -r)/kernel/drivers
INSTALLSCRIPTSDIR = /usr/local/universe/scripts
INSTALLSCRIPTSDIRSED = $(subst /,\/,$(INSTALLSCRIPTSDIR))
INSTALLINITDIR := $(shell if [ -d /etc/init.d ]; then echo "/etc/init.d"; else echo "/etc/rc.d"; fi)

.PHONY: setup_driver_auto uninstall_driver_auto

$(BASENAME).ko default: $(BASENAME).c
	$(MAKE) -C $(KDIR) M=$(PWD) modules

ifneq ($(OSSUFFIX),)
-include Makefile.$(OSSUFFIX)
else
setup_driver_auto:
	@echo "Driver is installed but not automatically setup."
	@echo "Please make the appropriate changes to the init.d/rc.d conf files."

uninstall_driver_auto:
	@echo "Driver is removed but not from the init.d/rc.d conf files."
	@echo "Please make the appropriate changes to the init.d conf files."
endif

install: $(BASENAME).ko 
	@echo "Beginning driver installation..."
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLDIR)/vme || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLSCRIPTSDIR) || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 $(BASENAME).ko $(INSTALLDIR)/vme || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 744 install_universe_driver.sh $(INSTALLSCRIPTSDIR) || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 744 remove_universe_driver.sh $(INSTALLSCRIPTSDIR) || exit $$?
	@sed 's/@INSTALLSCRIPTSDIR@/$(INSTALLSCRIPTSDIRSED)/' < $(BASENAME).input > $(BASENAME).temp
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -T $(BASENAME).temp $(INSTALLINITDIR)/$(BASENAME) || exit $$?
	@ echo "$(BASENAME) installed in $(INSTALLINITDIR)" 
	@rm -f $(BASENAME).temp
	@$(MAKE) setup_driver_auto
	@echo "Install complete."

uninstall:
	@echo "Beginning uninstall..."
	@rm -f $(INSTALLDIR)/vme/$(BASENAME).ko	
	@$(MAKE) uninstall_driver_auto
	@echo "Uninstall complete."

loaddriver:
	@echo "Loading driver..."
	@./install_universe_driver.sh

unloaddriver:
	@echo "Unloading driver..."
	@./remove_universe_driver.sh

check:
	@if [ -f /proc/$(BASENAME) ]; then cat /proc/$(BASENAME); else echo "Driver not loaded."; exit 1; fi

clean:
	rm -rf $(BASENAME).o Module.symvers $(BASENAME).ko* $(BASENAME).mod* .$(BASENAME).* .tmp_versions
