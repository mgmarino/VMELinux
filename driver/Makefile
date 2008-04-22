include $(PWD)/../config.mk

obj-m := universe.o 

KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)
BASENAME = $(basename $(obj-m))
INSTALLDIR = /lib/modules/$(shell uname -r)/kernel/drivers
INSTALLSCRIPTSDIR = /usr/local/universe/scripts
INSTALLSCRIPTSDIRSED = $(subst /,\/,$(INSTALLSCRIPTSDIR))

$(BASENAME).ko default: $(BASENAME).c
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: $(BASENAME).ko 
	@echo "Beginning driver installation..."
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLDIR)/vme || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLSCRIPTSDIR) || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 $(BASENAME).ko $(INSTALLDIR)/vme || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 744 install_universe_driver.sh $(INSTALLSCRIPTSDIR) || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 744 remove_universe_driver.sh $(INSTALLSCRIPTSDIR) || exit $$?
	@sed 's/@INSTALLSCRIPTSDIR@/$(INSTALLSCRIPTSDIRSED)/' < $(BASENAME).input > $(BASENAME).temp
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -T $(BASENAME).temp /etc/init.d/$(BASENAME) || exit $$?
	@chkconfig --list | awk  '{print $1}' | grep -q $(BASENAME); if [ "${?}" != 0 ]; then chkconfig --add $(BASENAME); fi
	@chkconfig --level 0123456 $(BASENAME) off
	@chkconfig --level 2345 $(BASENAME) on
	@rm -f $(BASENAME).temp
	@echo "Installation complete."
	@echo "Be sure to begin the service with: service $(BASENAME) start"

uninstall:
	@echo "Beginning uninstall..."
	@rm -f $(INSTALLDIR)/vme/$(BASENAME).ko	
	@rm -f /etc/init.d/$(BASENAME)
	@chkconfig --list | awk  '{print $1}' | grep -q $(BASENAME); if [ "${?}" == 0 ]; then chkconfig --del $(BASENAME); fi
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
