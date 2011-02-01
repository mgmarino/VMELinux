include ../config.mk

SOURCES = $(wildcard *.cc) 
OBJECTS = $(SOURCES:.cc=.o)
HEADERS = $(wildcard *.hh)
INSTALLDIR = /usr/local/universe

FLAGS = -O2 -Wall -I../driver
#FLAGS = -g -Wall -I../driver

LIB = libuniverse_api.a

all: depend $(LIB)

.depend depend:
	@echo Checking Dependencies...
	@g++ -M $(FLAGS) *.cc > .depend

%.o: %.cc
	@echo compiling $*
	g++ -c $(FLAGS) $<

$(LIB): $(OBJECTS)
	$(AR) -r $(LIB) $(OBJECTS)

install: $(LIB)
	@echo "Beginning installation..."
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLDIR) || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLDIR)/include || exit $$? 
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -d $(INSTALLDIR)/lib || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 $(LIB) $(INSTALLDIR)/lib || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 universe_api.h $(INSTALLDIR)/include || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 ../driver/universe.h $(INSTALLDIR)/include || exit $$?
	$(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 ../driver/ConcurrentVX40x.h $(INSTALLDIR)/include || exit $$?
	@for i in $(HEADERS); do ($(INSTALL) -o $(INSTALLUSR) -g $(INSTALLGRP) -m 644 $$i $(INSTALLDIR)/include) || exit $$?; done
	@echo "Installation complete."
 
uninstall:
	@rm -f $(INSTALLDIR)/include/universe_api.h $(INSTALLDIR)/include/universe.h $(INSTALLDIR)/lib/$(LIB)
	@rm -f $(INSTALLDIR)/include/ConcurrentVX40x.h
	@for i in $(HEADERS); do rm -f $(INSTALLDIR)/include/$$i; done
# now try to get rid of the folders
	@find $(INSTALLDIR) -depth -type d -empty -exec rm -r {} \;

clean :
	rm -f $(LIB) *.o .depend

ifneq ($(MAKECMDGOALS), clean)
-include .depend
endif
