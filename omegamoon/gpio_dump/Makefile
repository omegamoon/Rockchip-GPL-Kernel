#############################################################################
#
#	Makefile for building the gpio module, lib, and app
#
#############################################################################

SUBDIRS = lib app

CLEAN_SUBDIRS = $(SUBDIRS)

all: $(SUBDIRS)
clean: $(SUBDIRS)

.PHONY: $(SUBDIRS)

$(SUBDIRS) : 
	$(MAKE) -I../toolchain/linaro/arm-linux-gnueabihf/include/c++/4.8.2/tr1 -C $@ $(MAKECMDGOALS) 

