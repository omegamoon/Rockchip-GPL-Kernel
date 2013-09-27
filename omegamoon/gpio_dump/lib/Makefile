#############################################################################
#
#	Makefile for building the gpio libray
#
#############################################################################

LIBNAME		= gpio

SHARED_SONAME	= lib$(LIBNAME).so
SHARED_LIBNAME	= lib$(LIBNAME).so
STATIC_LIBNAME	= lib$(LIBNAME).a

TOOLCHAIN_BASE  = ../../toolchain/linaro
CROSS_COMPILE	?= $(TOOLCHAIN_BASE)/bin/arm-linux-gnueabihf-

CC		= $(CROSS_COMPILE)gcc
AR		= $(CROSS_COMPILE)ar

CFLAGS += \
	-std=gnu99 \
	-Wall \
	-Wimplicit \
	-Wpointer-arith \
	-Wswitch \
	-Wredundant-decls \
	-Wreturn-type \
	-Wshadow \
	-Wunused \
	-Wcast-qual \
	-Wnested-externs \
	-Wmissing-prototypes \
	-Wstrict-prototypes \
	-Wmissing-declarations \
	-I ../module 

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)

OBJ_FILES = user-gpio.o
STATIC_OBJ_FILES = $(addprefix static/,$(OBJ_FILES))
SHARED_OBJ_FILES = $(addprefix shared/,$(OBJ_FILES))

all : $(STATIC_LIBNAME) $(SHARED_LIBNAME)

$(SHARED_OBJ_FILES) : shared
$(STATIC_OBJ_FILES) : static

static shared :
	mkdir $@

.PHONY: %.d
%.d: ;

static/%.o : %.c %.d
	@echo "Compiling $< ..."
	$(COMPILE.c) $(DEP_OUTPUT_OPTION) $(OUTPUT_OPTION) $<

shared/%.o : %.c %.d
	@echo "Compiling $< ..."
	$(COMPILE.c) -fPIC $(DEP_OUTPUT_OPTION) $(OUTPUT_OPTION) $<

$(STATIC_LIBNAME) : $(STATIC_OBJ_FILES)
	@echo "Creating static library $@ ..."
	rm -f $@
	$(AR) $(ARFLAGS) $@ $^

$(SHARED_LIBNAME) : $(SHARED_OBJ_FILES)
	@echo "Creating shared library $@ ..."
	$(CC) -shared -rdynamic -Wl,-soname,$(SHARED_SONAME) -o $@ $^ 

clean:
	rm -rf shared static *.a *.so*

DEP_FILES = $(strip $(STATIC_OBJ_FILES:.o=.d) $(SHARED_OBJ_FILES:.o=.d))

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif

