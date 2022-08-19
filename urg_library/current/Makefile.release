PREFIX = /usr/local
#PREFIX = /mingw

include build_rule.mk

CONFIG_FILE = urg_c-config
CONFIG_FILE_CPP = urg_cpp-config
INCLUDE_DIR_C = $(PREFIX)/include/urg_c/
INCLUDE_DIR_CPP = $(PREFIX)/include/urg_cpp/
URGLIB_C_STATIC = liburg_c.a
URGLIB_C_SHARED = $(shell if test `echo $(OS) | grep Windows`; then echo "urg_c.dll"; else echo "liburg_c.so"; fi)
URGLIB_CPP_STATIC = liburg_cpp.a
URGLIB_CPP_SHARED = $(shell if test `echo $(OS) | grep Windows`; then echo "urg_cpp.dll"; else echo "liburg_cpp.so"; fi)
S_PREFIX = $(shell echo "$(PREFIX)" | sed "s/\//\\\\\\\\\//g")
S_LIBS = $(shell if test `echo $(OS) | grep Windows`; then echo "-lwsock32 -lsetupapi"; else if test `echo $(OS) | grep Mac`; then echo ""; else echo "-lrt"; fi; fi)
all : $(CONFIG_FILE) $(CONFIG_FILE_CPP)
	cd src/ && $(MAKE)
	cd samples/ && $(MAKE)

clean :
	$(RM) $(CONFIG_FILE)
	$(RM) $(CONFIG_FILE_CPP)
	cd src/ && $(MAKE) clean
	cd samples/ && $(MAKE) clean

install : all
	install -d $(INCLUDE_DIR_C)
	install -d $(INCLUDE_DIR_CPP)
	install -m 644 include/c/*.h $(INCLUDE_DIR_C)
	install -m 644 include/cpp/*.h $(INCLUDE_DIR_CPP)
	install -d $(PREFIX)/lib/
	install -m 644 src/$(URGLIB_C_STATIC) $(PREFIX)/lib/
	install -m 644 src/$(URGLIB_C_SHARED) $(PREFIX)/lib/
	install -m 644 src/$(URGLIB_CPP_STATIC) $(PREFIX)/lib/
	install -m 644 src/$(URGLIB_CPP_SHARED) $(PREFIX)/lib/
	install -d $(PREFIX)/bin/
	install -m 755 $(CONFIG_FILE) $(PREFIX)/bin/
	install -m 755 $(CONFIG_FILE_CPP) $(PREFIX)/bin/

uninstall :
	$(RM) -r $(INCLUDE_DIR_C)
	$(RM) -r $(INCLUDE_DIR_CPP)
	$(RM) $(PREFIX)/lib/$(URGLIB_C_STATIC)
	$(RM) $(PREFIX)/lib/$(URGLIB_C_SHARED)
	$(RM) $(PREFIX)/lib/$(URGLIB_CPP_STATIC)
	$(RM) $(PREFIX)/lib/$(URGLIB_CPP_SHARED)
	$(RM) $(PREFIX)/bin/$(CONFIG_FILE)
	$(RM) $(PREFIX)/bin/$(CONFIG_FILE_CPP)

$(CONFIG_FILE) : $(CONFIG_FILE).in Makefile
	cat $(CONFIG_FILE).in | sed "s/PREFIX/$(S_PREFIX)/g" | sed "s/LIBS/$(S_LIBS)/g" > $(CONFIG_FILE)
$(CONFIG_FILE_CPP) : $(CONFIG_FILE_CPP).in Makefile
	cat $(CONFIG_FILE_CPP).in | sed "s/PREFIX/$(S_PREFIX)/g" | sed "s/LIBS/$(S_LIBS)/g" > $(CONFIG_FILE_CPP)
