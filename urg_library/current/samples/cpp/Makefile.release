TARGET = sensor_parameter get_distance get_distance_handshake get_distance_intensity get_multiecho get_multiecho_intensity sync_time_stamp

SRCDIR = ../../src
URG_CPP_LIB = ../../src/liburg_cpp.a

include ../../build_rule.mk

CXXFLAGS = -O2 $(INCLUDES) -I../../include/cpp
LDLIBS = -lm $(shell if test `echo $(OS) | grep Windows`; then echo "-lwsock32 -lsetupapi"; else if test `uname -s | grep Darwin`; then echo ""; else echo "-lrt"; fi; fi) -L$(SRCDIR)

#

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET) *.exe

$(TARGET) : Connection_information.o $(URG_CPP_LIB)

$(URG_CPP_LIB) :
	cd $(@D)/ && $(MAKE) $(@F)
