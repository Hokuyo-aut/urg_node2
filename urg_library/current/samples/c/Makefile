# Makefile for c/urg/samples
# $Id$

# Compile options
SRCDIR = ../../src
INCLUDEDIR = ../../include/c
CC = gcc
CFLAGS = -g -O0 -Wall -Werror -W $(INCLUDES)
INCLUDES = -I$(INCLUDEDIR)
LDFLAGS =
LDLIBS = -lm `/bin/sh ld_wsock.sh` `/bin/sh ld_setupapi.sh`

# Target
TARGET = \
	get_distance \
	get_distance_handshake \
	get_distance_intensity \
	get_multiecho \
	get_multiecho_intensity \
	calculate_xy \
	sync_time_stamp \
	sensor_parameter \
	find_port \
	timeout_test \
	reboot_test \
	angle_convert_test \

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET) *.exe

depend :
	makedepend -Y -- $(INCLUDES) -- $(wildcard *.h *.c)

.PHONY : all depend clean

######################################################################
REQUIRE_LIB = $(SRCDIR)/liburg_c.a
$(REQUIRE_LIB) : $(wildcard $(SRCDIR)/*.[ch])
	cd $(@D)/ && $(MAKE) $(@F)

get_distance get_distance_handshake get_distance_intensity get_multiecho get_multiecho_intensity calculate_xy sync_time_stamp sensor_parameter timeout_test reboot_test angle_convert_test : open_urg_sensor.o $(REQUIRE_LIB)
find_port : $(REQUIRE_LIB)
