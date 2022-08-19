# Makefile for libs/c
# $Id$

# Compile options
CC = gcc
CXX = g++
#CFLAGS = -O2 -Wall -Werror -W $(INCLUDES)
CFLAGS = -g -O0 -Wall -Werror -W -I../include/c $(INCLUDES)
CXXFLAGS = $(CFLAGS) -I../include/cpp
INCLUDES =
LDFLAGS =
LDLIBS =

# Target
URG_C_LIB_STATIC = liburg_c.a
URG_C_LIB_SHARED = $(shell if test `echo $(OS) | grep Windows`; then echo "urg_c.dll"; else echo "liburg_c.so"; fi)
URG_CPP_LIB_STATIC = liburg_cpp.a
URG_CPP_LIB_SHARED = $(shell if test `echo $(OS) | grep Windows`; then echo "urg_cpp.dll"; else echo "liburg_cpp.so"; fi)

TARGET = $(URG_C_LIB_STATIC) $(URG_CPP_LIB_STATIC) \
		 $(URG_C_LIB_SHARED) $(URG_CPP_LIB_SHARED)

OBJ_C = urg_sensor.o urg_utils.o urg_debug.o urg_connection.o \
        urg_ring_buffer.o urg_serial.o urg_serial_utils.o urg_tcpclient.o
OBJ_CPP = ticks.o Urg_driver.o

CFLAGS = -g -O2 $(INCLUDES) -I../include/c -fPIC
CXXFLAGS = $(CFLAGS) -I../include/cpp
LDLIBS = -lm $(shell if test `echo $(OS) | grep Windows`; then echo "-lwsock32 -lsetupapi"; fi)

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET)

depend :
	makedepend -Y -- $(INCLUDES) -- $(wildcard *.h *.c)

.PHONY : all clean depend
######################################################################
$(URG_C_LIB_STATIC) : $(OBJ_C)
	$(AR) rv $@ $(OBJ_C)

$(URG_C_LIB_SHARED) : $(OBJ_C)
	$(CC) $(CFLAGS) -shared -o $@ $(OBJ_C) $(LDLIBS)

$(URG_CPP_LIB_STATIC) : $(OBJ_C) $(OBJ_CPP)
	$(AR) rv $@ $(OBJ_C) $(OBJ_CPP)

$(URG_CPP_LIB_SHARED) : $(OBJ_C) $(OBJ_CPP)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_C) $(OBJ_CPP) $(LDLIBS)

urg_serial_utils.o : urg_serial_utils_windows.c urg_serial_utils_linux.c
