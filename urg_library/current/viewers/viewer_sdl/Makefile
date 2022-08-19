# viewers/viewer_sdl/

# Compile options
CC = gcc
CFLAGS = -g -O0 -Wall -Werror -W `sdl-config --cflags` `urg_c-config --cflags`
LDFLAGS = -L/usr/X11/lib
ifeq ($(OS),Windows_NT)
LDLIBS = `sdl-config --libs` -lopengl32 `urg_c-config --libs`
else
LDLIBS = `sdl-config --libs` -lGL `urg_c-config --libs`
endif


TARGET = viewer_sdl

all : $(TARGET)

clean :
	$(RM) $(TARGET) *.o

depend :
	makedepend -Y -- $(INCLUDES) -- $(wildcard *.h *.c)

.PHONY : all clean depend
######################################################################

viewer_sdl : plotter_sdl.o $(REQUIRE_LIB)

# DO NOT DELETE

plotter_sdl.o: plotter_sdl.h
viewer_sdl.o: plotter_sdl.h
