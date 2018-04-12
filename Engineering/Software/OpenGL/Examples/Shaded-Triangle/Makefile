CPPFLAGS=$(shell sdl2-config --cflag) $(EXTRA_CPPFLAGS)
LDLIBS=$(shell sdl2-config --libs) -lGLEW $(EXTRA_LDLIBS)
EXTRA_LDLIBS?=-lGL
all: triangle
clean: rm -f *.o triangle
.PHONY: all clean
