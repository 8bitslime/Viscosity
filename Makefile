#Build viscosity physics library
CC := gcc
CFLAGS := -std=c99 -Iinclude/ -IMMath/

FILES := src/viscosity.o src/shape.o src/world.o

libviscosity.a: $(FILES)
	ar rcs libviscosity.a $(FILES)

.PHONY: rebuild
rebuild:
	touch -c src/*.c
	$(MAKE)

.PHONY: clean
clean:
	rm -f src/*.o libviscosity.a
