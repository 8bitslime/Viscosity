#Build viscosity physics library
OPTIONS = -std=c99 -Iinclude/ -IMMath/

viscosity: src/viscosity.o src/shape.o src/world.o
	ar rcs libviscosity.a $^

src/viscosity.o: src/viscosity.c
	gcc $(OPTIONS) -c -o src/viscosity.o src/viscosity.c

src/shape.o: src/shape.c
	gcc $(OPTIONS) -c -o src/shape.o src/shape.c

src/world.o: src/world.c
	gcc $(OPTIONS) -c -o src/world.o src/world.c

.PHONY: clean
clean:
	rm -f src/*.o libviscosity.a
