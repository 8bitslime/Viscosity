#Build viscosity physics library
INC=-Iinclude/ -IMMath/

viscosity: viscosity.o shape.o world.o
	ar rcs libviscosity.a src/viscosity.o src/shape.o src/world.o

viscosity.o: src/viscosity.c
	gcc $(INC) -c -o src/viscosity.o src/viscosity.c

shape.o: src/shape.c
	gcc $(INC) -c -o src/shape.o src/shape.c

world.o: src/world.c
	gcc $(INC) -c -o src/world.o src/world.c

clean:
	rm src/*.o libviscosity.a
