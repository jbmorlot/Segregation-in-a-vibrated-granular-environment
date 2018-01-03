
CC = gcc
OBJS = main.o vector2D.o particle.o pair.o world.o integration_verlet.o draw.o utils.o integration_RK2.o integration_RK4.o corrector_kernel.o
HEADER = vector2D.h particle.h pair.h world.h integration_verlet.h integration.h draw.h defines.h utils.h integration_RK2.h integration_RK4.h corrector_kernel.h
EXEC = projet
CFLAGS = -W -Werror
OPTS = -g -O3 -fexpensive-optimizations
LDFLAGS = -lm -L/usr/lib/SDL -lSDL #-lSDL_ttf

all: $(EXEC)

$(EXEC): $(OBJS) $(HEADER)
	$(CC) $(OBJS) -o $(EXEC) $(LDFLAGS)

%.o: %.c $(HEADER)
	$(CC) -o $@ -c $< $(CFLAGS) $(OPTS)

.PHONY: clean mrproper archive

clean:
	-rm *.o

mrproper: clean
	-rm $(EXEC) *~

archive: mrproper
	tar czf ../projet.tar.gz *


