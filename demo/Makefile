all:
	gcc demo.c -o demo -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lML -lPF `pkg-config --cflags --libs sdl2` -I/usr/include/ML -I/usr/include/PF
clean:
	rm demo
