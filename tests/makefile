all:
	cc main.c -o tests -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lml -lcmocka -I../include ../libpf.a `pkg-config --cflags --libs sdl2` -D_GNU_SOURCE

run:	all
	./tests

clean:
	rm tests
