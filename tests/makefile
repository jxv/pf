all:
	cc *.c -o tests -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lml -lcmocka -I../include ../libpf.a -D_GNU_SOURCE

run:	all
	./tests

clean:
	rm tests
