all:
	cc demo.c -o demo -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lml -I../include ../libpf.a `pkg-config --cflags --libs sdl2` -D_GNU_SOURCE
clean:
	rm demo
gcw0:
	mipsel-gcw0-linux-uclibc-cc demo.c -o demo -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lml -lpf `/opt/gcw0-toolchain/usr/bin/pkg-config --cflags --libs sdl2` -D_GNU_SOURCE

opk:
	mksquashfs release pf_demo.opk -all-root -noappend -no-exports -no-xattrs
