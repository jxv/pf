all:
	gcc src/pf.c -c -o src/pf.o -I./include -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lml -D_GNU_SOURCE
	ar rvs libpf.a src/pf.o
clean:
	rm libpf.a src/pf.o
install:
	cp include/*.h /usr/include
	cp libpf.a /usr/lib/

gcw0:
	mipsel-gcw0-linux-uclibc-cc src/pf.c -c -o src/pf.o -I./include -Wall -Werror -pedantic -std=c11 -ffast-math -g -O2 -lc -lm -D_GNU_SOURCE -I./include
	mipsel-gcw0-linux-uclibc-ar rvs libpf.a src/*.o
install_gcw0:
	mkdir -p /opt/gcw0-toolchain/usr/mipsel-gcw0-linux-uclibc/sysroot/usr/include
	cp include/*.h /opt/gcw0-toolchain/usr/mipsel-gcw0-linux-uclibc/sysroot/usr/include
	cp libpf.a /opt/gcw0-toolchain/usr/mipsel-gcw0-linux-uclibc/sysroot/usr/lib
