all:
	gcc src/pf.c -c -o src/pf.o -I./include/PF -Wall -Werror -pedantic -std=c11 -ffast-math -g -O3 -lc -lm -lML -I/usr/include/ML
	ar rvs libPF.a src/pf.o
clean:
	rm libPF.a src/pf.o
install:
	mkdir -p /usr/include/PF
	cp include/PF/*.h /usr/include/PF
	cp libPF.a /usr/lib/
