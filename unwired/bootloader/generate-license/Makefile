all:
	gcc -std=c99 -c generate-license.c -o generate-license.o
	gcc -std=c99 -c ecc.c -o ecc.o
	gcc -o generate-license generate-license.o ecc.o
	rm -f generate-license.o ecc.o