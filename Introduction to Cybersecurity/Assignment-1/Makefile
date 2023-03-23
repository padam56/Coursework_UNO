CC=gcc
CFLAGS_1=-m32 -march=i386 -O0 -fcf-protection=none -z execstack -fno-stack-protector -fno-pie -D_FORTIFY_SOURCE=0 --save-temps -fno-asynchronous-unwind-tables -mpreferred-stack-boundary=2 -w

fmt: fmt.c
	$(CC) $(CFLAGS_1) -g -o fmt fmt.c

a1: a1.c
	$(CC) $(CFLAGS_1) -g -o a1 a1.c

run:
	env -i SECRET_ENV=s3cr3t ./a1

clean:
	rm *.i *.o *.s a1 fmt
	
edit:
	geany Makefile fmt.c a1.c &

a1s: a1-sol.c
	$(CC) $(CFLAGS_1) -g -o a1s a1-sol.c
	
runs:
	env -i SECRET=s3cr3t ./a1s
	
