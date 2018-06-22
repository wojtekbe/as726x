obj-m += as726x.o

all:
	make -C ../linux-3.10 M=$(PWD) modules

fcat:
	$(CROSS_COMPILE)gcc fcat.c -o fcat

clean:
	make -C ../linux-3.10 M=$(PWD) clean
	-rm -rf fcat
