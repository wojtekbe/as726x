obj-m += as726x.o

all:
	make -C ../linux-3.10 M=$(PWD) modules

clean:
	make -C ../linux-3.10 M=$(PWD) clean
