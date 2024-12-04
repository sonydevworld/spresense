obj-m = spr_camera.o

all: spr_camera.ko

spr_camera.ko: spr_camera.c spr_camera.h
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

insmod: spr_camera.ko
	sudo insmod spr_camera.ko

rmmod:
	sudo rmmod spr_camera
