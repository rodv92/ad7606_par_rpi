obj-m +=ad7606_par.o
CFLAGS_ad7606_par.o := -DDEBUG

all: module dt
	echo Built Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: overlay.dts
	cpp -nostdinc -I /usr/src/linux-headers-6.12.34+rpt-common-rpi/include/ -I arm64  -undef -x assembler-with-cpp  ad7606.dts ad7606.dts.preprocessed

	dtc -@ -Hepapr -I dts -O dtb -o ad7606.dtbo ad7606.dts.preprocessed

clean: 
	make -c /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf ad7606.dtbo
