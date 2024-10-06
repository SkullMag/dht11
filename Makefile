# The name of the module
obj-m += dht11.o

# The directory with kernel headers (for Raspberry Pi, it should point to your kernel's source)
KDIR := /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Default target to build the module
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean target to remove generated files
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f *.mod *.mod.c *.o *.ko *.order *.symvers .*.cmd

# Load the module
load:
	sudo insmod dht11.ko

# Unload the module
unload:
	sudo rmmod dht11

