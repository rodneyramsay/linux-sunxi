
################################################################################
#
# Common Variables that already set:
#     LICHEE_KDIR
#     LICHEE_MOD_DIR
#     CROSS_COMPILE
#     ARCH
#
#################################################################################
PWD=$(shell pwd)

obj-y+=nand_interface.o
obj-y+=nfd/nand_blk.o
obj-y+=nfd/nand_osal_for_linux.o
obj-y+=libnand

install: build
	cp nand.ko $(LICHEE_MOD_DIR)/

build:
	@echo $(LICHEE_KDIR)
	cp libnand libnand.a
	$(MAKE) -C $(LICHEE_KDIR) M=$(PWD)


clean:
	@rm -rf *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~
