PWD=$(shell pwd)
MAKEOPTS = LICHEE_MOD_DIR=$(PWD) LICHEE_KDIR=$(PWD)/..

modules = nand mali example 

all:
	for i in $(modules) ; do make $(MAKEOPTS) -C $$i ; done

