#
# arch/blackfin/mach-bf561/Makefile
#

obj-y := ints-priority.o dma.o

obj-$(CONFIG_BF561_COREB) += coreb.o
obj-$(CONFIG_ICC:m=y)	+= icc.o
obj-$(CONFIG_SMP)  += smp.o secondary.o atomic.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
