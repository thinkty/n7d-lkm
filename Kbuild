MODULE := n7d
OBJECTS := n7d_buffer.o n7d_ops.o n7d_dev.o n7d_main.o

obj-m += $(MODULE).o
$(MODULE)-y := $(OBJECTS)

ccflags-y += -Wall -Werror -Iinclude