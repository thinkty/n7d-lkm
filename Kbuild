MODULE := n7d
OBJECTS := main.o n7d_ops.o

obj-m += $(MODULE).o
$(MODULE)-y := $(OBJECTS)

ccflags-y += -Wall -Werror -Iinclude