BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -O3 -fPIC -w

SRCS = $(shell find . -maxdepth 1 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/_make/generic.mk
