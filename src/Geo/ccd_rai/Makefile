BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -O3 -fPIC -std=c99 -w -I..

SRCS = $(shell find . -maxdepth 1 -name '*.c' )
OBJS = $(SRCS:%.c=%.o)

include $(BASE)/_make/generic.mk
