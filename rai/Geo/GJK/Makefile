BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

#CFLAGS += -fPIC -w -g
CFLAGS += -O3 -fPIC -w -g

SRCS = gjk.c
OBJS = gjk.o

include $(BASE)/build/generic.mk
