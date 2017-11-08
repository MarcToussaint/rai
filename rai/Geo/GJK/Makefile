BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -fPIC -w -g
#CFLAGS += 0O3 -fPIC -w

SRCS = gjk.c
OBJS = gjk.o

include $(BASE)/build/generic.mk
