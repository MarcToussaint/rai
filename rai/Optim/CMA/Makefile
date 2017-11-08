BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -O3 -fPIC -w

SRCS = cmaes.c
OBJS = cmaes.o 

include $(BASE)/build/generic.mk
