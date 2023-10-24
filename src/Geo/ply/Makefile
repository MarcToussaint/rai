BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -O3 -fPIC -w

SRCS = ply.c
OBJS = ply.o 

include $(BASE)/_make/generic.mk
