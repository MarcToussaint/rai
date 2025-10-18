BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

CFLAGS += -O3 -fPIC -w

SRCS = cmaes.c boundary_transformation.c
OBJS = cmaes.o boundary_transformation.o

include $(BASE)/_make/generic.mk
