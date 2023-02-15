BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Algo

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/makeutils/generic.mk
