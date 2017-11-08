BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so
-include $(BASE)/build/config.mk

GL = 1

DEPEND = Core Gui

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
