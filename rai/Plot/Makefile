BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so
-include $(BASE)/build/config.mk

DEPEND = Core Gui

GL = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
