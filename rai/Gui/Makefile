BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so
-include $(BASE)/build/config.mk

GL = 1
GTK = 1
#GTKGL = 1
FREEGLUT = 1
GRAPHVIZ = 1
GL2PS = 1

DEPEND = Core Geo

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
