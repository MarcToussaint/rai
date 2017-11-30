BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so
-include $(BASE)/build/config.mk

DEPEND_UBUNTU = freeglut3-dev graphviz-dev #libx11-dev libxi-dev libxmu-dev

GL = 1
GTK = 0
#GTKGL = 1
FREEGLUT = 1
GRAPHVIZ = 1
GL2PS = 0

DEPEND = Core Geo

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
