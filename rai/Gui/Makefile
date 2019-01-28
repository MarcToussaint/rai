BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so
-include $(BASE)/build/config.mk

X11 = 1
PNG = 1
GL = 1
GTK = 0
#GTKGL = 1
FREEGLUT = 0
GRAPHVIZ = 1
GL2PS = 0
GLFW = 1

DEPEND = Core Geo

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

ifeq ($(GLFW),1)
DEPEND_UBUNTU += libglfw3-dev
CXXFLAGS  += -DRAI_GLFW
CPATH := $(HOME)/opt/include:$(CPATH)
LPATH := $(HOME)/opt/lib:$(LPATH)
LIBS      += -lglfw
endif


include $(BASE)/build/generic.mk
