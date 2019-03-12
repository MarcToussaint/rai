BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

#OPTIM = fast_debug

QHULL = 1
GL = 1
ASSIMP = 1
FCL = 1

DEPEND = Core extern_GJK extern_Lewiner extern_ply extern_ccd

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
