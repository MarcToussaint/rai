BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug
QHULL = 1
GL = 1
ASSIMP = 1
FCL = 1

CXXFLAGS += -DRAI_extern_GJK -DRAI_extern_Lewiner -DRAI_extern_ply -DRAI_extern_ccd

DEPEND = Core extern_GJK extern_Lewiner extern_ply extern_ccd

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
