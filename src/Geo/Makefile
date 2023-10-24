BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

DEPEND = Core Optim Algo

EIGEN = 1
QHULL = 1
GL = 1
ASSIMP = 1
FCL = 1
PLY = 1
GJK = 1
CCD = 1
Lewiner = 1
CXXFLAGS += -DRAI_VHACD

FCL_LOCAL = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
