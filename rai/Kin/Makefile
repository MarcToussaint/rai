BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

GL = 1
PHYSX = 1
BULLET = 1
ODE = 0
SWIFT = 1
ASSIMP = 1

DEPEND = Core Geo GeoOptim Algo Gui 

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
