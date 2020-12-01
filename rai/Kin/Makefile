BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

DEPEND = Core Geo GeoOptim Algo Gui 

GL = 1
PHYSX = 1
BULLET = 1
ODE = 0
SWIFT = 1
ASSIMP = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
