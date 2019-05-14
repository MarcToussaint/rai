BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

#OPTIM = fast_debug

BULLET = 1
GL = 1
PHYSX = 1
ODE = 0
DEPEND = Core Geo GeoOptim Algo Gui extern_SWIFT extern_SWIFT_decomposer
#CXXFLAGS += -Wno-unknown-pragmas -O0

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
