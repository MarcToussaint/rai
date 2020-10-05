BASE   = ../..
NAME   = $(shell basename `pwd`)
EXT    = $(shell python3-config --extension-suffix)
OUTPUT = lib$(NAME)$(EXT)

DEPEND = Core Kin KOMO LGP Operate RosCom Perception

PYBIND = 1
PHYSX = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
