BASE   = ../..
NAME   = $(shell basename `pwd`)
EXT    = $(shell python3-config --extension-suffix)
OUTPUT = lib$(NAME)$(EXT)

DEPEND = Core Kin KOMO LGP Control Perception Operate

PYBIND = 1
PHYSX = 1

CXXFLAGS += -DRAI_BIND_KOMO

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
