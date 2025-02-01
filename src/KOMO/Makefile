BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

#OPTIM = debug

#CXXFLAGS += -fno-stack-protector
DEPEND = Core Geo Kin Optim PathAlgos Logic

GL = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
