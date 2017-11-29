BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

DEPEND_UBUNTU = g++ libx11-dev liblapack-dev libf2c2-dev gnuplot libjsoncpp-dev

LAPACK = 1
PTHREAD = 1
ROS = 1
JSON = 1
CXXFLAGS += -DMLR_CORE_PATH=\"`pwd`\"

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
