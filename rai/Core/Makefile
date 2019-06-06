BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

DEPEND_UBUNTU += g++ gnuplot

X11 = 1
LAPACK = 1
PTHREAD = 1
ROS = 1
JSON = 1
EIGEN = 1
CXXFLAGS += -DRAI_ROOT_PATH=\"`pwd`/../..\"

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
