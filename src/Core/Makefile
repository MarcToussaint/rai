BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

OPTIM = fast_debug

DEPEND_UBUNTU += g++ make gnupg gdb cmake gnuplot

X11 = 1
LAPACK = 1
PTHREAD = 1
ROS = 1
JSON = 1
YAML = 1
H5 = 1
EIGEN = 1
CXXFLAGS += -DRAI_ROOT_PATH=\"`pwd`/../..\"

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
