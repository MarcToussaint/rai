BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

ROS = 1
PCL = 1
DEPEND = Core KOMO Kin Gui Logic

#CXXFLAGS += -DROS_PACKAGE_NAME='"pr2_module"'

#CPATHS += $(BASE)/projects/marc_controller_pkg/msg_gen/cpp/include

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
