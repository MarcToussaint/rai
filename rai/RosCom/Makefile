BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND_UBUNTU = ros-kinetic-desktop ros-kinetic-object-recognition-msgs ros-kinetic-ar-track-alvar-msgs

ROS = 1
PCL = 1
DEPEND = Core KOMO Kin Gui Logic

CXXFLAGS += -DRAI_ROS_BAXTER
CPATHS += $(BASE)/../external/devel/include

#CXXFLAGS += -DROS_PACKAGE_NAME='"pr2_module"'

#CPATHS += $(BASE)/projects/marc_controller_pkg/msg_gen/cpp/include

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
