BASE   = ../..
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

#OPTIM=fast

DEPEND = Core Geo Gui Kin Algo Control

GL = 1
LAPACK = 1
OPENCV = 1
PCL = 1
X264 = 0

#CXXFLAGS += -D__STDC_CONSTANT_MACROS -DHAVE_LIBAV
#LIBS += -lavformat -lavcodec -lavutil -lswscale -lx264 -lz
#CXXFLAGS += -Wno-unknown-pragmas

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
