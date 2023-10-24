BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

QHULL = 1

CXXFLAGS += -O3 -fPIC -DNO_CYCLE_DETECTION -DSWIFT_DECOMP -w

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk
