BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libextern_$(NAME).a

QHULL = 1

CXXFLAGS += -DSWIFT_DECOMP -O3 -fPIC -w -IRAPID

SRCS = $(shell find . -maxdepth 2 -name '*.cpp' -not -name 'togl.cpp')
OBJS = $(SRCS:%.cpp=%.o)

# target: $(OUTPUT) ./decomposer.exe
# ./decomposer.exe: main.o
# ./decomposer.exe: OBJS+=main.o
# LIBS += -lextern_SWIFT -lqhull -lm 

DEPEND = Geo

include $(BASE)/_make/generic.mk
