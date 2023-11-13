BASE   = ../..
NAME   = $(shell basename `pwd`)
EXT    = $(shell python3-config --extension-suffix)
OUTPUT = _robotic.so #$(EXT)

DEPEND = Core Algo Optim Gui Geo Kin KOMO PathAlgos

PYBIND = 1
PHYSX = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/_make/generic.mk

docs:
	env PYTHONPATH="." pybind11-stubgen --ignore-invalid=all _robotic
	mv stubs/_robotic-stubs/__init__.pyi _robotic.pyi
	rm -Rf stubs
