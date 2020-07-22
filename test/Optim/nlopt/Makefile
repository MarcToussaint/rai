BASE = ../../..

DEPEND = Core Optim

NLOPT = 1

ifeq ($(NLOPT),1)
CXXFLAGS += -DRAI_NLOPT `pkg-config --cflags nlopt`
LIBS     += `pkg-config --libs nlopt`
endif

include $(BASE)/build/generic.mk
