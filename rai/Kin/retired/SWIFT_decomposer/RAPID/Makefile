OUTPUT    = ../../../../../lib/libRAPID.a
CXXFLAGS += -O3 -fPIC -Wno-deprecated
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)

$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)
