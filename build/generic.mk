# CXX=clang++
# this is a generic make file
# supposed to be called from lib, test and projects makefiles
#
# user configuration options are set in config.mk -- not here!

.SECONDARY:

.PRECIOUS: %.o

BASE_REAL = $(shell realpath $(BASE))

################################################################################
#
# load user options from the local make-config
#
################################################################################
include $(BASE)/build/config.mk


################################################################################
#
# standard objects to be compiled, output file
#
################################################################################
ifndef OBJS
OBJS = main.o
endif
ifndef OUTPUT
OUTPUT = x.exe
endif
ifndef SRCS
SRCS = $(OBJS:%.o=%.cpp)
endif


################################################################################
#
# basic compiler settings
#
################################################################################
# (a tag like `OPTIM=fast' in the local Makefiles changes default debug mode)

ifndef CXX
CXX	= g++
CC	= gcc
endif
MOC = moc
UIC = uic
YACC = bison -d

LINK	= $(CXX)
CPATHS	+= $(BASE)/src $(BASE)/include
LPATHS	+= $(BASE)/src /usr/local/lib
LIBS += -lrt
SHAREFLAG = -shared #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined

CXXFLAGS += -fPIC
CFLAGS += -fPIC

ifndef MLR_NO_CXX11
CXXFLAGS += -std=c++0x
endif

ifndef OPTIM
OPTIM = debug
endif

ifeq ($(OPTIM),debug)
CXXFLAGS := -g -Wall $(CXXFLAGS)#-Wno-int-to-pointer-cast#-Wno-invalid-offsetof
endif
ifeq ($(OPTIM),fast_debug)
CXXFLAGS := -g -O -Wall $(CXXFLAGS)
endif
ifeq ($(OPTIM),penibel)
CXXFLAGS := -g -Wall -Wextra $(CXXFLAGS)
endif
ifeq ($(OPTIM),fast)
CXXFLAGS := -O3 -Wall -DMLR_NOCHECK $(CXXFLAGS)
endif
ifeq ($(OPTIM),prof)
CXXFLAGS := -O3 -pg -Wall -DMLR_NOCHECK -fno-inline $(CXXFLAGS)
LDFLAGS += -pg
endif
ifeq ($(OPTIM),callgrind)
CXXFLAGS := -O3 -g -Wall -DMLR_NOCHECK $(CXXFLAGS) #-fno-inline
endif


################################################################################
#
# load defines for linking to external libs
#
################################################################################
include $(BASE)/build/defines.mk


################################################################################
#
# VARS for SWIG wrappers
#
################################################################################

ifndef MLR_PATH
MLR_PATH=$(HOME)/git/mlr
endif

MODULE_NAME=$(shell echo $(notdir $(CURDIR)) | tr A-Z a-z)

SWC_FLAGS=-std=c++0x -g
SWC_INCLUDES=-I$(MLR_PATH)/share/src -I/usr/include/python2.7 
SWC_LIB_PATH=-L$(MLR_PATH)/share/lib -Xlinker -rpath $(MLR_PATH)/share/lib
SWC_CXXFLAGS=-c $(SWC_FLAGS) -fPIC $(SWC_INCLUDES) -DSWIG_TYPE_TABLE=mlr
SWC_LDFLAGS=$(SWC_FLAGS) -shared $(SWC_LIB_PATH) $(SWC_INCLUDES) $(DEPEND:%=-l%) -l$(notdir $(CURDIR))

ifndef SWIG
SWIG=swig2.0
endif
SWIG_INCLUDE=-I$(MLR_PATH)/share/src -I$(MLR_PATH)/share/include/numpy
SWIG_FLAGS=-c++ -python $(SWIG_INCLUDE)


################################################################################
#
# depending on a local components
#
################################################################################

BUILDS := $(DEPEND:%=makeDepend/%) $(BUILDS)
LIBS := $(DEPEND:%=-l%) $(LIBS)
CXXFLAGS := $(DEPEND:%=-DMLR_%) $(CXXFLAGS)


################################################################################
#
# export Linux/MSVC include/lib paths
#
################################################################################

CPATH := $(CPATH):$(CPATHS:%=:%:)
LPATH := $(LPATH):$(LPATHS:%=:%:)
LDFLAGS += $(LPATHS:%=-L%)
LD_RUN_PATH := $(LD_RUN_PATH):$(LPATH)
LD_LIBRARY_PATH := $(LD_LIBRARY_PATH):$(LPATH)
export CPATH
export LPATH
export LD_RUN_PATH
export LD_LIBRARY_PATH
export MSVC_CPATH
export MSVC_LPATH


################################################################################
#
# concrete make targets
#
################################################################################

default: $(OUTPUT)
all: $(OUTPUT) #this is for qtcreator, which by default uses the 'all' target

clean: force
	rm -f $(OUTPUT) $(OBJS) $(PREOBJS) callgrind.out.* $(CLEAN)
	@rm -f $(MODULE_NAME)_wrap.* $(MODULE_NAME)py.so $(MODULE_NAME)py.py
	@find $(BASE) -type d -name 'Make.lock' -delete -print
	@find $(BASE)/src \( -type f -or -type l \) \( -name 'lib*.so' -or -name 'lib*.a' \)  -delete -print

cleanLocks: force
	@find $(BASE) -type d -name 'Make.lock' -delete -print

cleanAll: force
	@find $(BASE) -type d -name 'Make.lock' -delete -print
	@find $(BASE) \( -type f -or -type l \) \( -name '*.o' -or -name 'lib*.so' -or -name 'lib*.a' -or -name 'x.exe' \) -delete -print

cleanLibs: force
	@find $(BASE)/src -type f \( -name 'lib*.so' -or -name 'lib*.a' \)  -delete -print

cleanDepends: force
	@find $(BASE) -type f -name 'Makefile.dep' -delete -print

depend: generate_Makefile.dep

info: force
	@echo; echo ----------------------------------------
	@echo "     " "environment configuration (see make-generic file)";
	@echo ----------------------------------------; echo
	@echo "  PWD =" "$(PWD)"
	@echo "  BASE =" "$(BASE)"
	@echo "  BASE_REAL =" "$(BASE_REAL)"
	@echo "  NAME =" "$(NAME)"
	@echo "  LIBPATH =" "$(LIBPATH)"
	@echo "  EXTERNALS =" "$(EXTERNALS)"
	@echo "  CXX =" "$(CXX)"
	@echo "  OPTIM =" "$(OPTIM)"
	@echo "  CXXFLAGS =" "$(CXXFLAGS)"
	@echo "  LINK =" "$(LINK)"
	@echo "  LDFLAGS =" "$(LDFLAGS)"
	@echo "  CPATH =" "$(CPATH)"
	@echo "  LPATHS =" "$(LPATHS)"
	@echo "  LPATH =" "$(LPATH)"
	@echo "  LD_RUN_PATH =" "$(LD_RUN_PATH)"
	@echo "  MSVC_CPATH =" "$(MSVC_CPATH)"
	@echo "  MSVC_LPATH =" "$(MSVC_LPATH)"
	@echo "  SRCS =" "$(SRCS)"
	@echo "  OBJS =" "$(OBJS)"
	@echo "  LIBS =" "$(LIBS)"
	@echo "  PREOBJS =" "$(PREOBJS)"
	@echo "  OUTPUT =" "$(OUTPUT)"
	@echo "  DEPEND =" "$(DEPEND)"
	@echo "  BUILDS =" "$(BUILDS)"
	@echo "  MAKEMODE =" "$(MAKEMODE)"
	@echo


################################################################################
#
# optionally include dependencies
#
################################################################################
-include Makefile.dep


################################################################################
#
# SWIG rules
#
################################################################################

# keep the actual wrapper
.SECONDARY: $(MODULE_NAME)_wrap.cxx  

%_wrap.cxx: %.i
	$(SWIG) $(SWIG_FLAGS) $<

%_wrap.o: %_wrap.cxx
	$(CXX) $(SWC_CXXFLAGS) $<

_%.so: %_wrap.o
	$(CXX) $< $(SWC_LDFLAGS) -o $@

%py.py: 

pywrapper: $(OUTPUT) $(MODULE_NAME)py.so $(MODULE_NAME)py.py
	install -D $(MODULE_NAME)py.py ~/.local/lib/python2.7/site-packages/$(MODULE_NAME)py.py
	install -D $(MODULE_NAME)py.so ~/.local/lib/python2.7/site-packages/_$(MODULE_NAME)py.so


################################################################################
#
# rules
#
################################################################################

%.exe: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.so: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) $(SHAREFLAG)

%.lib: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) -static ### $(SHAREFLAG)

%.a: $(PREOBJS) $(BUILDS) $(OBJS)
	ar -crvs $@ $(OBJS)

%.mexglx: $(PREOBJS) $(OBJS)
	mex -cxx $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

ifeq ($(CUDA),1)
%_cuda.o: %_cuda.cpp
	if test ! -L $*_cuda.cu; then ln -s -f $*_cuda.cpp $*_cuda.cu; fi;
	$(NXX) $(NXXFLAGS) -o $@ -c $*_cuda.cu
else
%_cuda.o: %_cuda.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<
endif

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.obj: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%.obj: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

%_ui.h: %.ui
	$(UIC) -o $*_ui.h $<

%_moc.cpp: %.h
	$(MOC) -o $*_moc.cpp $*.h

%_moc.cxx: %.h
	$(MOC) -o $*_moc.cxx $*.h

%.tab.cpp: %.ypp
	$(YACC) $<

%.tab.h: %.ypp
	$(YACC) --defines=$@ $<

## generate a make dependency file
generate_Makefile.dep: $(SRCS)
	-$(CXX) -MM $(SRCS) $(CXXFLAGS) > Makefile.dep

includeAll.cxx: force
	find . -maxdepth 1 -name '*.cpp' -exec echo "#include \"{}\"" \; > includeAll.cxx


################################################################################
#
# meta rules -- for calling make in other directories
#
################################################################################

makeDepend/extern_%: %
	+@-$(BASE)/build/make-path.sh $< libextern_$*.a
	@cd $(BASE)/src && ln -sf $(NAME)/$*/libextern_$*.a libextern_$*.a

makeDepend/Hardware_%: $(BASE)/src/Hardware/%
	+@-$(BASE)/build/make-path.sh $< libHardware_$*.so
	@cd $(BASE)/src && ln -sf Hardware/$*/libHardware_$*.so libHardware_$*.so

makeDepend/%: $(BASE)/src/%
	+@-$(BASE)/build/make-path.sh $< lib$*.so
	@cd $(BASE)/src && ln -sf $*/lib$*.so lib$*.so

makePath/%: %
	+@-$(BASE)/build/make-path.sh $< x.exe

makeTest/%: %
	+@-$(BASE)/build/make-path.sh $< x.exe MLR_TESTS=1

runPath/%: %
	+@-$(BASE)/build/run-path.sh $< x.exe

cleanPath/%: %
	@echo "                                                ***** clean " $*
	@-rm -f $*/Makefile.dep
	@-$(MAKE) -C $* -f Makefile clean --no-print-directory

cleanPath/%: $(BASE)/src/%
	@echo "                                                ***** clean " $<
	@-rm -f $</Makefile.dep
	@-$(MAKE) -C $< -f Makefile clean --no-print-directory

makePythonPath/%: %
	make --directory=$< pywrapper

#$(BASE)/build/config.mk: $(BASE)/build/config.mk.default
#	cp $< $@

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude


force:	;
