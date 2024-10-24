# CXX=clang++
# this is a generic make file
# supposed to be called from lib, test and projects makefiles
#
# user configuration options are set in config.mk -- not here!

.SECONDARY:

.PRECIOUS: %.o

BASE_ORIGINAL := $(BASE)
BASE := $(shell readlink -f $(BASE))


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

## if we weren't called from make-path.sh add cleanLocks
#ifndef SUB_MAKE
#PREOBJS := cleanLocks $(PREOBJS)
#endif


################################################################################
#
# default target
#
################################################################################

default: $(OUTPUT)
all: $(OUTPUT) #this is for qtcreator, which by default uses the 'all' target


################################################################################
#
# load user options from the local make-config
# load defines for linking to external libs
#
################################################################################

ifneq ("$(wildcard localConfig.mk)","")

$(BASE)/_make/config.mk:: localConfig.mk
	cp $< $@

else ifneq ("$(wildcard $(BASE)/../config.mk)","")

$(BASE)/_make/config.mk:: $(BASE)/../config.mk
	cp $< $@

else

$(BASE)/_make/config.mk:: $(BASE)/_make/config.mk.default
	cp $< $@

endif

include $(BASE)/_make/config.mk
include $(BASE)/_make/defines.mk

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
CPATHS	+= $(BASE)/src $(HOME)/.local/include
ifdef CONDA_PREFIX
CPATHS  += $(CONDA_PREFIX)/include
endif
ifdef BASE2
CPATHS	+= $(BASE2)/src
endif
LPATHS	+= $(BASE)/lib $(HOME)/.local/lib /usr/local/lib
ifdef BASE2
LPATHS	+= $(BASE2)/lib
endif
#CPATHS  += /usr/include
#LPATHS  += /usr/lib/x86_64-linux-gnu
LIBS += -lrt

#google-pprof:
##LIBS += -Wl,--no-as-needed -lprofiler -Wl,--as-needed -ltcmalloc
#CPUPROFILE=/tmp/prof.out ./x.exe
#google-pprof ./x /tmp/prof.out

SHAREFLAG = -shared #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined

CXXFLAGS += -Wno-terminate -Wno-array-bounds -Wno-pragmas -fPIC
CFLAGS += -fPIC -O3

ifndef RAI_NO_CXX11
CXXFLAGS += -std=c++2a
endif

ifndef OPTIM
OPTIM = fast_debug
endif

ifeq ($(OPTIM),debug)
CXXFLAGS := -g -Wall $(CXXFLAGS)#-Wno-int-to-pointer-cast#-Wno-invalid-offsetof
endif
ifeq ($(OPTIM),fast_debug)
CXXFLAGS := -g -O3 -Wall $(CXXFLAGS)
endif
ifeq ($(OPTIM),penibel)
CXXFLAGS := -g -Wall -Wextra $(CXXFLAGS)
endif
ifeq ($(OPTIM),fast)
CXXFLAGS := -O3 -Wall $(CXXFLAGS)
endif
ifeq ($(OPTIM),prof)
CXXFLAGS := -O3 -pg -Wall -DRAI_NOCHECK -fno-inline $(CXXFLAGS)
LDFLAGS += -pg
endif
ifeq ($(OPTIM),callgrind)
CXXFLAGS := -O -g -Wall -DRAI_NOCHECK -fno-inline $(CXXFLAGS)
endif


################################################################################
#
# VARS for SWIG wrappers
#
################################################################################

MODULE_NAME=$(shell echo $(notdir $(CURDIR)) | tr A-Z a-z)

SWC_FLAGS=-std=c++0x -g
SWC_INCLUDES=-I$(BASE)/src -I/usr/include/python2.7 
SWC_LIB_PATH=-L$(BASE)/lib -Xlinker -rpath $(BASE)/lib
SWC_CXXFLAGS=-c $(SWC_FLAGS) -fPIC $(SWC_INCLUDES) -DSWIG_TYPE_TABLE=mlr
SWC_LDFLAGS=$(SWC_FLAGS) -shared $(SWC_LIB_PATH) $(SWC_INCLUDES) $(DEPEND:%=-l%) -l$(notdir $(CURDIR))

ifndef SWIG
SWIG=swig2.0
endif
SWIG_INCLUDE=-I$(BASE)/src -I$(BASE)/include/numpy
SWIG_FLAGS=-c++ -python $(SWIG_INCLUDE)


################################################################################
#
# depending on a local components
#
################################################################################

BUILDS := $(DEPEND:%=inPath_makeLib/%) $(BUILDS)
LIBS := $(DEPEND:%=-l%) $(LIBS)
#CXXFLAGS := $(DEPEND:%=-DRAI_%) $(CXXFLAGS)


################################################################################
#
# export include/lib paths
#
################################################################################

CPATH := $(CPATH):$(CPATHS:%=:%:)
LPATH := $(LPATH):$(LPATHS:%=:%:)
LDFLAGS += $(LPATHS:%=-L%) #$(LPATHS:%=-Wl,-rpath,%)
LD_RUN_PATH := $(LD_RUN_PATH):$(LPATH)
LD_LIBRARY_PATH := $(LD_LIBRARY_PATH):$(LPATH)
export CPATH
export LPATH
export LD_RUN_PATH
export LD_LIBRARY_PATH


################################################################################
#
# concrete make targets
#
################################################################################

default: $(OUTPUT)
all: $(OUTPUT) #this is for qtcreator, which by default uses the 'all' target

clean: cleanLocks cleanLibs force
	@echo "   *** clean      " $(PWD)
	rm -f $(OUTPUT) $(OBJS) $(PREOBJS) callgrind.out.* $(CLEAN)
	@rm -f $(MODULE_NAME)_wrap.* $(MODULE_NAME)py.so $(MODULE_NAME)py.py
	if [ -f "main.ipynb" ]; then jupyter-nbconvert --clear-output --inplace main.ipynb; fi

cleanLocks: force
	@echo "   *** cleanLocks " $(PWD)
	@find $(PWD) $(BASE) $(BASE2) -type d -name 'Make.lock' -delete -print || exit 0

cleanLibs: force
	@echo "   *** cleanLibs  " $(PWD)
	@find $(BASE) $(BASE2) \( -type f -or -type l \) \( -name 'lib*.so' -or -name 'lib*.a' \) -not -path "$(BASE2)/build/*" -delete -print

cleanAll: cleanLocks cleanDepends force
	@echo "   *** cleanAll   " $(PWD)
	@find $(PWD) $(BASE) $(BASE2) \( -type f -or -type l \) \( -name '*.o' -or -name 'lib*.so' -or -name 'lib*.a' -or -name 'x.exe' -or -name 'unity.cxx' \) -delete -print

cleanDepends: force
	@find $(BASE) $(BASE2) -type f -name 'Makefile.dep' -delete -print

clean/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --clear-output --inplace $<

installUbuntu: force
	sudo apt-get -q $(APTGETYES) install $(DEPEND_UBUNTU)

printUbuntu: force
	@echo $(DEPEND_UBUNTU)

printDepend: force
	@echo $(DEPEND)

depend: generate_Makefile.dep

# dependAll: force
# 	@echo "   *** dependAll   " $(PWD)
# 	@find $(PWD) $(BASE) $(BASE2) -type f -name 'Makefile' -execdir $(MAKE) depend \;


info: force
	@echo; echo ----------------------------------------
	@echo "     " "environment configuration (see make-generic file)";
	@echo ----------------------------------------; echo
	@echo "  PWD =" "$(PWD)"
	@echo "  BASE_ORIGINAL =" "$(BASE_ORIGINAL)"
	@echo "  BASE =" "$(BASE)"
	@echo "  BASE2 =" "$(BASE2)"
	@echo "  NAME =" "$(NAME)"
	@echo "  LIBPATH =" "$(LIBPATH)"
	@echo "  EXTERNALS =" "$(EXTERNALS)"
	@echo "  DEPEND_UBUNTU =" "$(DEPEND_UBUNTU)"
	@echo "  CXX =" "$(CXX)"
	@echo "  OPTIM =" "$(OPTIM)"
	@echo "  CXXFLAGS =" "$(CXXFLAGS)"
	@echo "  LINK =" "$(LINK)"
	@echo "  LDFLAGS =" "$(LDFLAGS)"
	@echo "  CPATHS =" "$(CPATHS)"
	@echo "  CPATH =" "$(CPATH)"
	@echo "  LPATHS =" "$(LPATHS)"
	@echo "  LPATH =" "$(LPATH)"
	@echo "  LD_RUN_PATH =" "$(LD_RUN_PATH)"
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
	@mkdir -p $(BASE)/lib
	cp $@ $(BASE)/lib

#%.so: $(PREOBJS) $(BUILDS) z.unity.o
#	$(LINK) $(LDFLAGS) -o $@ z.unity.o $(LIBS) $(SHAREFLAG)
#	cp $@ $(BASE)/lib

%.lib: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) -static ### $(SHAREFLAG)

%.a: $(PREOBJS) $(BUILDS) $(OBJS)
	ar -crvs $@ $(OBJS)
	@mkdir -p $(BASE)/lib
	cp $@ $(BASE)/lib

#%.a: $(PREOBJS) $(BUILDS)
#	$(CXX) $(CXXFLAGS) -c $(SRCS)
#	ar -crvs $@ $*.o
#	cp $@ $(BASE)/lib

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
	-$(CXX) -MM $(SRCS) $(CFLAGS) $(CXXFLAGS) > Makefile.dep

unity.cxx: $(SRCS) force
	@find . -maxdepth 1 -name '*.h' -printf '#include "%f"\n' | sort > unity.cxx
	@find . -maxdepth 1 -name '*.cpp' -printf '#include "%f"\n' | sort >> unity.cxx
#	@find . -maxdepth 1 -name '*.cpp' -fprintf unity.cxx '#include "%f"\n'
#	@echo "$(SRCS:%=#include\"%\"\n)" > unity.cxx



################################################################################
#
# meta rules -- for calling make in other directories
#
################################################################################

inPath_makeLib/extern_%: % $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< libextern_$*.a

inPath_makeLib/Hardware_%: $(BASE2)/Hardware/% $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< libHardware_$*.so

inPath_makeLib/%: $(BASE)/src/% $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< lib$*.so

inPath_makeLib/%: $(BASE)/src/contrib/% $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< lib$*.so

ifdef BASE2
inPath_makeLib/%: $(BASE2)/src/% $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< lib$*.so
endif

inPath_make/%: % $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< x.exe

inPath_makeTest/%: % $(PREOBJS)
	+@-$(BASE)/_make/make-path.sh $< x.exe RAI_TESTS=1

inPath_run/%: % $(PREOBJS)
	+@-$(BASE)/_make/run-path.sh $< x.exe

inPath_clean/%: %
	@echo "                                                ***** clean " $*
	@-rm -f $*/Makefile.dep
	@-$(MAKE) -C $* -f Makefile clean --no-print-directory

inPath_clean/%: $(BASE)/src/%
	@echo "                                                ***** clean " $<
	@-rm -f $</Makefile.dep
	@-$(MAKE) -C $< -f Makefile clean --no-print-directory

ifdef BASE2
inPath_clean/%: $(BASE2)/src/%
	@echo "                                                ***** clean " $<
	@-rm -f $</Makefile.dep
	@-$(MAKE) -C $< -f Makefile clean --no-print-directory
endif

inPath_unity/%: $(BASE)/src/%
	@echo "                                                ***** unity.cxx " $<
	@-$(MAKE) -C $< -f Makefile unity.cxx --no-print-directory

inPath_unity/%: $(BASE2)/src/%
	@echo "                                                ***** unity.cxx " $<
	@-$(MAKE) -C $< -f Makefile unity.cxx --no-print-directory

inPath_depend/%: %
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory

inPath_depend/%: $(BASE)/src/%
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory

ifdef BASE2
inPath_depend/%: $(BASE2)/src/%
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory
endif

inPath_installUbuntu/%: $(BASE)/src/%
	@echo "                                                ***** init " $*
	@-$(MAKE) -C $< installUbuntu --no-print-directory

ifdef BASE2
inPath_installUbuntu/%: $(BASE2)/src/%
	@echo "                                                ***** init " $*
	@-$(MAKE) -C $< installUbuntu --no-print-directory
endif

inPath_printUbuntu/%: $(BASE)/src/%
	@echo "#" $*
	@-$(MAKE) -C $< printUbuntu --no-print-directory

inPath_printDepend/%: $(BASE)/src/%
	@echo "#" $*
	@-$(MAKE) -C $< printDepend --no-print-directory

inPath_makePython/%: %
	make --directory=$< pywrapper

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude


force:	;
