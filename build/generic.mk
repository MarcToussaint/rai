# CXX=clang++
# this is a generic make file
# supposed to be called from lib, test and projects makefiles
#
# user configuration options are set in config.mk -- not here!

.SECONDARY:

.PRECIOUS: %.o

BASE_ORIGINAL := $(BASE)
BASE := $(shell realpath $(BASE))


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

$(BASE)/config.mk:: localConfig.mk
	cp $< $@

else ifneq ("$(wildcard $(BASE)/../config.mk)","")

$(BASE)/config.mk:: $(BASE)/../config.mk
	cp $< $@

else

$(BASE)/config.mk:: $(BASE)/build/config.mk.default
	cp $< $@

endif

include $(BASE)/config.mk

include $(BASE)/build/defines.mk


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
CPATHS	+= $(BASE)/rai $(HOME)/opt/include
ifdef BASE2
CPATHS	+= $(BASE2)
endif
LPATHS	+= $(BASE)/lib $(HOME)/opt/lib /usr/local/lib
LIBS += -lrt
SHAREFLAG = -shared #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined

CXXFLAGS += -Wno-terminate -Wno-pragmas -fPIC
CFLAGS += -fPIC

ifndef RAI_NO_CXX11
#CXXFLAGS += -std=c++0x
CXXFLAGS += -std=c++14
endif

ifndef OPTIM
OPTIM = debug
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
	@find $(BASE)/rai $(BASE)/lib $(BASE2) \( -type f -or -type l \) \( -name 'lib*.so' -or -name 'lib*.a' \)  -delete -print

cleanAll: cleanLocks cleanDepends force
	@echo "   *** cleanAll   " $(PWD)
	@find $(PWD) $(BASE) $(BASE2) \( -type f -or -type l \) \( -name '*.o' -or -name 'lib*.so' -or -name 'lib*.a' -or -name 'x.exe' \) -delete -print

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
	cp $@ $(BASE)/lib

#%.so: $(PREOBJS) $(BUILDS) z.SRCS.o
#	$(LINK) $(LDFLAGS) -o $@ z.SRCS.o $(LIBS) $(SHAREFLAG)
#	cp $@ $(BASE)/lib

%.lib: $(PREOBJS) $(BUILDS) $(OBJS)
	$(LINK) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) -static ### $(SHAREFLAG)

%.a: $(PREOBJS) $(BUILDS) $(OBJS)
	ar -crvs $@ $(OBJS)
	cp $@ $(BASE)/lib

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

z.SRCS.cxx: $(SRCS)
	@echo "$(SRCS:%=#include\"%\"\n)" > z.SRCS.cxx
#	find . -maxdepth 1 -name '*.cpp' -exec echo "#include \"{}\"" \; > libInc.cxx


################################################################################
#
# meta rules -- for calling make in other directories
#
################################################################################

inPath_makeLib/extern_%: % $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< libextern_$*.a

inPath_makeLib/Hardware_%: $(BASE2)/Hardware/% $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< libHardware_$*.so

inPath_makeLib/%: $(BASE)/rai/% $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< lib$*.so

inPath_makeLib/%: $(BASE)/rai/contrib/% $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< lib$*.so

ifdef BASE2
inPath_makeLib/%: $(BASE2)/% $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< lib$*.so
endif

inPath_make/%: % $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< x.exe

inPath_makeTest/%: % $(PREOBJS)
	+@-$(BASE)/build/make-path.sh $< x.exe RAI_TESTS=1

inPath_run/%: % $(PREOBJS)
	+@-$(BASE)/build/run-path.sh $< x.exe

inPath_clean/%: %
	@echo "                                                ***** clean " $*
	@-rm -f $*/Makefile.dep
	@-$(MAKE) -C $* -f Makefile clean --no-print-directory

inPath_clean/%: $(BASE)/rai/%
	@echo "                                                ***** clean " $<
	@-rm -f $</Makefile.dep
	@-$(MAKE) -C $< -f Makefile clean --no-print-directory

ifdef BASE2
inPath_clean/%: $(BASE2)/%
	@echo "                                                ***** clean " $<
	@-rm -f $</Makefile.dep
	@-$(MAKE) -C $< -f Makefile clean --no-print-directory
endif

inPath_depend/%: %
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory

inPath_depend/%: $(BASE)/rai/%
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory

ifdef BASE2
inPath_depend/%: $(BASE2)/%
	@echo "                                                ***** depend " $<
	@-$(MAKE) -C $< -f Makefile depend --no-print-directory
endif

inPath_installUbuntu/%: $(BASE)/rai/%
	@echo "                                                ***** init " $*
	@-$(MAKE) -C $< installUbuntu --no-print-directory

ifdef BASE2
inPath_installUbuntu/%: $(BASE2)/%
	@echo "                                                ***** init " $*
	@-$(MAKE) -C $< installUbuntu --no-print-directory
endif

inPath_printUbuntu/%: $(BASE)/rai/%
	@echo "#" $*
	@-$(MAKE) -C $< printUbuntu --no-print-directory

inPath_printDepend/%: $(BASE)/rai/%
	@echo "#" $*
	@-$(MAKE) -C $< printDepend --no-print-directory

inPath_makePython/%: %
	make --directory=$< pywrapper

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude --exclude-from $(NAME)/tar.exclude


force:	;
