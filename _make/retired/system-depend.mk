ifneq ($(SYS),Linux)
CXXFLAGS+= -DRAI_$(SYS)
endif

ifeq ($(SYS),Linux)
ifndef CXX
CXX	= g++
CC	= gcc
endif
LINK	= $(CXX)
CPATH	:= $(BASE)/include:$(BASE)/src:$(RAI_LIBPATH)/include:$(CPATH)
LPATHS	+= $(BASE)/lib $(RAI_LIBPATH)/lib
LIBS += -lrt
#MEXFLAGS  = -cxx #CC='$(CXX)' CXX='$(CXX)' LD='$(CXX)'
SHAREFLAG = -shared #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined
MOC = moc-qt4
UIC = uic-qt4
endif

ifeq ($(SYS), Darwin)
ifndef CXX
CXX	= g++
CC 	= gcc
endif
LINK	 = $(CXX)
CPATH	:= $(BASE)/include:$(BASE)/src:$(CPATH)
LPATH	:= $(BASE)/lib:$(LPATH)
SHAREFLAG = -dynamiclib #-Wl,--warn-unresolved-symbols #-Wl,--no-allow-shlib-undefined
MOC = moc
UIC = uic
endif

ifeq ($(SYS),Cygwin)
CXX	= g++
CC	= g++
LINK	= g++
CPATH	:= $(CPATH):../../include:$(LIBPATH)/include:/usr/X11R6/include
LPATH	:= $(LPATH):$(BASE)/lib:/usr/X11R6/lib
QTDIR 	= /usr/lib/qt3
LDFLAGS	+= -o $(OUTPUT)
SHAREFLAG = -shared
endif

ifeq ($(SYS),MinGW)
CXX	= "$(MINGDIR)/bin/g++"
LINK	= "$(MINGDIR)/bin/g++"
CPATH	:= $(CPATH);../../include;$(LIBPATH)/include;$(MINGDIR)/include
LPATH	:= $(LPATH);$(LIBPATH)/lib_$(SYS);$(LIBPATH)/lib_Cygwin;$(LIBPATH)/lib;$(MINGDIR)/lib
LDFLAGS	+= -o $(OUTPUT)
SHAREFLAG = -shared
ifeq ($(QT),1)
QT 	= $(LIBPATH)/qt-win-4.1.2
endif
endif

ifeq ($(SYS),MSVC)
BASE	:= $(BASE:/cygdrive/c/%=C:/%)
CXX	= "$(MSDEVDIR)/../../VC98/bin/cl"
LINK	= "$(MSDEVDIR)/../../VC98/bin/link"
HOME	= C:/home
QTDIR	= C:/Programme/Qt-2.3.0
MSVC_CPATH := $(MSVC_CPATH);../../include;$(LIBPATH)/include;$(LIBPATH)/stl/stlport;$(MSDEVDIR)/../../VC98/include;$(MSDEVDIR)/../../VC98/alt/include;$(MSDEVDIR)/../../VC98/mfc/include
MSVC_LPATH := $(MSVC_LPATH);$(MSDEVDIR)/../../VC98/mfc/lib;$(MSDEVDIR)/../../VC98/Lib;$(LIBPATH)/lib_MSVC;$(LIBPATH)/lib_Cygwin
CXXFLAGS  += -nologo -c -W3 -GR -GX -Zm500
#	-D"_MSC_VER 1300" -DNOUNICODE -D_GDI32_ -D_MBCS -DQT_DLL -DQT_THREAD_SUPPORT
CXXFLAGSD+= -MLd -Od -Zi
LDFLAGS	 += -nologo -stack:0x1000000
LDFLAGSD += -debug
MSVCLibs += user32.lib
OBJ	:= $(OBJ:%=%bj)
LDFLAGS	+= -out:"$(OUTPUT)"#$(LPATHS:%=-libpath:%)
# gdi32.lib winspool.lib comdlg32.lib
# kernel32.lib
#	   advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib \
#	   odbc32.lib odbccp32.lib
endif
