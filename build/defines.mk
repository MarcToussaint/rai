################################################################################
#
# linking to external libraries
#
################################################################################
# (a tag like `FREEGLUT = 1' can be defined in the make-config as needed)

ARCH = $(shell uname -m)

ifeq ($(OPENMP),1)
CXXFLAGS += -fopenmp -DOPENMP
endif

ifeq ($(CUDA),1)
CXXFLAGS += -DMLR_CUDA
NXX = nvcc #$(MLR_LIBPATH)/cuda/bin/
CPATH   := $(CPATH):$(MLR_LIBPATH)/cuda/include:$(MLR_LIBPATH)/cudaSDK/C/common/inc
ifeq ($(ARCH),x86_64)
LPATHS += $(MLR_LIBPATH)/cuda/lib64 $(MLR_LIBPATH)/cudaSDK/lib
else
LPATHS += $(MLR_LIBPATH)/cuda/lib32 $(MLR_LIBPATH)/cudaSDK/lib
CUDA_EMU = 1
#$(warning WARNING: using cuda EMULATION mode)
endif
ifeq ($(CUDA_EMU),1)  #emulation mode!!
NXXFLAGS = -g -deviceemu
LIBS += -lcudart -lcublasemu #-lcutil
else
NXXFLAGS = -O0 -Xcompiler -fPIC
LIBS += -lcudart -lcublas #-lcutil
endif
endif

ifeq ($(MLR_TESTS),1)
CXXFLAGS += -DMLR_GTEST -DEXAMPLES_AS_TESTS
LPATHS += /usr/src/gtest
LIBS += -lgtest -lpthread
endif

ifeq ($(FREEGLUT),1)
CXXFLAGS  += -DMLR_FREEGLUT
MSVC_CPATH := $(MSVC_CPATH);$(LIBPATH)/freeglut/include
MSVC_LPATH := $(MSVC_LPATH);$(LIBPATH)/freeglut/DebugStatic
LIBS += -lglut -lGLU# -lGL -lX11
CygwinLibs+= -lglut -lGLU -lGL
MSVCLibs  += opengl32.lib glu32.lib vfw32.lib
GL := 1
endif

ifeq ($(GTKGL),1)
CXXFLAGS  += -DMLR_GTKGL
GL := 1
GTK := 1
endif

ifeq ($(FLTK),1)
CXXFLAGS  += -DMLR_FLTK
LIBS += -lfltk -lfltk_gl
GL := 1
endif

ifeq ($(QTGL),1)
CXXFLAGS  += -DMLR_QTGL -DMLR_QT -DQT_DLL# -DNOUNICODE
GL := 1
QT := 1
endif

ifeq ($(GL),1)
CXXFLAGS  += -DMLR_GL
LIBS += -lGLEW -lglut -lGLU -lGL -lX11
endif

ifeq ($(QT),1)
CXXFLAGS  += -DMLR_QT `pkg-config --cflags  Qt5Core Qt5Gui Qt5OpenGL Qt5Svg`
LIBS      += `pkg-config --libs Qt5Core Qt5Gui Qt5OpenGL Qt5Svg`
endif

ifeq ($(GTK),1)
CXXFLAGS += -DMLR_GTK `pkg-config --cflags gtk+-2.0 gtkglext-1.0`
LIBS     += `pkg-config --libs  gtk+-2.0 gtkglext-1.0` -lgthread-2.0
endif

ifeq ($(GTK3),1)
CXXFLAGS += -DMLR_GTK `pkg-config --cflags gtk+-3.0`
LIBS     += `pkg-config --libs  gtk+-3.0`
endif

ifeq ($(GRAPHVIZ),1)
CXXFLAGS += -DMLR_GRAPHVIZ
LIBS += -lcgraph -lgvc
endif

ifeq ($(WX),1)
CXXFLAGS  += -DMLR_WX -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D__WXGTK__ -pthread
CPATH := $(CPATH):/usr/lib/wx/include/gtk2-unicode-release-2.8:/usr/include/wx-2.8
LIBS += -pthread -Wl,-Bsymbolic-functions  -lwx_gtk2u_richtext-2.8 -lwx_gtk2u_aui-2.8 -lwx_gtk2u_xrc-2.8 -lwx_gtk2u_qa-2.8 -lwx_gtk2u_html-2.8 -lwx_gtk2u_adv-2.8 -lwx_gtk2u_core-2.8 -lwx_baseu_xml-2.8 -lwx_baseu_net-2.8 -lwx_baseu-2.8
endif

ifeq ($(ODE),1)
CXXFLAGS  += -DMLR_ODE -DdDOUBLE
LIBS += -lode
endif

ifeq ($(SWIFT),1)
CXXFLAGS  += -DMLR_SWIFT
CPATH	  := $(CPATH):$(LIBPATH)/SWIFT++_1.2/include
LIBS += -lSWIFT++
QHULL := 1
endif

ifeq ($(GJK),1)
DEPEND +=  extern_GJK
endif

ifeq ($(LEWINER),1)
DEPEND += extern_Lewiner
endif

ifeq ($(PLY),1)
DEPEND += extern_ply
endif

ifeq ($(SOLID),1)
CXXFLAGS  += -DMLR_SOLID
CPATH     := $(CPATH):$(LIBPATH)/FreeSOLID-2.1.1/include
LIBS += -lFreeSOLID
endif

ifeq ($(ANN),1)
CXXFLAGS  += -DMLR_ANN
LIBS += -lann
endif

ifeq ($(QHULL),1)
CXXFLAGS  += -DMLR_QHULL
LIBS      += -lqhull
endif

ifeq ($(ARCH_LINUX),1)
CXXFLAGS += -DARCH_LINUX -DATLAS
endif

ifeq ($(OpenML),1)
CXXFLAGS  += -DMLR_OpenML
MSVC_CPATH := $(MSVC_CPATH);$(OpenML)/include
MSVC_LPATH := $(MSVC_LPATH);$(OpenML)/lib
MSVCLibs  += ML10.lib MLU10.lib
endif

ifeq ($(Shark),1)
CXXFLAGS  += -DMLR_Shark
CPATH	  := $(CPATH):$(SHARK)/include
endif

ifeq ($(IT++),1)
CXXFLAGS  += -DMLR_ITpp
CPATH	  := $(CPATH):$(IT++)/include
LPATH	  := $(LPATH):$(IT++)/lib
LIBS += -lit++ -lit++external -lg2c
CygwinLibs+= -lit++ -lit++external -lg2c
endif

ifeq ($(GL2PS),1)
CXXFLAGS  += -DMLR_GL2PS
CPATH	  := $(CPATH):$(GL2PS)
LIBS += -lgl2ps
endif

ifeq ($(GSL),1)
CXXFLAGS  += -DMLR_GSL
LIBS      += -lgsl
endif

ifeq ($(OCTAVE),1)
CXXFLAGS += -DMLR_OCTAVE -D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4
ifeq ($(ARCH_LINUX),1)
CPATH := $(CPATH):/usr/include/octave-3.6.2
LPATHS += /usr/lib/octave/3.6.2
else
CPATH := $(CPATH):/usr/include/octave-3.8.1:/usr/include/octave-3.8.1/octave
LPATHS += /usr/lib/octave-3.8.1
endif
LIBS	+= -loctinterp -loctave
# TIP!!!: run 'mkoctfile --verbose main.cpp' and have a look at the compile options!
endif

ifeq ($(SOIL),1)
CXXFLAGS  += -DTL_SOIL
LIBS    += -lSOIL
endif

##LAPACK MUST BE BEFORE OPENCV! (since OpenCV includes its own lapack binaries, which screw things up...)
ifeq ($(LAPACK),1)
CXXFLAGS  += -DMLR_LAPACK
CPATH	  := $(LIBPATH)/lapack/include:$(CPATH)
LIBS += -llapack -lblas
MSVC_CPATH := $(LIBPATH)/lapack/include;$(MSVC_CPATH)
CygwinLibs+= -lcblas -latlas -lclapack -lcblaswr -lI77 -lF77
MinGWLibs += -lcblas -lclapack -lcblaswr -latlas -lI77 -lF77 -lcygwin
#MSVCLibs += libcblas.a libclapack.a libcblaswr.a libatlas.a libF77.a libc.lib libcygwin.a
endif

ifeq ($(OPENCV),1)
  ifeq ($(OLDUBUNTU),1)
    CXXFLAGS  += -DMLR_OPENCV `pkg-config --cflags-only-other opencv-2.3.1`
    CXXFLAGS += `pkg-config --cflags-only-I opencv-2.3.1`
    %CPATH := $(CPATH):$(IFLAGS:\-I%=\:%\:) % is it possible to add the includes to the CPATH?
    LIBS      += `pkg-config --libs opencv-2.3.1`
  else 
    CXXFLAGS  += -DMLR_OPENCV `pkg-config --cflags-only-other opencv`
    CXXFLAGS += `pkg-config --cflags-only-I opencv`
    %CPATH := $(CPATH):$(IFLAGS:\-I%=\:%\:) % is it possible to add the includes to the CPATH?
    LIBS      += `pkg-config --libs opencv`
  endif
endif

ifeq ($(HSL),1)
CXXFLAGS  += -DMLR_HSL
CPATH	  := $(CPATH):$(LIBPATH)/HSL-archive/include
LPATH	  := $(LPATH):$(LIBPATH)/HSL-archive/lib
LIBS += -lHSL-debr
endif

ifeq ($(PLIB),1)
CXXFLAGS  += -DMLR_PLIB
LIBS += -lplibjs -lplibul
endif

ifeq ($(TONY),1)
CXXFLAGS  += -DMLR_TONY
CPATH   := $(CPATH):$(LIBPATH)/tony_mdp/include
LIBS += -lmdp
endif

ifeq ($(DAI),1)
CXXFLAGS  += -DMLR_DAI
CPATH   := $(CPATH):$(LIBPATH)/libDAI-0.2.2/include
#LPATH   := $(LPATH):$(LIBPATH)/libDAI-0.2.2/lib
LIBS += -ldai
endif

ifeq ($(IBDS),1)
CPATH := $(CPATH):$(MLR_LIBPATH)/ibds/include
LPATHS += $(MLR_LIBPATH)/ibds/lib
LIBS += -lDynamicSimulation -lCollisionDetection -lMath -lLibBulletCollision -lLibLinearMath -lqhull
endif

ifeq ($(PCL),1)
EIGEN = 1
#QHULL = 1
CXXFLAGS  +=  -DMLR_PCL -DEIGEN_USE_NEW_STDVECTOR -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
LIBS += -lpcl_keypoints -lpcl_visualization -lpcl_registration -lpcl_segmentation -lpcl_features -lpcl_surface -lpcl_tracking -lpcl_filters -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -lpcl_octree -lpcl_common
CPATH := $(CPATH):/usr/include/pcl-1.7:

#CPATH := $(CPATH):/opt/ros/$(ROS_VERSION)/include/pcl_ros:/usr/include/eigen3:/usr/include/pcl-1.7:
#CPATH := $(CPATH):/usr/include/eigen3:/home/lib/include/pcl-1.7:
#LPATH += /opt/ros/$(ROS_VERSION)/lib
#LPATH += /home/lib/lib/
#FREENECT = 1
endif

ifeq ($(EIGEN),1)
CXXFLAGS += -DMT_EIGEN
CPATH := $(CPATH):/usr/include/eigen3
endif

ifeq ($(HYBRID_AUTOMATON),1)
CXXFLAGS += -DMT_HYBRID_AUTOMATON
LIBS += -lhybrid_automaton -ltinyxml
CPATH := $(CPATH):/home/johannes/src/rswin/hybrid_automaton/include
LPATH := $(LPATH):/home/johannes/src/rswin/hybrid_automaton/build
endif

ifeq ($(FREENECT),1)
CXXFLAGS += -DMLR_FREENECT
ifeq ($(FREENECT_LOC),1)
CPATH := $(HOME)/git/libfreenect/include:$(CPATH):/usr/include/libusb-1.0
LPATH += $(HOME)/git/libfreenect/build/lib
else
CPATH := $(CPATH):/usr/include/libusb-1.0
endif
LIBS += -lfreenect -lusb-1.0 -lpthread
endif


ifeq ($URGLASER),1)
CPATH     := $(CPATH):$(LIBPATH)/urg-0.8.16/include/c
CPATH     := $(CPATH):$(LIBPATH)/urg-0.8.16/include/cpp
LPATH     := $(LPATH):$(LIBPATH)/urg-0.8.16/src/c/urg/.libs
LPATH     := $(LPATH):$(LIBPATH)/urg-0.8.16/src/c/system/.libs
LPATH     := $(LPATH):$(LIBPATH)/urg-0.8.16/src/c/connection/.libs
LIBS += -lc_urg -lc_urg_system -lc_urg_connection
endif

ifeq ($(DYNAMIXEL),1)
CXXFLAGS  += -DMLR_DYNAMIXEL
CPATH     := $(CPATH):$(LIBPATH)/dynamixel/include
LPATH     := $(LPATH):$(LIBPATH)/dynamixel/lib
LIBS      += -ldxl
endif

ifeq ($(BUMBLE),1)
CXXFLAGS  += -DMLR_BUMBLE
#CPATH     := $(CPATH):$(LIBPATH)/pgrlibdcstereo/
#LPATH     := $(LPATH):$(LIBPATH)/pgrlibdcstereo/
LIBS += -ldc1394 # -lpgrlibdcstereo
endif

ifeq ($(FELZ),1)
CXXFLAGS  += -DMLR_FELZ
CPATH     := $(CPATH):$(LIBPATH)/libcolorseg/include
LPATH     := $(LPATH):$(LIBPATH)/libcolorseg/lib
LIBS += -lcolorseg
endif

ifeq ($(ESS),1)
CXXFLAGS  += -DMLR_ESS
CPATH     := $(CPATH):$(LIBPATH)/blaschko-ESS-1.1/include
LPATH     := $(LPATH):$(LIBPATH)/blaschko-ESS-1.1/lib
LIBS += -less
endif

ifeq ($(SURF),1)
CPATH     := $(CPATH):$(LIBPATH)/opensurf/
LPATH     := $(LPATH):$(LIBPATH)/opensurf/
LIBS += -lopensurf_$(ARCH)
endif

ifeq ($(PTHREAD),1)
CXXFLAGS  += -DMLR_PTHREAD
LIBS += -lpthread -lX11
endif

ifeq ($(PHYSX),1)
CXXFLAGS += -DMLR_PHYSX -D_DEBUG -DPX_DISABLE_FLUIDS -DCORELIB -DPX32 -DLINUX
CPATH := $(CPATH):$(HOME)/opt/include:$(HOME)/opt/include/physx
#PhysX/Include:$(MLR_LIBPATH)/PhysX/Include/extensions:$(MLR_LIBPATH)/PhysX/Include/foundation:$(MLR_LIBPATH)/PhysX/Include/deprecated
LPATH := $(HOME)/opt/lib/physx:$(LPATH)
LIBS += -Wl,--start-group -lpthread -lrt\
-lLowLevel \
-lLowLevelCloth \
-lPhysX3CharacterKinematic \
-lPhysX3 \
-lPhysX3Common \
-lPhysX3Cooking \
-lPhysX3Extensions \
-lPhysX3Vehicle \
-lPhysXProfileSDK \
-lPhysXVisualDebuggerSDK \
-lPvdRuntime \
-lPxTask \
-lSceneQuery \
-lSimulationController 
endif

ifeq ($(PORTAUDIO),1)
CXXFLAGS  += -DMLR_PORTAUDIO
LPATHS += $(HOME)/opt/portaudio/lib/.libs
CPATHS += $(HOME)/opt/portaudio/include
LIBS += -lportaudio
endif

ifeq ($(G4),1)
CXXFLAGS += -DG4_INSTALLED
LIBS += -lG4Track -lusb-1.0
endif

ifeq ($(PTHREAD),1)
LIBS += -lpthread
endif

ifeq ($(X264),1)
CXXFLAGS += -DX264_INSTALLED
endif

ifeq ($(ROS),1)
CXXFLAGS  += -DMLR_ROS
ROSP=pr2_mechanism/pr2_controller_interface\
pr2_mechanism/pr2_mechanism_model\
pr2_mechanism/pr2_hardware_interface\
ros_control/hardware_interface\
ros_control/controller_interface

CPATHS += /opt/ros/$(ROS_VERSION)/include $(ROSP:%=/opt/ros/$(ROS_VERSION)/stacks/%/include)
LPATHS += /opt/ros/$(ROS_VERSION)/lib $(ROSP:%=/opt/ros/$(ROS_VERSION)/stacks/%/lib)

ifndef ROS_VERSION
  ROS_VERSION:=indigo
endif

ifeq ($(ROS_VERSION),groovy)
CXXFLAGS  += -DMLR_ROS_GROOVY
LIBS += -rdynamic -lpr2_mechanism_model -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lboost_signals-mt -lxmlrpcpp -ltinyxml -lboost_filesystem-mt -lclass_loader -lPocoFoundation -ldl -lrosconsole -lboost_regex-mt -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lboost_date_time-mt -lboost_system-mt -lboost_thread-mt -lpthread -lcpp_common -lorocos-kdl
endif
ifeq ($(ROS_VERSION),indigo)
CXXFLAGS  += -DMLR_ROS_INDIGO
LIBS += -rdynamic -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lxmlrpcpp -ltinyxml -lclass_loader -lPocoFoundation -ldl -lrosconsole -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lpthread -lcpp_common -lorocos-kdl -ltf -lboost_system -lpcl_ros_tf
endif
ifeq ($(ROS_VERSION),kinetic)
CXXFLAGS  += -DMLR_ROS_KINETIC
LIBS += -rdynamic -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lxmlrpcpp -ltinyxml -lclass_loader -lPocoFoundation -ldl -lrosconsole -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lpthread -lcpp_common -lorocos-kdl -ltf -lboost_system -lpcl_ros_tf
CPATHS += $(HOME)/git/catkin_ws/install/include
LPATHS += $(HOME)/git/catkin_ws/install/lib
endif


endif
