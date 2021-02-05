################################################################################
#
# linking to external libraries
#
################################################################################
# (a tag like `FREEGLUT = 1' can be defined in the make-config as needed)

ARCH = $(shell uname -m)

ifeq ($(RAI_CMAKE),1)
LPATHS += $(BASE)/../build
LIBS += -lrai
endif

ifeq ($(JSON),1)
DEPEND_UBUNTU += libjsoncpp-dev
LIBS += -ljsoncpp
endif

ifeq ($(OPENMP),1)
CXXFLAGS += -fopenmp -DOPENMP
endif

ifeq ($(PYBIND),1)
DEPEND_UBUNTU += python3-dev python3 python3-numpy python3-pip python3-distutils
#pybind11-dev NO! don't use the ubuntu package. Instead use:
#  pip3 install --user pybind11
CXXFLAGS += -DRAI_PYBIND `python3-config --cflags` `python3 -m pybind11 --includes`
LIBS += `python3-config --ldflags`
#CPATH := $(CPATH):$(BASE)/../pybind11/include::$(BASE)/../../pybind11/include
endif

ifeq ($(X11),1)
DEPEND_UBUNTU += libx11-dev
CXXFLAGS += -DRAI_X11
LIBS += -lX11
endif

ifeq ($(PNG),1)
DEPEND_UBUNTU += libpng-dev
CXXFLAGS += -DRAI_PNG
LIBS += -lpng
endif

ifeq ($(FCL),1)
DEPEND_UBUNTU += libfcl-dev
CXXFLAGS  += -DRAI_FCL
LIBS      += -lfcl
endif

ifeq ($(CCD),1)
DEPEND += extern_ccd
CXXFLAGS += -DRAI_CCD
endif

ifeq ($(PLY),1)
DEPEND += extern_ply
CXXFLAGS += -DRAI_PLY
endif

ifeq ($(GJK),1)
DEPEND += extern_GJK
CXXFLAGS += -DRAI_GJK
endif

ifeq ($(Lewiner),1)
DEPEND += extern_Lewiner
CXXFLAGS += -DRAI_Lewiner
endif

ifeq ($(ASSIMP),1)
DEPEND_UBUNTU += libassimp-dev
CXXFLAGS += -DRAI_ASSIMP
LIBS += -lassimp
endif

ifeq ($(CERES),1)
DEPEND_UBUNTU += libceres-dev
CXXFLAGS += -DRAI_CERES
#CPATHS += $(HOME)/git/ceres-solver/include
#CPATHS += $(HOME)/git/ceres-solver/build/config
#CPATHS += $(HOME)/git/ceres-solver/internal/ceres/miniglog
#LPATHS += $(HOME)/git/ceres-solver/build/lib
LIBS += -lceres
#-lglog -lcholmod -llapack -lblas -lpthread
#/usr/lib/x86_64-linux-gnu/libspqr.so /usr/lib/x86_64-linux-gnu/libtbbmalloc.so /usr/lib/x86_64-linux-gnu/libtbb.so /usr/lib/x86_64-linux-gnu/libcholmod.so /usr/lib/x86_64-linux-gnu/libccolamd.so /usr/lib/x86_64-linux-gnu/libcamd.so /usr/lib/x86_64-linux-gnu/libcolamd.so /usr/lib/x86_64-linux-gnu/libamd.so /usr/lib/x86_64-linux-gnu/liblapack.so /usr/lib/x86_64-linux-gnu/libf77blas.so /usr/lib/x86_64-linux-gnu/libatlas.so /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so /usr/lib/x86_64-linux-gnu/librt.so /usr/lib/x86_64-linux-gnu/libcxsparse.so /usr/lib/x86_64-linux-gnu/liblapack.so /usr/lib/x86_64-linux-gnu/libf77blas.so /usr/lib/x86_64-linux-gnu/libatlas.so /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so /usr/lib/x86_64-linux-gnu/librt.so /usr/lib/x86_64-linux-gnu/libcxsparse.so /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1 -lpthread /usr/lib/x86_64-linux-gnu/libglog.so
endif

ifeq ($(NLOPT),1)
DEPEND_UBUNTU += libnlopt-cxx-dev gfortran
CXXFLAGS += -DRAI_NLOPT `pkg-config --cflags nlopt`
LIBS     += `pkg-config --libs nlopt`
endif

ifeq ($(IPOPT),1)
DEPEND_UBUNTU += coinor-libipopt-dev
CXXFLAGS += -DRAI_IPOPT `pkg-config --cflags ipopt`
LIBS     += `pkg-config --libs ipopt`
endif

ifeq ($(OMPL),1)
CXXFLAGS += -DRAI_OMPL
CPATHS  += /usr/local/include/ompl-1.6/
endif

ifeq ($(CUDA),1)
CXXFLAGS += -DRAI_CUDA
NXX = nvcc #$(RAI_LIBPATH)/cuda/bin/
CPATH   := $(CPATH):$(RAI_LIBPATH)/cuda/include:$(RAI_LIBPATH)/cudaSDK/C/common/inc
ifeq ($(ARCH),x86_64)
LPATHS += $(RAI_LIBPATH)/cuda/lib64 $(RAI_LIBPATH)/cudaSDK/lib
else
LPATHS += $(RAI_LIBPATH)/cuda/lib32 $(RAI_LIBPATH)/cudaSDK/lib
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

ifeq ($(RAI_TESTS),1)
CXXFLAGS += -DRAI_GTEST -DEXAMPLES_AS_TESTS
LPATHS += /usr/src/gtest
LIBS += -lgtest -lpthread
endif

ifeq ($(GLFW),1)
DEPEND_UBUNTU += libglfw3-dev
CXXFLAGS  += -DRAI_GLFW
LIBS      += -lglfw
GL := 1
endif

ifeq ($(FREEGLUT),1)
DEPEND_UBUNTU += freeglut3-dev
CXXFLAGS  += -DRAI_FREEGLUT
MSVC_CPATH := $(MSVC_CPATH);$(LIBPATH)/freeglut/include
MSVC_LPATH := $(MSVC_LPATH);$(LIBPATH)/freeglut/DebugStatic
LIBS += -lglut -lGLU# -lGL -lX11
CygwinLibs+= -lglut -lGLU -lGL
MSVCLibs  += opengl32.lib glu32.lib vfw32.lib
GL := 1
endif

ifeq ($(GTKGL),1)
CXXFLAGS  += -DRAI_GTKGL
GL := 1
GTK := 1
endif

ifeq ($(FLTK),1)
CXXFLAGS  += -DRAI_FLTK
LIBS += -lfltk -lfltk_gl
GL := 1
endif

ifeq ($(QTGL),1)
CXXFLAGS  += -DRAI_QTGL -DRAI_QT -DQT_DLL# -DNOUNICODE
GL := 1
QT := 1
endif

ifeq ($(GL),1)
DEPEND_UBUNTU += libglew-dev freeglut3-dev
CXXFLAGS  += -DRAI_GL
LIBS += -lGLEW -lglut -lGLU -lGL -lX11
endif

ifeq ($(QT),1)
CXXFLAGS  += -DRAI_QT `pkg-config --cflags  Qt5Core Qt5Gui Qt5OpenGL Qt5Svg`
LIBS      += `pkg-config --libs Qt5Core Qt5Gui Qt5OpenGL Qt5Svg`
endif

ifeq ($(GTK),1)
CXXFLAGS += -DRAI_GTK `pkg-config --cflags gtk+-2.0 gtkglext-1.0`
LIBS     += `pkg-config --libs  gtk+-2.0 gtkglext-1.0` -lgthread-2.0
endif

ifeq ($(GTK3),1)
CXXFLAGS += -DRAI_GTK `pkg-config --cflags gtk+-3.0`
LIBS     += `pkg-config --libs  gtk+-3.0`
endif

ifeq ($(GRAPHVIZ),1)
DEPEND_UBUNTU += graphviz graphviz-dev
CXXFLAGS += -DRAI_GRAPHVIZ
LIBS += -lcgraph -lgvc
endif

ifeq ($(WX),1)
CXXFLAGS  += -DRAI_WX -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -D__WXGTK__ -pthread
CPATH := $(CPATH):/usr/lib/wx/include/gtk2-unicode-release-2.8:/usr/include/wx-2.8
LIBS += -pthread -Wl,-Bsymbolic-functions  -lwx_gtk2u_richtext-2.8 -lwx_gtk2u_aui-2.8 -lwx_gtk2u_xrc-2.8 -lwx_gtk2u_qa-2.8 -lwx_gtk2u_html-2.8 -lwx_gtk2u_adv-2.8 -lwx_gtk2u_core-2.8 -lwx_baseu_xml-2.8 -lwx_baseu_net-2.8 -lwx_baseu-2.8
endif

ifeq ($(ANN),1)
DEPEND_UBUNTU += libann-dev
CXXFLAGS  += -DRAI_ANN
LIBS += -lann
endif

ifeq ($(QHULL),1)
DEPEND_UBUNTU += libqhull-dev
CXXFLAGS  += -DRAI_QHULL
LIBS      += -lqhull
endif

ifeq ($(ARCH_LINUX),1)
CXXFLAGS += -DARCH_LINUX -DATLAS
endif

ifeq ($(OpenML),1)
CXXFLAGS  += -DRAI_OpenML
MSVC_CPATH := $(MSVC_CPATH);$(OpenML)/include
MSVC_LPATH := $(MSVC_LPATH);$(OpenML)/lib
MSVCLibs  += ML10.lib MLU10.lib
endif

ifeq ($(Shark),1)
CXXFLAGS  += -DRAI_Shark
CPATH	  := $(CPATH):$(SHARK)/include
endif

ifeq ($(IT++),1)
CXXFLAGS  += -DRAI_ITpp
CPATH	  := $(CPATH):$(IT++)/include
LPATHS += $(IT++)/lib
LIBS += -lit++ -lit++external -lg2c
CygwinLibs+= -lit++ -lit++external -lg2c
endif

ifeq ($(GL2PS),1)
CXXFLAGS  += -DRAI_GL2PS
CPATH	  := $(CPATH):$(GL2PS)
LIBS += -lgl2ps
endif

ifeq ($(GSL),1)
CXXFLAGS  += -DRAI_GSL
LIBS      += -lgsl
endif

ifeq ($(OCTAVE),1)
CXXFLAGS += -DRAI_OCTAVE -D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4
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
DEPEND_UBUNTU += liblapack-dev libf2c2-dev
CXXFLAGS  += -DRAI_LAPACK
CPATH	  := $(LIBPATH)/lapack/include:$(CPATH)
LIBS += -llapack -lblas
MSVC_CPATH := $(LIBPATH)/lapack/include;$(MSVC_CPATH)
CygwinLibs+= -lcblas -latlas -lclapack -lcblaswr -lI77 -lF77
MinGWLibs += -lcblas -lclapack -lcblaswr -latlas -lI77 -lF77 -lcygwin
#MSVCLibs += libcblas.a libclapack.a libcblaswr.a libatlas.a libF77.a libc.lib libcygwin.a
endif

ifeq ($(OPENCV),1)
DEPEND_UBUNTU += libopencv-dev
  ifeq ($(OLDUBUNTU),1)
    CXXFLAGS  += -DRAI_OPENCV `pkg-config --cflags-only-other opencv-2.3.1`
    CXXFLAGS += `pkg-config --cflags-only-I opencv-2.3.1`
    %CPATH := $(CPATH):$(IFLAGS:\-I%=\:%\:) % is it possible to add the includes to the CPATH?
    LIBS      += `pkg-config --libs opencv-2.3.1`
  else 
    CXXFLAGS  += -DRAI_OPENCV `pkg-config --cflags-only-other opencv`
    CXXFLAGS += `pkg-config --cflags-only-I opencv`
    %CPATH := $(CPATH):$(IFLAGS:\-I%=\:%\:) % is it possible to add the includes to the CPATH?
    LIBS      += `pkg-config --libs opencv`
  endif
endif

ifeq ($(OPENCV4),1)
CXXFLAGS  += -DRAI_OPENCV
CPATH := $(HOME)/opt/include/opencv4/:$(CPATH)
LIBS += -lopencv_core -lopencv_highgui
endif

ifeq ($(HSL),1)
CXXFLAGS  += -DRAI_HSL
CPATH	  := $(CPATH):$(LIBPATH)/HSL-archive/include
LPATHS += $(LIBPATH)/HSL-archive/lib
LIBS += -lHSL-debr
endif

ifeq ($(PLIB),1)
CXXFLAGS  += -DRAI_PLIB
LIBS += -lplibjs -lplibul
endif

ifeq ($(TONY),1)
CXXFLAGS  += -DRAI_TONY
CPATH   := $(CPATH):$(LIBPATH)/tony_mdp/include
LIBS += -lmdp
endif

ifeq ($(DAI),1)
CXXFLAGS  += -DRAI_DAI
CPATH   := $(CPATH):$(LIBPATH)/libDAI-0.2.2/include
#LPATH   := $(LPATH):$(LIBPATH)/libDAI-0.2.2/lib
LIBS += -ldai
endif

ifeq ($(IBDS),1)
CPATH := $(CPATH):$(RAI_LIBPATH)/ibds/include
LPATHS += $(RAI_LIBPATH)/ibds/lib
LIBS += -lDynamicSimulation -lCollisionDetection -lMath -lLibBulletCollision -lLibLinearMath -lqhull
endif

ifeq ($(PCL),1)
DEPEND_UBUNTU += libpcl-dev
EIGEN = 1
#QHULL = 1
CXXFLAGS  +=  -DRAI_PCL -DEIGEN_USE_NEW_STDVECTOR -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
LIBS += -lpcl_keypoints -lpcl_visualization -lpcl_registration -lpcl_segmentation -lpcl_features -lpcl_surface -lpcl_tracking -lpcl_filters -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -lpcl_octree -lpcl_common -lboost_system
CPATH := $(CPATH):/usr/include/pcl-1.8:

#CPATH := $(CPATH):/opt/ros/$(ROS_VERSION)/include/pcl_ros:/usr/include/eigen3:/usr/include/pcl-1.7:
#CPATH := $(CPATH):/usr/include/eigen3:/home/lib/include/pcl-1.7:
#LPATH += /opt/ros/$(ROS_VERSION)/lib
#LPATH += /home/lib/lib/
#FREENECT = 1
endif

ifeq ($(EIGEN),1)
DEPEND_UBUNTU += libeigen3-dev
CXXFLAGS += -DRAI_EIGEN -fopenmp
LDFLAGS += -fopenmp
CPATH := $(CPATH):/usr/include/eigen3
endif

ifeq ($(HYBRID_AUTOMATON),1)
CXXFLAGS += -DRAI_HYBRID_AUTOMATON
LIBS += -lhybrid_automaton -ltinyxml
CPATH := $(CPATH):/home/johannes/src/rswin/hybrid_automaton/include
LPATH := $(LPATH):/home/johannes/src/rswin/hybrid_automaton/build
endif

ifeq ($(FREENECT),1)
CXXFLAGS += -DRAI_FREENECT
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
LPATHS += $(LIBPATH)/urg-0.8.16/src/c/urg/.libs
LPATHS += $(LIBPATH)/urg-0.8.16/src/c/system/.libs
LPATHS += $(LIBPATH)/urg-0.8.16/src/c/connection/.libs
LIBS += -lc_urg -lc_urg_system -lc_urg_connection
endif

ifeq ($(DYNAMIXEL),1)
CXXFLAGS  += -DRAI_DYNAMIXEL
CPATH     := $(CPATH):$(LIBPATH)/dynamixel/include
LPATHS += $(LIBPATH)/dynamixel/lib
LIBS      += -ldxl
endif

ifeq ($(BUMBLE),1)
CXXFLAGS  += -DRAI_BUMBLE
#CPATH     := $(CPATH):$(LIBPATH)/pgrlibdcstereo/
#LPATHS += $(LIBPATH)/pgrlibdcstereo/
LIBS += -ldc1394 # -lpgrlibdcstereo
endif

ifeq ($(FELZ),1)
CXXFLAGS  += -DRAI_FELZ
CPATH     := $(CPATH):$(LIBPATH)/libcolorseg/include
LPATHS += $(LIBPATH)/libcolorseg/lib
LIBS += -lcolorseg
endif

ifeq ($(ESS),1)
CXXFLAGS  += -DRAI_ESS
CPATH     := $(CPATH):$(LIBPATH)/blaschko-ESS-1.1/include
LPATHS += $(LIBPATH)/blaschko-ESS-1.1/lib
LIBS += -less
endif

ifeq ($(SURF),1)
CPATH     := $(CPATH):$(LIBPATH)/opensurf/
LPATHS += $(LIBPATH)/opensurf/
LIBS += -lopensurf_$(ARCH)
endif

ifeq ($(PTHREAD),1)
CXXFLAGS  += -DRAI_PTHREAD
LIBS += -lpthread
endif

ifeq ($(SWIFT),1)
DEPEND += extern_SWIFT extern_SWIFT_decomposer
CXXFLAGS += -DRAI_SWIFT
endif

ifeq ($(ODE),1)
CXXFLAGS  += -DRAI_ODE -DdDOUBLE
LIBS += -lode
endif

ifeq ($(SOLID),1)
CXXFLAGS  += -DRAI_SOLID
CPATH     := $(CPATH):$(LIBPATH)/FreeSOLID-2.1.1/include
LIBS += -lFreeSOLID
endif

ifeq ($(DART),1)
CXXFLAGS += -DRAI_DART -std=c++14
CPATH := $(CPATH):$(HOME)/git/dart/build:$(HOME)/git/dart
LPATHS += $(HOME)/git/dart/build/lib
LIBS += -ldart-gui -ldart-utils-urdf -ldart-utils -ldart -lboost_system
endif

ifeq ($(PHYSX),1)
CXXFLAGS += -DRAI_PHYSX -D_DEBUG -DPX_DISABLE_FLUIDS -DCORELIB -DPX32 -DLINUX
CPATH := $(CPATH):$(HOME)/opt/physx3.4/include:$(HOME)/opt/physx3.4/include/physx
#PhysX/Include:$(RAI_LIBPATH)/PhysX/Include/extensions:$(RAI_LIBPATH)/PhysX/Include/foundation:$(RAI_LIBPATH)/PhysX/Include/deprecated
LPATHS += $(HOME)/opt/physx3.4/lib
LIBS += -lpthread -lrt\
-lPhysX3Extensions -lPhysX3_x64 -lPhysX3Cooking_x64 -lPhysX3Common_x64 -lPxFoundation_x64

#Physx-3.3:
#-lLowLevel \
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

ifeq ($(BULLET_LOCAL),1)
#BULLET_PATH=$(HOME)/git/bullet3
CXXFLAGS  += -DRAI_BULLET -DBT_USE_DOUBLE_PRECISION
CPATH := $(HOME)/opt/include/bullet/:$(CPATH)
#LPATH := $(BULLET_PATH)/bin:$(LPATH)
#CPATH := $(CPATH):$(BULLET_PATH)/src
#btLIB = _gmake_x64_release
LIBS += -lBulletSoftBody -lBulletDynamics -lBulletCollision  -lLinearMath
endif

ifeq ($(BULLET),1)
DEPEND_UBUNTU += libbullet-dev
CXXFLAGS += -DRAI_BULLET `pkg-config --cflags bullet`
LIBS     += `pkg-config --libs bullet`
endif

ifeq ($(PORTAUDIO),1)
CXXFLAGS  += -DRAI_PORTAUDIO
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

CXXFLAGS  += -DRAI_ROS
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
CXXFLAGS  += -DRAI_ROS_GROOVY
LIBS += -rdynamic -lpr2_mechanism_model -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lboost_signals-mt -lxmlrpcpp -ltinyxml -lboost_filesystem-mt -lclass_loader -lPocoFoundation -ldl -lrosconsole -lboost_regex-mt -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lboost_date_time-mt -lboost_system-mt -lboost_thread-mt -lpthread -lcpp_common -lorocos-kdl
endif
ifeq ($(ROS_VERSION),indigo)
CXXFLAGS  += -DRAI_ROS_INDIGO
LIBS += -rdynamic -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lxmlrpcpp -ltinyxml -lclass_loader -lPocoFoundation -ldl -lrosconsole -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lpthread -lcpp_common -lorocos-kdl -ltf -lboost_system -lpcl_ros_tf
endif
ifeq ($(ROS_VERSION),kinetic)
DEPEND_UBUNTU += ros-kinetic-desktop ros-kinetic-object-recognition-msgs ros-kinetic-ar-track-alvar-msgs
CXXFLAGS  += -DRAI_ROS_KINETIC 
LIBS += -rdynamic -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lcollada_parser -lrosconsole_bridge -lroscpp -lxmlrpcpp -ltinyxml -lclass_loader -lPocoFoundation -ldl -lrosconsole -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lpthread -lcpp_common -lorocos-kdl -ltf -lboost_system
endif
ifeq ($(ROS_VERSION),melodic)
DEPEND_UBUNTU += ros-melodic-desktop ros-melodic-object-recognition-msgs ros-melodic-ar-track-alvar-msgs
CXXFLAGS  += -DRAI_ROS_MELODIC 
LIBS += -rdynamic -lkdl_parser -lurdf -lurdfdom_model -lurdfdom_model_state -lurdfdom_sensor -lurdfdom_world -lrosconsole_bridge -lroscpp -lxmlrpcpp -ltinyxml -lclass_loader -lPocoFoundation -ldl -lrosconsole -llog4cxx -lroslib -lmessage_filters -lconsole_bridge -lroscpp_serialization -lrostime -lpthread -lcpp_common -lorocos-kdl -ltf -lboost_system
endif

endif
