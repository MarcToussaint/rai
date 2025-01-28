cmake_minimum_required(VERSION 3.5)

project(rai_devel)

## variable default values
set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "..." FORCE)
endif()
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN;${CMAKE_INSTALL_PREFIX}/lib")

## options
option(USE_PHYSX "link to physx" ON)
option(USE_H5 "link to h5 library" ON)
option(USE_FREETYPE "link to freetype2 library" ON)
option(BUILD_TESTS "build tests" OFF)

## compile options
add_compile_options(
  -O3 -g -Wall -Wpedantic
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++14>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-terminate>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-array-bounds>
  )

################################################################################

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL8 -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG -DRAI_PLY
  -DRAI_GJK -DRAI_CCD -DRAI_FCL -DRAI_Lewiner -DRAI_VHACD
  )

################################################################################

include_directories(
  src
  $ENV{HOME}/.local/include
  /usr/include/eigen3
  )

file(GLOB SRC_Core src/Core/*.cpp)
file(GLOB SRC_Optim src/Optim/*.cpp)
file(GLOB SRC_Geo src/Geo/*.cpp)
file(GLOB SRC_Algo src/Algo/*.cpp)
file(GLOB SRC_Search src/Search/*.cpp)
file(GLOB SRC_PathAlgos src/PathAlgos/*.cpp)
file(GLOB SRC_Gui src/Gui/*.cpp)
file(GLOB SRC_Kin src/Kin/*.cpp)
file(GLOB SRC_KOMO src/KOMO/*.cpp)
file(GLOB SRC_Logic src/Logic/*.cpp)
file(GLOB SRC_LGP src/LGP/*.cpp)
file(GLOB SRC_Control src/Control/*.cpp)

file(GLOB SRC_GeoGJK src/Geo/GJK/*.c)
file(GLOB SRC_GeoPly src/Geo/ply/*.c)
file(GLOB SRC_GeoCcd src/Geo/ccd_rai/*.c)
file(GLOB SRC_GeoLewiner src/Geo/Lewiner/*.cpp)

add_library(raiobjs OBJECT
  ${SRC_Core}
  ${SRC_Optim}
  ${SRC_Geo}
  ${SRC_GeoGJK}
  ${SRC_GeoPly}
  ${SRC_GeoCcd}
  ${SRC_GeoLewiner}
  ${SRC_Algo}
  ${SRC_Search}
  ${SRC_PathAlgos}
  ${SRC_Gui}
  ${SRC_Kin}
  ${SRC_KOMO}
  ${SRC_Logic}
  ${SRC_LGP}
  ${SRC_Control}
  )

################################################################################

link_directories(
  $ENV{HOME}/.local/lib
  )

if(USE_H5)
  link_directories(/usr/lib/x86_64-linux-gnu/hdf5/serial)
endif()

if(USE_PHYSX)
  link_directories($ENV{HOME}/.local/lib/PhysX)
endif()

set_property(TARGET raiobjs PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(rai SHARED $<TARGET_OBJECTS:raiobjs>)
add_library(rai_static STATIC $<TARGET_OBJECTS:raiobjs>)

################################################################################

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#target_link_libraries (rai Eigen3::Eigen)
find_package(PkgConfig REQUIRED)

if(USE_FREETYPE)
  find_package(Freetype REQUIRED)
  add_definitions(-DRAI_FREETYPE)
  include_directories(rai ${FREETYPE_INCLUDE_DIRS})
  target_link_libraries(rai PRIVATE ${FREETYPE_LIBRARIES})
  message(STATUS "[rai] using freetype2 libs: " ${FREETYPE_LIBRARIES})
endif()

if(USE_H5)
  find_package(HDF5 REQUIRED)
  add_definitions(-DRAI_H5)
  include_directories(${HDF5_INCLUDE_DIRS})
  target_link_libraries(rai PRIVATE hdf5_cpp hdf5 z)
  message(STATUS "[rai] using H5 libs: hdf5 hdf5_cpp in " ${HDF5_INCLUDE_DIRS})
endif()

if(USE_PHYSX)
  add_definitions(-DRAI_PHYSX -DNDEBUG -DPX_DISABLE_FLUIDS -DCORELIB -DPX32 -DLINUX)
  target_link_libraries(rai PRIVATE PhysXExtensions_static_64 PhysX_static_64 PhysXCooking_static_64 PhysXCommon_static_64 PhysXFoundation_static_64)
  message(STATUS "[rai] using physx libs from ~/.local")
endif()

if(USE_OPENCV)
  add_definitions(-DRAI_OPENCV)
  include_directories($ENV{HOME}/.local/include/opencv4)
  target_link_libraries(rai opencv_core opencv_highgui opencv_imgproc opencv_videoio)
  message(STATUS "[rai] using OpenCV libs from ~/.local")
endif()

pkg_check_modules(DEPS REQUIRED glfw3 glew blas lapack jsoncpp libpng assimp qhull)
find_package(GLUT REQUIRED)

message(STATUS "[rai] pkg-config libraries: " "${DEPS_LIBRARIES}")

# default/required dependencies
target_link_libraries(rai PRIVATE
  ${DEPS_LIBRARIES}
  ${GLUT_LIBRARIES}
  fcl ccd ANN
  )

################################################################################

get_property(_defs DIRECTORY PROPERTY COMPILE_DEFINITIONS)
message(STATUS "[rai] compiler flags: " "${_defs}")
message(STATUS "[rai] source dir: " "${CMAKE_SOURCE_DIR}")

################################################################################

add_executable(kinEdit bin/src_kinEdit/main.cpp)
target_link_libraries(kinEdit rai)

add_executable(meshTool bin/src_meshTool/main.cpp)
target_link_libraries(meshTool rai)

if(BUILD_TESTS)
add_executable(test_timingOpt test/Control/timingOpt/main.cpp)
target_link_libraries(test_timingOpt rai)
add_executable(test_mobileBase test/Control/mobileBase/main.cpp)
target_link_libraries(test_mobileBase rai)
add_executable(test_switches test/KOMO/switches/main.cpp)
target_link_libraries(test_switches rai)
add_executable(test_manip test/KOMO/manip/main.cpp)
target_link_libraries(test_manip rai)
add_executable(test_timeopt test/KOMO/timeopt/main.cpp)
target_link_libraries(test_timeopt rai)
add_executable(test_tutorial test/KOMO/tutorial/main.cpp)
target_link_libraries(test_tutorial rai)
add_executable(test_komo test/KOMO/komo/main.cpp)
target_link_libraries(test_komo rai)
add_executable(test_splined test/KOMO/splined/main.cpp)
target_link_libraries(test_splined rai)
add_executable(test_factored test/KOMO/factored/main.cpp)
target_link_libraries(test_factored rai)
add_executable(test_skeleton test/KOMO/skeleton/main.cpp)
target_link_libraries(test_skeleton rai)
add_executable(test_geo test/Geo/geo/main.cpp)
target_link_libraries(test_geo rai)
add_executable(test_convex test/Geo/convex/main.cpp)
target_link_libraries(test_convex rai qhull)
add_executable(test_PairCollisions test/Geo/PairCollisions/main.cpp)
target_link_libraries(test_PairCollisions rai)
add_executable(test_mesh test/Geo/mesh/main.cpp)
target_link_libraries(test_mesh rai)
add_executable(test_signedDistanceFunctions test/Geo/signedDistanceFunctions/main.cpp)
target_link_libraries(test_signedDistanceFunctions rai)
add_executable(test_collisionEngines test/Kin/collisionEngines/main.cpp)
target_link_libraries(test_collisionEngines rai)
add_executable(test_CameraView test/Kin/CameraView/main.cpp)
target_link_libraries(test_CameraView rai)
add_executable(test_pairCollision_jacobians test/Kin/pairCollision_jacobians/main.cpp)
target_link_libraries(test_pairCollision_jacobians rai)
add_executable(test_features test/Kin/features/main.cpp)
target_link_libraries(test_features rai)
add_executable(test_bullet test/Kin/bullet/main.cpp)
target_link_libraries(test_bullet rai)
add_executable(test_simulation test/Kin/simulation/main.cpp)
target_link_libraries(test_simulation rai)
add_executable(test_SSB_distance test/Kin/SSB_distance/main.cpp)
target_link_libraries(test_SSB_distance rai)
add_executable(test_dynamics test/Kin/dynamics/main.cpp)
target_link_libraries(test_dynamics rai)
add_executable(test_kin test/Kin/kin/main.cpp)
target_link_libraries(test_kin rai)
#add_executable(test_opengl_threaded test/Gui/opengl_threaded/main.cpp)
#target_link_libraries(test_opengl_threaded rai)
#add_executable(test_opengl test/Gui/opengl/main.cpp)
#target_link_libraries(test_opengl rai)
#add_executable(test_projections test/Gui/projections/main.cpp)
#target_link_libraries(test_projections rai)
#add_executable(test_opengl_windowless test/Gui/opengl_windowless/main.cpp)
#target_link_libraries(test_opengl_windowless rai)
#add_executable(test_opengl_mini test/Gui/opengl_mini/main.cpp)
#target_link_libraries(test_opengl_mini rai)
add_executable(test_RenderData test/Gui/RenderData/main.cpp)
target_link_libraries(test_RenderData rai)
add_executable(test_pcl test/Perception/pcl/main.cpp)
target_link_libraries(test_pcl rai)
add_executable(test_util test/Core/util/main.cpp)
target_link_libraries(test_util rai)
add_executable(test_array test/Core/array/main.cpp)
target_link_libraries(test_array rai)
add_executable(test_thread test/Core/thread/main.cpp)
target_link_libraries(test_thread rai)
add_executable(test_graph test/Core/graph/main.cpp)
target_link_libraries(test_graph rai)
add_executable(test_hdf5 test/Core/hdf5/main.cpp)
target_link_libraries(test_hdf5 rai)
add_executable(test_yaml test/Core/yaml/main.cpp)
target_link_libraries(test_yaml rai yaml-cpp)
add_executable(test_ML-regression test/Algo/ML-regression/main.cpp)
target_link_libraries(test_ML-regression rai)
add_executable(test_ann test/Algo/ann/main.cpp)
target_link_libraries(test_ann rai)
add_executable(test_rrt test/Algo/rrt/main.cpp)
target_link_libraries(test_rrt rai)
add_executable(test_spanningTree test/Algo/spanningTree/main.cpp)
target_link_libraries(test_spanningTree rai)
add_executable(test_splines test/Algo/splines/main.cpp)
target_link_libraries(test_splines rai)
add_executable(test_pickAndPlace test/LGP/pickAndPlace/main.cpp)
target_link_libraries(test_pickAndPlace rai)
add_executable(test_newLGP test/LGP/newLGP/main.cpp)
target_link_libraries(test_newLGP rai)
add_executable(test_pddl test/Logic/pddl/main.cpp)
target_link_libraries(test_pddl rai)
add_executable(test_fol test/Logic/fol/main.cpp)
target_link_libraries(test_fol rai)
add_executable(test_optim test/Optim/optim/main.cpp)
target_link_libraries(test_optim rai)
add_executable(test_nlopt test/Optim/nlopt/main.cpp)
target_link_libraries(test_nlopt rai)
add_executable(test_constrained test/Optim/constrained/main.cpp)
target_link_libraries(test_constrained rai)
add_executable(test_benchmarks test/Optim/benchmarks/main.cpp)
target_link_libraries(test_benchmarks rai)
add_executable(test_ipopt test/Optim/ipopt/main.cpp)
target_link_libraries(test_ipopt rai)
endif()

################################################################################

message(STATUS "[rai] installing headers and lib in " ${CMAKE_INSTALL_PREFIX}/include/rai|lib)

install(
  DIRECTORY src/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rai
  FILES_MATCHING
  PATTERN "*.h" PATTERN "*.ipp"
  PATTERN "*retired*" EXCLUDE
  )

install(
  TARGETS rai rai_static
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(
  TARGETS kinEdit meshTool
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

