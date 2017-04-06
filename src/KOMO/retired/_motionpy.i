// Read DOCSTRING to get an idea of motionpy!
%define DOCSTRING_MOTIONPY
"
This is a SWIG wrapper to be able to use the motion framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_MOTIONPY) motionpy

%feature("autodoc", "1");

%include "Core/array_typemaps.i"
%import "Core/_corepy.i"
%import "Ors/_orspy.i"
%import "Optim/_optimpy.i"

//===========================================================================
%module motionpy
%{
  #include "motion.h"
  #include "motionHeuristics.h"
  #include "rrt_planner.h"
  #include "taskMap_constrained.h"
  #include "taskMap_default.h"
  #include "taskMap_proxy.h"
  #include "pr2_heuristics.h"
  #include <Optim/optimization.h>
  #include <sstream>
%}

//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

%inline %{
  typedef MT::Array<int> intA;
  typedef MT::Array<uint> uintA;
  typedef MT::Array<arr*> arrL;
%}

%include "motion.h"

%ignore setPlaceGoals;
%ignore setHomingGoals;
%include "motionHeuristics.h"

%include "rrt_planner.h"
%include "taskMap_constrained.h"
%include "taskMap_default.h"
%include "taskMap_proxy.h"
%include "pr2_heuristics.h"
