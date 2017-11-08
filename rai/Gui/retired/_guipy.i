// Read DOCSTRING to get an idea of guipy!
%define DOCSTRING_GUIPY
"
This is a simple SWIG wrapper to be able to use the gui framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_GUIPY) guipy

%feature("autodoc", "1");

%include "Core/array_typemaps.i"
%import "Core/_corepy.i"

//===========================================================================

%module motionpy
%{
  #include "mesh.h"
  #include "opengl.h"
  #include <Optim/optimization.h>
  #include <sstream>
%}

%ignore glDrawRobotArm;
%ignore glClickUI;
%ignore ors::Camera::setOffset;

%include "mesh.h"
%include "opengl.h"
