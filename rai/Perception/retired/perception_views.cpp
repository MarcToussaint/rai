/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "perception.h"
#include "../Gui/opengl.h"

#if 0

//===========================================================================

#define REG_VIEW(VAR) REGISTER_VIEW(VAR##_View, VAR)

REG_VIEW(Image)
//REG_VIEW(FloatImage)
REG_VIEW(HoughLines)
REG_VIEW(Patching)
REG_VIEW(SURFfeatures)
REG_VIEW(PerceptionOutput)

//===========================================================================

void Image_View::glInit() {
  ((Image*)object)->get_img(copy, nullptr);
  gl->img = &copy;
}

void Image_View::glDraw() {
  ((Image*)object)->get_img(copy, nullptr);
  gl->img = &copy;
}

//===========================================================================

void HoughLines_View::glInit() {
  ((HoughLines*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

void HoughLines_View::glDraw() {
  ((HoughLines*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

//===========================================================================

void Patching_View::glInit() {
  ((Patching*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

void Patching_View::glDraw() {
  ((Patching*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

//===========================================================================

void SURFfeatures_View::glInit() {
  ((SURFfeatures*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

void SURFfeatures_View::glDraw() {
  ((SURFfeatures*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

//===========================================================================

void PerceptionOutput_View::glInit() {
  ((PerceptionOutput*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

void PerceptionOutput_View::glDraw() {
  ((PerceptionOutput*)object)->get_display(copy, nullptr);
  gl->img = &copy;
}

#endif
