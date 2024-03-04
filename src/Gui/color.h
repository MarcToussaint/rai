/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

namespace rai {
/// simple float[3] color class
class Color {
 public:
  float
  r, ///< red
  g, ///< green
  b; ///< blue

  /// ...
  friend inline Color operator+(const Color& c1, const Color& c2) {
    return Color(c1.r+c2.r, c1.g+c2.g, c1.b+c2.b);
  }

  /// ...
  friend inline Color operator*(float f, const Color& c2) {
    return Color(f*c2.r, f*c2.g, f*c2.b);
  }

 public:
  /// initializes to white
  Color() { setGray(1.); }

  /// initialize with RGB
  Color(float red, float green, float blue) { setRgb(red, green, blue); }

  /// copy operator
  Color& operator=(const Color& c) { r=c.r; g=c.g; b=c.b; return *this; }

  /// return true iff black
  bool operator!() { if(r==0. && g==0. && b==0.) return true; return false; }

  /// float-pointer access
  operator const float* () const { return (float*)this; }

  /// chooses color from a color table (distributed around the hue-scale)
  void setIndex(unsigned i);

  /// set RGA values
  void setRgb(float red, float green, float blue) { r=red; g=green; b=blue; }

  /// set RGA values from bytes in [0, 255]
  void setRgbByte(byte red, byte green, byte blue);

  /// set color by hue [0, 360], saturation [0, 255], and value [0, 255]
  void setHsv(int hue, byte sat, byte val);

  /// set color by temperature: hot=red, middle=yellow, cold=blue
  void setTemp(float temp);

  /// set color by temperature: red - yellow - gray(middle) - green - blue
  void setTemp2(float temp);

  /// set gray value [0, 1]
  void setGray(float gray);

  /// get RGB values as bytes [0, 255]
  void getRgb(byte& R, byte& G, byte& B) const;

  /// get the gray value (average of RGB)
  float getGray() const;

  arr getArr() const;

  /// mix with white
  void whiten(float f);

  /// mix with black
  void blacken(float f);

  /// plain color mixing
  void mix(Color& A, Color& B, float f=.5);

  /// additive color mixing
  void mixAdd(Color& A, Color& B, float f=.5);

  /// subtractive color mixing
  void mixSub(Color& A, Color& B, float f=.5);

  /// take smaller of the two values
  void setMin(Color& A, Color& B);

  /// prototype for operator<<
  void write(std::ostream& os) const;

  /// prototype for operator>>
  void read(std::istream& is);
};
}
stdPipes(rai::Color)
