/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "color.h"

#include <iostream>
#include <math.h>

void rai::Color::setIndex(unsigned i) {
  if(!i) setRgb(0., 0., 0.); else setHsv(((i-1)*63)%360, 255, 255);
}

void rai::Color::setRgbByte(byte red, byte green, byte blue) {
  r=red/255.f; g=green/255.f; b=blue/255.f;
}

void rai::Color::setHsv(int hue, byte sat, byte val) {
  float h=hue/60.f, s=sat/255.f, v=val/255.f;
  h=(float)fmod(h, 6.f);
  r=g=b=0.;
  if(h<=1.)        { r=v; g=v*h; }
  if(h>1. && h<=2.) { g=v; r=v*(2.f-h); }
  if(h>2. && h<=3.) { g=v; b=v*(h-2.f); }
  if(h>3. && h<=4.) { b=v; g=v*(4.f-h); }
  if(h>4. && h<=5.) { b=v; r=v*(h-4.f); }
  if(h>5. && h<=6.) { r=v; b=v*(6.f-h); }
  r=s*r+(1.f-s)*v;
  g=s*g+(1.f-s)*v;
  b=s*b+(1.f-s)*v;
}

void rai::Color::setTemp(float temp) {
  Color hot(1., 0., 0.), middle(1., 1., 0.), cold(0., 0., 1.);
  if(temp>1.) temp=1.;
  if(temp<0.) temp=0.;
  if(temp>.5) { temp=2.f*temp-1.f; *this=temp*hot + (1.-temp)*middle; } else { temp=2.f*temp; *this=temp*middle + (1.f-temp)*cold; }
}

void rai::Color::setTemp2(float temp) {
  Color r(1., 0., 0.), y(1., 1., 0.), zero(.5, .5, .5), g(0., 1., 0.), b(0., 0., 1.);
  if(temp>1.) temp=1.;
  if(temp<-1.) temp=-1.;
  if(temp>.5) {  temp=2.*temp-1.; *this=temp*r + (1.-temp)*y; return; }
  if(temp>.0) {  temp=2.*temp;    *this=temp*y + (1.-temp)*zero; return; }
  if(temp>-.5) { temp=-2.*temp;   *this=temp*g + (1.-temp)*zero; return; }
  { temp=-2.*temp-1.; *this=temp*b + (1.-temp)*g; return; }
}

void rai::Color::setGray(float gray) { if(gray<0) gray=0.; if(gray>1) gray=1.; r=g=b=gray; }

void rai::Color::getRgb(byte& R, byte& G, byte& B) const {
  R=(byte)(255.*r); G=(byte)(255.*g); B=(byte)(255.*b);
}

float rai::Color::getGray() const { return (r+g+b)/3.; }

arr rai::Color::getArr() const { return arr{r, g, b}; }

void rai::Color::whiten(float f) {
  if(f>1.) f=1.; else if(f<0.) f=0.;
  r+=f*(1.-r); g+=f*(1.-g); b+=f*(1.-b);
}

void rai::Color::blacken(float f) {
  if(f>1.) f=1.; else if(f<0.) f=0.;
  r-=f*r; g-=f*g; b-=f*b;
}

void rai::Color::mix(rai::Color& A, rai::Color& B, float f) {
  if(f>1.) f=1.; else if(f<0.) f=0.;
  r=f*A.r+(1.-f)*B.r;
  g=f*A.g+(1.-f)*B.g;
  b=f*A.b+(1.-f)*B.b;
}

void rai::Color::mixAdd(rai::Color& A, rai::Color& B, float f) {
  if(f>1.) f=1.; else if(f<0.) f=0.;
  r=1.-f*(1.-A.r)+(1.-f)*(1.-B.r);
  g=1.-f*(1.-A.g)+(1.-f)*(1.-B.g);
  b=1.-f*(1.-A.b)+(1.-f)*(1.-B.b);
}

void rai::Color::mixSub(rai::Color& A, rai::Color& B, float f) {
  if(f>1.) f=1.; else if(f<0.) f=0.;
  r=1.-::pow(1.f-A.r, f)*::pow(1.f-B.r, 1.f-f);
  g=1.-::pow(1.f-A.g, f)*::pow(1.f-B.g, 1.f-f);
  b=1.-::pow(1.f-A.b, f)*::pow(1.f-B.b, 1.f-f);
}

void rai::Color::setMin(rai::Color& A, rai::Color& B) {
  r=A.r<B.r?A.r:B.r;
  g=A.g<B.g?A.g:B.g;
  b=A.b<B.b?A.b:B.b;
}

void rai::Color::write(std::ostream& os) const { os <<"(" <<r <<":" <<g <<":" <<b <<")"; }

void rai::Color::read(std::istream& is) { is >>PARSE("(") >>r >>PARSE(":") >>g >>PARSE(":") >>b >>PARSE(")"); }
