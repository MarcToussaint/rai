/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "plot.h"
#include "../Core/array.ipp"
#include "../Geo/geo.h"
#include "../Geo/mesh.h"
#include "../Gui/opengl.h"
#include "../Gui/color.h"

void drawGnuplot(rai::sPlotModule& data);

//===========================================================================
//
// global structures
//

Singleton<rai::PlotModule> plot;

namespace rai {

struct sPlotModule {
  rai::Array<arr> array;
  rai::Array<byteA> images;
  rai::Array<arr> points;
  rai::Array<arr> lines;
  rai::Array<rai::String> legend;
  rai::Array<rai::Vector> planes;
  rai::Mesh mesh;
};

}

rai::PlotModule::PlotModule() {
  self = make_unique<sPlotModule>();
  mode=gnupl;
  gl=0;
  light=false;
  grid=false;
  colors=true;
  drawBox=false;
  drawDots=false;
  perspective=false;
  thickLines=0;
}

rai::PlotModule::~PlotModule() {
#ifdef RAI_GL
  if(gl) { delete gl; gl=nullptr; }
#endif
}

//===========================================================================
//
// C interface implementations
//

void rai::PlotModule::update(bool wait, const char* txt) {
  if(!rai::getInteractivity()) {
    wait=false;
  }
  switch(mode) {
    case gnupl:
      drawGnuplot(*self);
      if(wait) rai::wait();
      break;
#ifdef RAI_GL
    case opengl:
      CHECK(gl, "");
      if(txt) gl->text = txt;
      if(wait) gl->watch();
      else gl->update();
      break;
#else
    case opengl:
      HALT("can't plot on OpenGL without RAI_GL flag");
      break;
#endif
    case xfig:
      NIY;
      break;
  }
}

void rai::PlotModule::Close() {
#ifdef RAI_GL
  if(mode==opengl) {
    if(gl) { delete gl; gl=NULL; }
  }
#endif
}

void rai::PlotModule::Clear() {
  self->array.clear();
  self->points.clear();
  self->lines.clear();
#ifdef RAI_GL
  self->planes.clear();
#endif
}

void rai::PlotModule::Gnuplot() { mode=gnupl; }

void rai::PlotModule::Opengl(bool perspective, double xl, double xh, double yl, double yh, double zl, double zh) {
  mode=opengl;
  perspective=perspective;
  if(!gl) {
    gl=new OpenGL("PlotModule", 400, 400);
    gl->add(*this);
    gl->setClearColors(1., 1., 1., 1.);
  }
  gl->camera.setPosition(.5*(xh+xl), .5*(yh+yl), 5.);
  gl->camera.focus(.5*(xh+xl), .5*(yh+yl), .0);
  gl->camera.setWHRatio((xh-xl)/(yh-yl));
  if(perspective) {
    gl->camera.setHeightAngle(45.);
  } else {
    gl->camera.setHeightAbs(1.2*(yh-yl));
  }
  gl->update();
}

void rai::PlotModule::Image(const byteA& x) { self->images.append(x); }

void rai::PlotModule::Function(const arr& f, double x0, double x1) {
  arr X;
  uint i, j;
  if(f.nd==2) {
    if(x0 || x1) {
      X.resize(f.d0, f.d1+1);
      for(i=0; i<f.d0; i++) { X(i, 0)=x0+(x1-x0)*i/(f.N-1); for(j=1; j<X.d1; j++) X(i, j)=f(i, j-1); }
    } else {
      X=f;
    }
  }
  if(f.nd==1) {
    if(x0 || x1) {
      X.resize(f.N, 2);
      for(i=0; i<f.d0; i++) { X(i, 0)=x0+(x1-x0)*i/(f.N-1); X(i, 1)=f(i); }
    } else {
      X=f;
      X.reshape(X.N, 1);
    }
  }
  self->lines.append(X);
}

void rai::PlotModule::Functions(const arr& F, double x0, double x1) {
  CHECK_EQ(F.nd, 2, "");
  arr tF;
  transpose(tF, F);
  for(uint j=0; j<tF.d0; j++) Function(tF[j], x0, x1);
}

void rai::PlotModule::FunctionPoints(const arr& x, const arr& f) {
  CHECK_EQ(x.d0, f.d0, "Domain and image of function have different size!")
  arr X(x.d0, x.d1+1);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
  }
  self->points.append(X);
}

void rai::PlotModule::Function(const arr& x, const arr& f) {
  CHECK_EQ(x.d0, f.d0, "Domain and image of function have different size!")
  CHECK_EQ(f.nd, 1, "Function image should be 1D")
  CHECK(x.d[x.nd-1]<3, "Can handle up to 2D domains")
  arr X(x.d0, x.d1+1);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
  }
  self->lines.append(X);
}

void rai::PlotModule::FunctionPrecision(const arr& x, const arr& f, const arr& h, const arr& l) {
  CHECK_EQ(x.d0, f.d0, "Domain and image of function have different size!")
  CHECK(f.nd==1&&h.nd==1&&l.nd==1, "Function image should be 1D")
  CHECK(x.d[x.nd-1]<2, "Can handle up to 1D domains")
  arr X(x.d0, x.d1+3);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
    X(i, j+1)=l(i);
    X(i, j+2)=h(i);
  }
  self->lines.append(X);
}

void rai::PlotModule::Surface(const arr& X) {
  self->array.append(X);
#ifdef RAI_GL
  self->mesh.clear();
  self->mesh.V.resize(X.N, 3);
  self->mesh.C.resize(X.N, 3);
  self->mesh.setGrid(X.d1, X.d0);
  //self->mesh.gridToStrips(X.d1, X.d0);
#endif
}

void rai::PlotModule::Point(double x, double y, double z) {
  arr p(1, 3); p(0, 0)=x; p(0, 1)=y; p(0, 2)=z;
  self->points.append(p);
}

void rai::PlotModule::Point(const arr& x) {
  arr p; p.referTo(x); p.reshape(1, p.N);
  self->points.append(p);
}

void rai::PlotModule::Points(const arr& X) {
  self->points.append(X);
}

void rai::PlotModule::ClearPoints() {
  self->points.clear();
}

void rai::PlotModule::Line(const arr& X, bool closed) {
  arr& app = self->lines.append(X);
  if(closed && app.d0) {
    arr x;
    x = app[0];
    app.append(x);
  }
}

void rai::PlotModule::Points(const arr& X, const arr& Y) {
  arr P;
  uint i, j;
  if(X.nd==2) {
    P.resize(X.d0, X.d1+1);
    for(i=0; i<P.d0; i++) {
      for(j=0; j<X.d1; j++) P(i, j)=X(i, j);
      P(i, j)=Y(i);
    }
  } else {
    P.resize(X.d0, 2);
    for(i=0; i<P.d0; i++) { P(i, 0)=X(i); P(i, 1)=Y(i); }
  }
  self->points.append(P);
}

void rai::PlotModule::Covariance(const arr& mean, const arr& cov) {
  if(mean.nd==2) {
    for(uint k=0; k<mean.d0; k++) Covariance(mean[k], cov[k]);
    return;
  }
  uint d=mean.N;
  if(d==1) {
    arr d(20, 2);
    uint i;
    for(i=0; i<d.d0; i++) { //standard Gaussian
      d(i, 0)=5. * ((i+.5)/d.d0 - .5);
      d(i, 1)=1./::sqrt(RAI_2PI)*::exp(-.5*d(i, 0)*d(i, 0));
    }
    for(i=0; i<d.d0; i++) { //standard Gaussian
      d(i, 0) = ::sqrt(cov(0, 0)) * d(i, 0) + mean(0);
      d(i, 1) *= 1./::sqrt(cov(0, 0));
    }
    Function(d);
  }
  if(d==2) {
    arr d(101, 2), Cov, U, V, w;
    double phi;
    uint i;
    if(cov.d0>2) { Cov=cov.sub(0, 1, 0, 1); } else { Cov.referTo(cov); }
    for(i=0; i<d.d0; i++) { //standard circle
      phi=RAI_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { d[i]()*=w; d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(1); }

    self->lines.append(d);
  }
  if(d==3) {
#if 1
    arr d(303, 3), Cov, U, V, w;
    double phi;
    uint i;
    for(i=0; i<101; i++) { //standard sphere
      phi=RAI_2PI*((double)i)/(101-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi); d(i, 2)=0.;
    }
    for(i=0; i<101; i++) {
      phi=RAI_2PI*((double)i)/(101-1);
      d(101+i, 0)=cos(phi); d(101+i, 1)=0.; d(101+i, 2)=sin(phi);
    }
    for(i=0; i<101; i++) {
      phi=RAI_2PI*((double)i)/(101-1);
      d(202+i, 0)=0.; d(202+i, 1)=cos(phi); d(202+i, 2)=sin(phi);
    }
    CHECK_EQ(cov.d0, 3, "");
    //lapack_cholesky(V, cov);
    svd(U, w, V, cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { d[i]()*=w; d[i]=V*d[i]; d[i]()+=mean; }
    d.reshape(3, 101, 3);
    self->lines.append(d[0]);
    self->lines.append(d[1]);
    self->lines.append(d[2]);
#else
    arr d(101, 2), dd(101, 3), Cov, U, V, w;
    double phi;
    uint i;
    //x-y
    Cov=cov.sub(0, 1, 0, 1);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=RAI_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(1); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=d(i, 0); dd(i, 1)=d(i, 1); dd(i, 2)=mean(2); }
    self->lines.append(dd);
    //y-z
    Cov=cov.sub(1, 2, 1, 2);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=RAI_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(1); d(i, 1)+=mean(2); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=mean(0); dd(i, 1)=d(i, 0); dd(i, 2)=d(i, 1); }
    self->lines.append(dd);
    //x-z
    Cov(0, 0)=cov(0, 0); Cov(1, 0)=cov(2, 0); Cov(0, 1)=cov(0, 2); Cov(1, 1)=cov(2, 2);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=RAI_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(2); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=d(i, 0); dd(i, 1)=mean(1); dd(i, 2)=d(i, 1); }
    self->lines.append(dd);
#endif
  }
}

void rai::PlotModule::VectorField(const arr& X, const arr& dX) {
  CHECK(X.nd==2 && samedim(X, dX), "");
  uint i;
  arr l(2, X.d1);
  for(i=0; i<X.d0; i++) {
    l[0]() = X[i];
    l[1]() = X[i]+dX[i];
    self->lines.append(l);
  }
}

void rai::PlotModule::MatrixFlow(uintA& M, double len) {
  CHECK_EQ(M.nd, 2, "");
  uint i, j;
  arr X, dX;
  X.resize(M.d0, M.d1, 3);
  for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) {
      X(i, j, 0)=-1.+(2.*j+1.)/X.d1;
      X(i, j, 1)=-1.+(2.*i+1.)/X.d0;
      X(i, j, 2)=1.;
    }
  X.reshape(M.d0*M.d1, 3);
  dX.resize(M.d0*M.d1, 3);
  for(i=0; i<X.d0; i++) {
    dX[i]() = X[M.elem(i)]-X[i];
  }
  dX *= len;
  VectorField(X, dX);
  Points(X);
}

#ifdef RAI_gauss_h
void rai::PlotModule::Gaussians(const GaussianA& G) {
  for(uint k=0; k<G.N; k++) { G(k).makeC(); Covariance(G(k).c, G(k).C); }
}
void rai::PlotModule::Gaussians(const GaussianL& G) {
  for(uint k=0; k<G.N; k++) { G(k)->makeC(); Covariance(G(k)->c, G(k)->C); }
}
#endif

//===========================================================================
//
// OpenGL draw routine
//

void rai::PlotModule::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  sPlotModule& data=*self;
//  uint a, i, j;

  uint idx = 0;

  rai::Color c;

  double x=0., y=0., z=0.;

  //light?
  if(light) glStandardLight(nullptr, gl);

  if(drawBox) {
    glColor3f(.7, .7, .7);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, 1, -1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, -1, -1);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, 1);
    glVertex3f(-1, 1, 1);
    glVertex3f(1, 1, 1);
    glVertex3f(1, -1, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, -1, -1);
    glVertex3f(1, -1, 1);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, 1, 1);
    glVertex3f(-1, 1, -1);
    glVertex3f(-1, 1, 1);
    glEnd();
  }

  //draw images
  //for(byteA& i:data.images) {
  //}

  //draw arrays
  for(arr& a:data.array) {
    CHECK_LE(a.nd, 2, "can't display 3(or higher)-dim arrays");
    //1D function, just series of y
    if(a.nd==1 || (a.nd==2 && a.d1==1)) {
      c.setIndex(idx);
      glColor(c.r, c.g, c.b);

      for(uint i=1; i<a.N; i++) {
        glBegin(GL_LINES);
        glVertex3f(2.*(i-1)/(a.N-1)-1., a.elem(i-1), 0);
        glVertex3f(2.*(i)/(a.N-1)-1., a.elem(i), 0);
        glEnd();
      }
      glBegin(GL_LINE_LOOP);
      glColor3f(0., 0., 0.);
      glVertex3f(-1, -1, 0);
      glVertex3f(-1, 1, 0);
      glVertex3f(1, 1, 0);
      glVertex3f(1, -1, 0);
      glEnd();
    }
    //2D function x -> y
    if(a.nd==2 && a.d1==2) { //2D path
      c.setIndex(idx);
      glColor(c.r, c.g, c.b);
      glBegin(GL_LINE_STRIP);
      for(uint i=0; i<a.d0; i++) {
        glVertex3f(2.*i/(a.d0-1)-1., a.operator()(i, 0), a.operator()(i, 1));
      }
      glEnd();
    }
    if(a.nd==2 && a.d1>2) { //2D landscapes
      uint i, j, X=a.d1, Y=a.d0;
      c.setIndex(idx);
      if(!grid) { //as a mesh
        c.whiten(.5);
        CHECK_EQ(Y*X, data.mesh.V.d0, "you must recall display(data.array) when dimensions changed");
        for(j=0; j<Y; j++) for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y= 2.*(double)j/(Y-1.)-1.;
            z=a(j, i);
            c.setTemp2(z);
            data.mesh.V(j*X+i, 0)=x;    data.mesh.V(j*X+i, 1)=y;    data.mesh.V(j*X+i, 2)=z;
            data.mesh.C(j*X+i, 0)=c.r;  data.mesh.C(j*X+i, 1)=c.g;  data.mesh.C(j*X+i, 2)=c.b;
          }
        data.mesh.computeNormals();
        glDisable(GL_CULL_FACE);
        data.mesh.glDraw(NoOpenGL);
        glEnable(GL_CULL_FACE);
      } else { //as a grid
        c.blacken(.5);
        for(j=0; j<Y; j++) { //along the x-axis
          glBegin(GL_LINE_STRIP);
          for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=a(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
        for(i=0; i<X; i++) { //along the y-axis
          glBegin(GL_LINE_STRIP);
          for(j=0; j<Y; j++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=a(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
      }
    }
    idx++;
  }

  //draw points
  for(arr& p:data.points) {
    c.setIndex(idx);
    glColor(c.r, c.g, c.b);
    //glBegin(GL_LINES);
    if(drawDots) glBegin(GL_POINTS);
    if(p.nd==2) {
      for(uint j=0; j<p.d0; j++) {
        if(p.d1==1) { x=(double)j; y=p(j, 0); z=0.; }
        if(p.d1==2) { x=p(j, 0); y=p(j, 1); z=0.; }
        if(p.d1>=3) { x=p(j, 0); y=p(j, 1); z=p(j, 2); }
        if(!drawDots) {
          glPushMatrix();
          glTranslatef(x, y, z);
          glDrawDiamond(.01, .01, .01);
          glPopMatrix();
        } else {
          glVertex3d(x, y, z);
        }
      }
    } else {
      if(p.d0==1) { x=p(0); y=0.; z=0.; }
      if(p.d0==2) { x=p(0); y=p(1); z=0.; }
      if(p.d0>=3) { x=p(0); y=p(1); z=p(2); }
      if(!drawDots) {
        glPushMatrix();
        glTranslatef(x, y, z);
        glDrawDiamond(.02, .02, .02);
        glPopMatrix();
      } else {
        glVertex3d(x, y, z);
      }
    }
    if(drawDots) glEnd();
    idx++;
  }

  //draw lines
  for(arr& l:data.lines) {
    if(colors) c.setIndex(idx); else c.setIndex(0);
    glColor(c.r, c.g, c.b);

    if(thickLines) {
      glLineWidth(thickLines);
    }

    glBegin(GL_LINE_STRIP);
    for(uint j=0; j<l.d0; j++) {
      if(l.d1==1) glVertex3d((double)j, l(j, 0), 0.);
      if(l.d1==2) glVertex3d(l(j, 0), l(j, 1), 0.);
      if(l.d1>=3) glVertex3d(l(j, 0), l(j, 1), l(j, 2));
    }
    glEnd();
    idx++;
  }

  //draw planes
  for(uint i=0; i<data.planes.N; i+=4) {
    c.setIndex(i/4+1);
    glColor(c.r, c.g, c.b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    glVertex3f(data.planes(i).x, data.planes(i).y, data.planes(i).z);
    glVertex3f(data.planes(i+1).x, data.planes(i+1).y, data.planes(i+1).z);
    glVertex3f(data.planes(i+2).x, data.planes(i+2).y, data.planes(i+2).z);
    glVertex3f(data.planes(i+3).x, data.planes(i+3).y, data.planes(i+3).z);
    glEnd();
  }
#else
  NIY;
#endif
}

//===========================================================================
//
// gnuplot draw routine
//

#define PLOTEVERY(block, with)  gnuplotcmd \
      <<"'z.plotdata' every :::" <<(block) <<"::" <<(block) <<(with);

void drawGnuplot(rai::sPlotModule& data) {
  uint i;

  //openfiles
  rai::String gnuplotcmd;
  std::ofstream gnuplotdata;
  rai::open(gnuplotdata, "z.plotdata");
  uint block=0;

  // include custom definition file if exists
  FILE* incf = fopen("z.plotcmd.inc", "r");
  if(incf) { fclose(incf);  gnuplotcmd <<"load 'z.plotcmd.inc'\n";}

  //gnuplotcmd <<"set size square\n";
  //if(wait) gnuplotcmd <<"set title 'CLICK LEFT TO CONTINUE'\n";

  if(data.lines.N+data.points.N) gnuplotcmd <<"\nplot \\\n";

  //pipe data
  bool ior=rai::IOraw;
  rai::IOraw=true;
  //lines
  for(i=0; i<data.lines.N; i++) {
    data.lines(i).write(gnuplotdata, " ", "\n", "  ", false, false);
    gnuplotdata <<'\n' <<std::endl;
    if(block) gnuplotcmd <<", \\\n";
    if(data.lines(i).d1!=4) {
      PLOTEVERY(block, " with l notitle");
    } else { //with filled error curves
      PLOTEVERY(block,
                " using 1:2:3 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \\\n ");
      PLOTEVERY(block,
                " using 1:2:4 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \\\n ");
      PLOTEVERY(block, " using 1:2 with l lc rgb 'green' notitle");
    }
    block++;
  }

  //points
  for(i=0; i<data.points.N; i++) {
    data.points(i).write(gnuplotdata, " ", "\n", "  ", false, false);
    gnuplotdata <<'\n' <<std::endl;
    if(block) gnuplotcmd <<", \\\n";
    rai::String a=" with p pt 3";
    if(i<data.legend.N) a<< " title '" <<data.legend(i) <<"' ";
    PLOTEVERY(block, a);
    block++;
  }

  if(data.array.N) gnuplotcmd <<"\n\npause mouse\nset dgrid3d\n\nsplot \\\n";

  //surfaces
  for(i=0; i<data.array.N; i++) {
    uint j, k, X=data.array(i).d1, Y=data.array(i).d0;
    for(j=0; j<Y; j++) {
      for(k=0; k<X; k++) {
        gnuplotdata <<2.*(double)k/(X-1.)-1. <<' ' <<-2.*(double)j/(Y-1.)+1. <<' ' <<data.array(i)(j, k) <<std::endl;
      }
    }
    gnuplotdata <<std::endl;
    if(i && block) gnuplotcmd <<", \\\n";
    PLOTEVERY(block, " with l notitle");
    block++;
  }
  rai::IOraw=ior;
  gnuplotcmd <<endl;

  //close files
  gnuplotdata.close();

  //call gnuplot
  gnuplot(gnuplotcmd, false, false, "z.pdf");
}

/*
double lo[3], hi[3];
sPlotModule(){
lo[0]=lo[1]=lo[2]= 0.;
hi[0]=hi[1]=hi[2]= 1.;
}
void setRange(double xl, double xh, double yl=-1., double yh=1., double zl=-1., double zh=1.){
lo[0]=xl; hi[0]=xh;
lo[1]=yl; hi[1]=yh;
lo[2]=zl; hi[2]=zh;
}

 void transBackPoint(double &x, double &y){
 x=(hi[0]-lo[0])*(x+1.)/2. + lo[0];
 y=(hi[1]-lo[1])*(y+1.)/2. + lo[1];
 }
*/
