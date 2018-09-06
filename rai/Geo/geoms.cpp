/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "geoms.h"

#include <Core/graph.h>
#include <Gui/opengl.h>

Singleton<rai::GeomStore> _GeomStore;

template<> const char* rai::Enum<rai::ShapeType>::names []= {
  "box", "sphere", "capsule", "mesh", "cylinder", "marker", "SSBox", "pointCloud", "ssCvx", "ssBox", NULL
};

rai::Geom::Geom(rai::GeomStore &_store) : store(_store), type(ST_none) {
  ID=store.geoms.N;
  store.geoms.append(this);
  size = {1.,1.,1.,.1};
  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;
  
}

rai::Geom::~Geom() {
  store.geoms(ID) = NULL;
}

void rai::Geom::read(const Graph &ats) {
  double d;
  arr x;
  rai::String str;
  rai::FileToken fil;
  
  ats.get(size, "size");
  if(ats.get(mesh.C, "color")) {
    CHECK(mesh.C.N==3 || mesh.C.N==4,"");
    //    if(x.N==3){ memmove(color, x.p, 3*sizeof(double)); color[3]=1.; }
    //    else memmove(color, x.p, 4*sizeof(double));
  }
  if(ats.get(d, "shape"))        { type=(ShapeType)(int)d;}
  else if(ats.get(str, "shape")) { str>> type; }
  else if(ats.get(d, "type"))    { type=(ShapeType)(int)d;}
  else if(ats.get(str, "type"))  { str>> type; }
  if(ats.get(str, "mesh"))     { mesh.read(FILE(str), str.getLastN(3).p, str); }
  else if(ats.get(fil, "mesh"))     { mesh.read(fil.getIs(), fil.name.getLastN(3).p, fil.name); }
  if(ats.get(d, "meshscale"))  { mesh.scale(d); }
  if(ats.get(x, "meshscale"))  { mesh.scale(x(0), x(1), x(2)); }
  
  if(mesh.V.N && type==ST_none) type=ST_mesh;
  
  createMeshes();
  
  //colored box?
  if(ats["coloredBox"]) {
    CHECK_EQ(mesh.V.d0, 8, "I need a box");
    arr col=mesh.C;
    mesh.C.resize(mesh.T.d0, 3);
    for(uint i=0; i<mesh.C.d0; i++) {
      if(i==2 || i==3) mesh.C[i] = col; //arr(color, 3);
      else if(i>=4 && i<=7) mesh.C[i] = 1.;
      else mesh.C[i] = .5;
    }
  }
}

void rai::Geom::createMeshes() {
  //create mesh for basic shapes
  switch(type) {
    case rai::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case rai::ST_box:
      mesh.setBox();
      mesh.scale(size(0), size(1), size(2));
      break;
    case rai::ST_sphere:
      sscCore.V = arr(1,3, {0.,0.,0.});
      mesh.setSSCvx(sscCore, size(3));
      //      mesh.setSphere();
      //      mesh.scale(size(3), size(3), size(3));
      break;
    case rai::ST_cylinder:
      CHECK(size(3)>1e-10,"");
      mesh.setCylinder(size(3), size(2));
      break;
    case rai::ST_capsule:
      CHECK(size(3)>1e-10,"");
      sscCore.V = arr(2,3, {0.,0.,-.5*size(2), 0.,0.,.5*size(2)});
      mesh.setSSCvx(sscCore, size(3));
      //      mesh.setCappedCylinder(size(3), size(2));
      //      mesh.setSSBox(2.*size(3), 2.*size(3), size(2)+2.*size(3), size(3));
      break;
    case rai::ST_retired_SSBox:
      HALT("deprecated?");
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      break;
    case rai::ST_marker:
      break;
    case rai::ST_mesh:
    case rai::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded");
      size(3) = 0.;
//    sscCore = mesh;
//    sscCore.makeConvexHull();
      break;
    case rai::ST_ssCvx:
      CHECK(size(3)>1e-10,"");
      if(!sscCore.V.N) {
        CHECK(mesh.V.N, "mesh or sscCore needs to be loaded");
        sscCore=mesh;
      }
      mesh.setSSCvx(sscCore, size(3));
      break;
    case rai::ST_ssBox:
      if(size(3)<1e-10) {
        sscCore.setBox();
        sscCore.scale(size(0), size(1), size(2));
        mesh = sscCore;
        break;
      }
      CHECK(size.N==4 && size(3)>1e-10,"");
      sscCore.setBox();
      sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      //      mesh.setSSCvx(sscCore, size(3));
      break;
    default: NIY;
  }
}

void rai::Geom::glDraw(OpenGL &gl) {
  bool drawCores = false;
  bool drawMeshes = true;
  switch(type) {
    case rai::ST_none: LOG(-1) <<"Geom '" <<ID <<"' has no type";  break;
    case rai::ST_box:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawBox(size(0), size(1), size(2));
      break;
    case rai::ST_sphere:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawSphere(size(3));
      break;
    case rai::ST_cylinder:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawCylinder(size(3), size(2));
      break;
    case rai::ST_capsule:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawCappedCylinder(size(3), size(2));
      break;
    case rai::ST_retired_SSBox:
      HALT("deprecated??");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes) {
        if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
        mesh.glDraw(gl);
      } else NIY;
      break;
    case rai::ST_marker:
      //      if(frame.K.orsDrawMarkers){
      glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0), !gl.drawMode_idColor);
      //      }
      break;
    case rai::ST_mesh:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
    case rai::ST_ssCvx:
      CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
      if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
    case rai::ST_ssBox:
      if(!mesh.V.N || !sscCore.V.N) {
        sscCore.setBox();
        sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
        mesh.setSSCvx(sscCore, size(3));
      }
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
    case rai::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
      
    default: HALT("can't draw that geom yet");
  }
}

rai::GeomStore::~GeomStore() {
  for(Geom* g:geoms) if(g) delete g;
  for(Geom* g:geoms) CHECK(!g, "geom is not deleted");
  geoms.clear();
}

rai::Geom &rai::GeomStore::get(uint id) {
  Geom *g=geoms.elem(id);
  CHECK(g, "geom does not exist");
  return *g;
}
