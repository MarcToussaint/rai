#include "geoms.h"

#include <Core/graph.h>
#include <Gui/opengl.h>

Singleton<mlr::GeomStore> _GeomStore;

template<> const char* mlr::Enum<mlr::ShapeType>::names []={
  "ST_box", "ST_sphere", "ST_capsule", "ST_mesh", "ST_cylinder", "ST_marker", "ST_SSBox", "ST_pointCloud", "ST_ssCvx", "ST_ssBox", NULL
};

mlr::Geom::Geom(mlr::GeomStore &_store) : store(_store), type(ST_none) {
    ID=store.geoms.N;
    store.geoms.append(this);
    size = {1.,1.,1.,.1};
    mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;

}

mlr::Geom::~Geom(){
    store.geoms(ID) = NULL;
}

void mlr::Geom::read(const Graph &ats){
  double d;
  arr x;
  mlr::String str;
  mlr::FileToken fil;

  ats.get(size, "size");
  if(ats.get(mesh.C, "color")){
      CHECK(mesh.C.N==3 || mesh.C.N==4,"");
      //    if(x.N==3){ memmove(color, x.p, 3*sizeof(double)); color[3]=1.; }
      //    else memmove(color, x.p, 4*sizeof(double));
  }
  if(ats.get(d, "shape"))        { type=(ShapeType)(int)d;}
  else if(ats.get(str, "shape")) { str>> type; }
  else if(ats.get(d, "type"))    { type=(ShapeType)(int)d;}
  else if(ats.get(str, "type"))  { str>> type; }
  if(ats.get(fil, "mesh"))     { mesh.read(fil.getIs(), fil.name.getLastN(3).p, fil.name); }
  if(ats.get(d, "meshscale"))  { mesh.scale(d); }
  if(ats.get(x, "meshscale"))  { mesh.scale(x(0), x(1), x(2)); }

  //create mesh for basic shapes
  switch(type) {
  case mlr::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
  case mlr::ST_box:
      mesh.setBox();
      mesh.scale(size(0), size(1), size(2));
      break;
  case mlr::ST_sphere:
      mesh.setSphere();
      mesh.scale(size(3), size(3), size(3));
      break;
  case mlr::ST_cylinder:
      CHECK(size(3)>1e-10,"");
      mesh.setCylinder(size(3), size(2));
      break;
  case mlr::ST_capsule:
      CHECK(size(3)>1e-10,"");
      //      mesh.setCappedCylinder(size(3), size(2));
//      sscCore.setBox();
//      sscCore.scale(0., 0., size(2));
//      mesh.setSSCvx(sscCore, size(3));
      mesh.setSSBox(2.*size(3), 2.*size(3), size(2)+2.*size(3), size(3));
      break;
  case mlr::ST_retired_SSBox:
      HALT("deprecated?");
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      break;
  case mlr::ST_marker:
      break;
  case mlr::ST_mesh:
  case mlr::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore = mesh;
      sscCore.makeConvexHull();
      break;
  case mlr::ST_ssCvx:
      CHECK(size(3)>1e-10,"");
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore=mesh;
      mesh.setSSCvx(sscCore, size(3));
      break;
  case mlr::ST_ssBox:
      CHECK(size.N==4 && size(3)>1e-10,"");
      sscCore.setBox();
      sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      //      mesh.setSSCvx(sscCore, size(3));
      break;
  default: NIY;
  }

  //colored box?
  if(ats["coloredBox"]){
    CHECK_EQ(mesh.V.d0, 8, "I need a box");
    arr col=mesh.C;
    mesh.C.resize(mesh.T.d0, 3);
    for(uint i=0;i<mesh.C.d0;i++){
      if(i==2 || i==3) mesh.C[i] = col; //arr(color, 3);
      else if(i>=4 && i<=7) mesh.C[i] = 1.;
      else mesh.C[i] = .5;
    }
  }
}

void mlr::Geom::glDraw(OpenGL &gl){
  bool drawCores = false;
  bool drawMeshes = true;
  switch(type) {
  case mlr::ST_none: LOG(-1) <<"Geom '" <<ID <<"' has no type";  break;
  case mlr::ST_box:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawBox(size(0), size(1), size(2));
      break;
  case mlr::ST_sphere:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawSphere(size(3));
      break;
  case mlr::ST_cylinder:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawCylinder(size(3), size(2));
      break;
  case mlr::ST_capsule:
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes && mesh.V.N) mesh.glDraw(gl);
      else glDrawCappedCylinder(size(3), size(2));
      break;
  case mlr::ST_retired_SSBox:
      HALT("deprecated??");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else if(drawMeshes){
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
      }else NIY;
      break;
  case mlr::ST_marker:
//      if(frame.K.orsDrawMarkers){
      glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
//      }
      break;
  case mlr::ST_mesh:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
  case mlr::ST_ssCvx:
      CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
      if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
  case mlr::ST_ssBox:
      if(!mesh.V.N || !sscCore.V.N){
          sscCore.setBox();
          sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
          mesh.setSSCvx(sscCore, size(3));
      }
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;
  case mlr::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
      if(drawCores && sscCore.V.N) sscCore.glDraw(gl);
      else mesh.glDraw(gl);
      break;

  default: HALT("can't draw that geom yet");
  }
}

mlr::GeomStore::~GeomStore(){
    for(Geom* g:geoms) if(g) delete g;
    for(Geom* g:geoms) CHECK(!g, "geom is not deleted");
    geoms.clear();
}

mlr::Geom &mlr::GeomStore::get(uint id){
    Geom *g=geoms.elem(id);
    CHECK(g, "geom does not exist");
    return *g;
}
