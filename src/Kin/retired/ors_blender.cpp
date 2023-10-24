/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
//#include "array.cpp"

//#include <fstream>
//<iostream
struct VertGroup {
  String name;
  uintA verts;
};

//template class rai::Array<VertGroup>;
//template class rai::Array<String>;

void readBlender(const char* filename, rai::Mesh& mesh, rai::Configuration& bl) {
  ifstream is(filename, std::ios::binary);
  CHECK(is.good(), "couldn't open file " <<filename);

  arr vertices, normals, frames, tailsHeads;
  uintA faces;
  rai::Array<VertGroup> G;
  uintA graph;
  rai::Array<rai::String> names;

  String tag, name;
  char c;
  uint i, j;
  arr x0, x1, x2, x3;

  String::readSkipSymbols="\"\n\r\t ";
  String::readEatStopSymbol = true;

  for(;;) {
    CHECK(is.good(), "error in scanning the file (previous tag = `" <<tag <<"'");
    String::readStopSymbols="\n\r\t ";  //space terminates tags
    is >>tag;
    String::readStopSymbols="\n\r\t\"";       //quotes terminate names
    if(tag=="vertices") {   vertices.readTagged(is, 0);   continue;  }
    if(tag=="normals") {    normals.readTagged(is, 0);    continue;  }
    if(tag=="faces") {      faces.readTagged(is, 0);      continue;  }
    if(tag=="frames") {     frames.readTagged(is, 0);     continue;  }
    if(tag=="tailsHeads") { tailsHeads.readTagged(is, 0); continue;  }
    if(tag=="groups") {
      is >>PARSE("<") >>j >>PARSE(">");
      G.resize(j);
      for(i=0; i<j; i++) {
        is >>G(i).name;
        G(i).verts.readTagged(is, 0);
      }
      continue;
    }
    if(tag=="graph") {
      is >>PARSE("{") >>i >>j >>PARSE("}");
      graph.resize(j, 2);
      c=is.get(); CHECK_EQ(c, '\n', "couldn't read newline after ascii tag :-(");
      is.read((char*)graph.p, graph.sizeT*graph.N);
      c=is.get(); CHECK_EQ(c, '\n', "couldn't read newline after array buffer :-(");
      continue;
    }
    if(tag=="names") {
      is >>PARSE("<") >>j >>PARSE(">");
      //fscanf(is.rdbuf()->_File, "<%d>", &j);
      names.resize(j);
      for(i=0; i<j; i++) is >>names(i);
      continue;
    }
    if(tag=="quit") { CHECK(is.good(), "not perfect import..."); break; }
    HALT("unknown tag `" <<tag <<"'");
  }

  rai::Vector* w;
  rai::Quaternion r; r.setDeg(0, 1, 0, 0); //don't rotate the mesh
  for(i=0; i<vertices.d0; i++) {
    w = (rai::Vector*)&vertices(i, 0);
    *w = r*(*w);
  }

  mesh.V=vertices;
  mesh.T=faces;
  mesh.Tn=normals;
  /* GROUPS: retired
  mesh.G.resize(vertices.d0); mesh.G=-1;
  for(i=0; i<G.N; i++) {
    for(b=0; b<names.N; b++) { if(G(i).name==names(b)) break; }
    if(b==names.N) {
      cout <<"unknown body: " <<G(i).name <<endl;
    } else {
      for(j=0; j<G(i).verts.N; j++) mesh.G(G(i).verts(j))=b;
    }
  }*/

  mesh.computeNormals();

  double v[4], l;
  rai::Body* n, *p;
  rai::Shape* s;
  String parent;
  rai::Joint* e;
  rai::Vector h, t;
  rai::Transformation f;
  rai::Quaternion ROT; ROT.setDeg(90, 1, 0, 0); //rotate the armature

  for(i=0; i<frames.d0; i++) {
    n=new rai::Body(bl);
    s=new rai::Shape(bl, *n); //always create a shape for a body...
    //rai::skip(is);
    n->name=names(i);
    f.pos.set(&frames(i, 3, 0)); f.pos=ROT*f.pos;
    f.rot.setMatrix(frames[i].sub(0, 2, 0, 2).p);
    f.rot.invert();
    f.addRelativeRotationDeg(90, 1, 0, 0);   f.rot=ROT*f.rot;
    t.set(&tailsHeads(i, 0, 0)); t=ROT*t;
    h.set(&tailsHeads(i, 1, 0)); h=ROT*h;

#if 0
    n->X.p = f.p;
    n->cog = f.r/(t-h)/2.;
#else
    n->X.pos = f.pos + (t-h)/2.;
#endif
    n->X.rot = f.rot;

    s->type=rai::ST_box;
    l=(t-h).length();
    v[0]=v[1]=v[3]=l/20.; v[2]=l;
    memmove(s->size, v, 4*sizeof(double));
  }
  for(i=0; i<graph.d0; i++) {
    p=bl.bodies(graph(i, 0));
    n=bl.bodies(graph(i, 1));
    //e=new_edge(p, n, bl.bodies, bl.joints);
    e=new rai::Joint(bl, p, n);
    e->type = rai::JT_hingeX;
    f.pos.set(&frames(graph(i, 1), 3, 0));  f.pos=ROT*f.pos;
    f.rot.setMatrix(frames[graph(i, 1)].sub(0, 2, 0, 2).p);
    f.rot.invert();
    f.addRelativeRotationDeg(90, 1, 0, 0);  f.rot=ROT*f.rot;

    e->A.setDifference(p->X, f);
    e->B.setDifference(f, n->X); //p=(h-t)/2;
  }
  //graphMakeLists(bl.bodies, bl.joints);
  /* GROUPS: retired
  mesh.GF.resize(bl.bodies.N);
  for(i=0; i<bl.bodies.N; i++) {
    mesh.GF(i) = &(bl.bodies(i)->X);
  }
  mesh.makeVerticesRelativeToGroup();
  mesh.collectTriGroups();
  //bl.calcNodeFramesFromEdges();
  */

  /*
  bl.topsort();
  bl.orderAsIndexed();
  bl.indexAllAsOrdered();
  */
  RAI_MSG("warning - structure is not sorted!");

  String::readEatStopSymbol = false;
}
