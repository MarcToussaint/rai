#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Gui/opengl.h>
#include <Geo/qhull.h>

void drawInit(void*){
  glStandardLight(NULL);
  glDrawAxes(1.);
  glColor(1.,.5,0.);
}

void TEST(Swift) {
  mlr::KinematicWorld G("swift_test.g");

  G.swift().setCutoff(2.);
  G.stepSwift();

  uint t;
  for(t=0;t<50;t++){
    G.bodies(0)->X.addRelativeTranslation(0,0,-.01);
    G.bodies(0)->X.addRelativeRotationDeg(10,1,0,0);
    G.calc_fwdPropagateFrames();

    G.stepSwift();

    G.reportProxies();

    G.watch(true);
  }
}


void TEST(Sphere) {
  OpenGL gl;
  mlr::Mesh mesh;

  //MeshSetTetrahedron(mesh);
  //MeshSetOctahedron(mesh);
  //MeshSetDodecahedron(mesh);
  //MeshSetBox(mesh);
  mesh.setSphere();
  //MeshSetHalfSphere(mesh);
  //MeshSetCylinder(mesh,.2,1.);
  //MeshSetCappedCylinder(mesh,.2,1.);
  gl.add(drawInit,0);
  gl.add(mesh);
  gl.watch();
  
  getTriangulatedHull(mesh.T,mesh.V);

  gl.watch();
}

void TEST(Meshes) {
  OpenGL gl;
  mlr::Mesh mesh;
  mesh.readStlFile(FILE("../../../data/pr2_model/head_v0/head_pan.stl"));
  mesh.scale(.001);
  mesh.writeTriFile(("z.full.tri"));
  mesh.fuseNearVertices(1e-6);
  mesh.writeTriFile(("z.e4.tri"));
  mesh.fuseNearVertices(1e-5);
  mesh.writeTriFile(("z.e3.tri"));
  mesh.fuseNearVertices(1e-4);
  mesh.writeTriFile(("z.e2.tri"));
  gl.add(drawInit,0);
  gl.add(mesh);
  gl.watch();
}

void TEST(Meshes2) {
  mlr::Mesh mesh1,mesh2;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(mesh1);
  mesh1.readTriFile(FILE("z.e3.tri"));
  mesh2.readTriFile(FILE("z.e3.tri"));
  uint i,m=0; double my=mesh1.V(m,1);
  for(i=0;i<mesh1.V.d0;i++) if(mesh1.V(i,1)>mesh1.V(m,1)){ m=i; my=mesh1.V(m,1); }
  mesh2.translate(0,my,0);
  mesh1.addMesh(mesh2);
  //mesh1.writeTriFile(("z.e3.tri"));
  //mesh1.writeOffFile(("z.e3.off"));
  gl.watch();
}

void TEST(Meshes3) {
  mlr::Mesh mesh;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(mesh);
  //MeshSetSphere(mesh,0);
  mesh.readTriFile(FILE("z.e4.tri"));
  gl.reportSelects=true;
  gl.watch();
  if(gl.topSelection){
    cout <<gl.topSelection->name <<endl;
    uint i=gl.topSelection->name >> 4;
    mesh.skin(i);
  }
  gl.watch();
}

int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testSwift();
  testSphere();
  testMeshes3();

  return 0;
}

