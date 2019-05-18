#include <stdlib.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

//#include "swift_decomposer.inc"
#include <Gui/color.h>

const char *USAGE=
"\n\
Usage:  rai_meshTools file.[tri|obj|off|ply|stl] <tags...>\n\
\n\
Tags can be -view, -box, -fuse, -clean, -center, -scale, -save, -qhull, -flip \n";

void drawInit(void*, OpenGL& gl){
  glStandardLight(NULL, gl);
  glDrawAxes(1.);
  glColor(1.,.5,0.);
}

void TEST(MeshTools) {
  cout <<USAGE <<endl;

  rai::String file;
  if(rai::argc>=2) file=rai::argv[1];
  else HALT("the first argument needs to be a mesh file");

  cout <<"FILE=" <<file <<endl;
  OpenGL *gl=NULL;

  rai::Mesh mesh;
  mesh.readFile(file);

  cout <<"#vertices = " <<mesh.V.d0 <<" #triangles=" <<mesh.T.d0 <<"bounding box: = " <<max(mesh.V, 0) <<'-' <<min(mesh.V, 0) <<endl;
//  mesh.C.clear();

  file(file.N-4)=0; //replace . by 0

  //modify
  if(rai::checkCmdLineTag("view")){
    cout <<"viewing..." <<endl;
    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->add(drawInit);
    gl->add(mesh);
    gl->watch();
  }
  if(rai::checkCmdLineTag("box")){
    cout <<"box" <<endl;
    mesh.box();
  }
  if(rai::checkCmdLineTag("scale")){
    double s;
    rai::getParameter(s,"scale");
    cout <<"scale " <<s <<endl;
    mesh.scale(s);
  }
  if(rai::checkCmdLineTag("qhull")){
    cout <<"qhull..." <<endl;
    mesh.deleteUnusedVertices();
#ifdef RAI_QHULL
    getTriangulatedHull(mesh.T,mesh.V);
#else
    RAI_MSG("can'd use qhull - compiled without RAI_QHULL flag");
#endif
  }
  if(rai::checkCmdLineTag("fuse")){
    double f;
    rai::getParameter(f,"fuse");
    cout <<"fuse " <<f <<endl;
    mesh.fuseNearVertices(f);
  }
  if(rai::checkCmdLineTag("clean")){
    cout <<"clean" <<endl;
    mesh.clean();
  }
  if(rai::checkCmdLineTag("flip")){
    cout <<"clean" <<endl;
    //mesh.fuseNearVertices(1e-2);
    mesh.flipFaces();
  }
  if(rai::checkCmdLineTag("center")){
    cout <<"center" <<endl;
    mesh.center();
  }
  // if(rai::checkCmdLineTag("swift")){
  //   mesh.writeTriFile(STRING(file<<"_x.tri"));
  //   rai::String cmd;
  //   cmd <<"decomposer_c-vs6d.exe -df " <<file <<"_x.dcp -hf " <<file <<"_x.chr " <<file <<"_x.tri";
  //   cout <<"swift: " <<cmd <<endl;
  //   if(::system(cmd)) RAI_MSG("system call failed");
  // }
  // if(rai::checkCmdLineTag("decomp")){
  //   NIY;
  //   // cout <<"decomposing..." <<endl;
  //   // intA triangleAssignments;
  //   // rai::Array<rai::Array<uint> > shapes;
  //   // decompose(mesh, STRING(file<<"_x.dcp"), triangleAssignments, shapes);
  //   // mesh.C.resize(mesh.T.d0,3);
  //   // for(uint t=0;t<mesh.T.d0;t++){
  //   //   rai::Color col;
  //   //   col.setIndex(triangleAssignments(t));
  //   //   mesh.C(t,0) = col.r;  mesh.C(t,1) = col.g;  mesh.C(t,2) = col.b;
  //   // }
  // }
  if(rai::checkCmdLineTag("view")){
    cout <<"viewing..." <<endl;
    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->add(drawInit);
    gl->add(mesh);
    gl->watch();
  }
  if(rai::checkCmdLineTag("save")){
    cout <<"saving..." <<endl;
    cout << "\tto " << file <<"_x.tri" << endl;
    mesh.writeTriFile(STRING(file<<"_x.tri"));
    cout << "\tto " << file <<"_x.off" << endl;
    mesh.writeOffFile(STRING(file<<"_x.off"));
    cout << "\tto " << file <<"_x.ply" << endl;
    mesh.writePLY(STRING(file<<"_x.ply"), true);
  }

  cout <<"#vertices = " <<mesh.V.d0 <<" #triangles=" <<mesh.T.d0 <<endl;
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testMeshTools();

  return 1;
}
