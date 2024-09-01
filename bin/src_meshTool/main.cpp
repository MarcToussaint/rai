#include <stdlib.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/RenderData.h>

//#include "swift_decomposer.inc"
#include <Gui/color.h>

const char *USAGE=
"\n\
Usage:  meshTool file.* <tags...>\n\
\n\
Tags can be -hide, -box, -fuse, -clean, -center, -scale, -qhull, -flip \n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  if(!rai::checkCmdLineTag("hide")) cout <<USAGE <<endl;

  rai::String file=rai::getParameter<rai::String>("file",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') file=rai::argv[1];
  LOG(0) <<"== opening mesh file `" <<file <<"'";

  if(file=="none") return 0;

  OpenGL *gl=nullptr;

  rai::Mesh mesh;
  mesh.readFile(file);

  bool edited=false;
  file(file.N-4)=0; //replace . by 0

  //modify
  if(!rai::checkCmdLineTag("hide")){
    cout <<"== mesh info ==";
    cout <<"  #vertices: " <<mesh.V.d0
        <<"\n  #triangles: " <<mesh.T.d0
        <<"\n  #colors: " <<mesh.C.d0 << " max color: " <<(mesh.C.N?max(mesh.C):0.)
        <<"\n  #Vnormals: " <<mesh.Vn.d0
        <<"\n  #Tnormals: " <<mesh.Tn.d0
       <<"\n  bounds: " <<mesh.getBounds()
      <<"\n  center: " <<mesh.getMean()
     <<"\n  area: " <<(mesh.T.N?mesh.getArea():0)
    <<"\n  volume: " <<(mesh.T.N?mesh.getVolume():0)
    <<"\n  cvxParts: " <<mesh.cvxParts.N
    <<endl;

    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->camera.setPosition(2., -2., 2.);
    gl->camera.focus(0., 0., 0., true);
    gl->clearColor = {.7f, .7f, .7f};
    gl->text = "before operations";
    gl->data().addStandardScene().add().mesh(mesh);
    gl->update(true);
  }

  if(rai::checkCmdLineTag("scale")){
    double s;
    rai::getParameter(s,"scale");
    cout <<"  scaling.. " <<s <<endl;
    mesh.scale(s);
    edited=true;
  }

  if(rai::checkCmdLineTag("qhull")){
    cout <<"  making convex.." <<endl;
    mesh.deleteUnusedVertices();
#ifdef RAI_QHULL
    getTriangulatedHull(mesh.T,mesh.V);
#else
    RAI_MSG("can'd use qhull - compiled without RAI_QHULL flag");
#endif
    edited=true;
  }

  if(rai::checkCmdLineTag("fuse")){
    double f = rai::getParameter<double>("fuse");
    cout <<"  fusing near vertices.. " <<f <<endl;
    mesh.fuseNearVertices(f);
    edited=true;
  }

  if(rai::checkCmdLineTag("clean")){
    cout <<"  cleaning.. " <<endl;
    mesh.clean();
    edited=true;
  }

  if(rai::checkCmdLineTag("flip")){
    cout <<"  flipping flaces.. " <<endl;
    mesh.flipFaces();
    edited=true;
  }

  if(rai::checkCmdLineTag("center")){
    cout <<"  centering.." <<endl;
    mesh.center();
    edited=true;
  }

   if(rai::checkParameter<bool>("decomp")){
     cout <<"  decomposing..." <<endl;
     rai::Mesh M = mesh.decompose();
     mesh = M;
     edited=true;
   }

  if(edited && !rai::checkCmdLineTag("hide")){
    cout <<"== after edit ==";
    cout <<"  #vertices: " <<mesh.V.d0
        <<"\n  #triangles: " <<mesh.T.d0
       <<"\n  bounds: " <<mesh.getBounds()
      <<"\n  center: " <<mesh.getMean()
     <<"\n  area: " <<mesh.getArea()
    <<"\n  volume: " <<mesh.getVolume()
    <<"\n  cvxParts: " <<mesh.cvxParts.N
    <<endl;

    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->text = "after operations";
    gl->data().clear().addStandardScene().add().mesh(mesh);
    gl->update(true);
  }

  cout <<"  saving.. to z.*" <<endl;
  mesh.writePLY("z.ply", true);
  mesh.writeArr(FILE("z.arr"));
  mesh.writeJson(FILE("z.msh"));
  mesh.writeH5("z.h5", "mesh");

  cout <<"== bye bye ==" <<endl;

  return 0;
}
