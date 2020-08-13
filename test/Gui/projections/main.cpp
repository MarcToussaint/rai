#include <Gui/opengl.h>
#include <Geo/mesh.h>

void dot(byteA& img, int I, int J){
  I = img.d0 - I; //y-pixel coordinate is flipped
  for(int i=I-3;i<=I+3;i++) for(int j=J-3;j<=J+3;j++){
    if(i>=0 && i<(int)img.d0 && j>=0 && j<(int)img.d1){
      img(i, j, {}) = byteA({3}, {255, 0, 0});
    }
  }
}

void testProjection(){
  OpenGL gl, gl2;
  gl.camera.zNear = .1;

  rai::Mesh M;
  M.setBox();
  M.scale(.4, .5, .6);

  gl.add(glStandardLight);
  gl.add(M);
  gl.update(nullptr, true);

  gl2.add( [&gl,&M](OpenGL &gl2){
    byteA &img = gl2.background;
    img = gl.captureImage;
    flip_image(img);

    arr X = M.V;
    cout <<"--" <<endl;
    for(uint i=0;i<X.d0;i++){
      //-- TEST 3D -> image projection
      //with own
      arr y = X[i].copy();
      gl.camera.project2PixelsAndTrueDepth(y, gl.width, gl.height);
      int I=y(1), J=y(0);
      double depth=y(2);
      dot(img, I, J);

      //with glut
      double px=X(i,0), py=X(i,1), pz=X(i,2);
      gl.project(px,py,pz, true);
      int I2=py;
      int J2=px;

      if(!(I-I2) && !(J-J2)){
        cout <<"Success -- OWN and GLU match" <<endl;
      }else{
        cout <<"OWN-GLU:" <<I - I2 <<' ' <<J-J2 <<endl;
      }
      cout <<"depth-match-err=" <<gl.camera.glConvertToTrueDepth(pz) - depth <<" (due to GL's non-linear depth transform?)" <<endl;

      //-- TEST image -> 3D projection
      //with own
      gl.camera.unproject_fromPixelsAndTrueDepth(y, gl.width, gl.height);
      cout <<"back-to-3D-err=" <<sqrDistance(X[i], y)<<endl;

      //with glut
      y(0)=px; y(1)=py; y(2)=pz;
      gl.camera.unproject_fromPixelsAndGLDepth(y, gl.width, gl.height);
      cout <<"back-to-3D-err=" <<sqrDistance(X[i], y) <<" (float precision only)" <<endl;
    }
  });

  for(uint k=0;k<50;k++){
    rai::wait(.1);
    if(gl2.update()=='q') break;
  }

}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  testProjection();

  return 0;
}

