#include <Gui/opengl.h>
#include <Geo/mesh.h>

void dot(byteA& img, int I, int J){
  I = img.d0 - I; //y-pixel coordinate is flipped
  for(int i=I-3;i<=I+3;i++) for(int j=J-3;j<=J+3;j++){
    if(i>=0 && i<img.d0 && j>=0 && j<img.d1){
      img(i, j, {}) = byteA(3, {255, 0, 0});
    }
  }
}

void testProjection(){
  OpenGL gl, gl2;

  rai::Mesh M;
  M.setBox();
  M.scale(.4, .5, .6);

  gl.add(glStandardLight);
  gl.add(M);
  gl.update(NULL, true);

  gl2.add( [&gl,&M](OpenGL &gl2){
    byteA &img = gl2.background;
    img = gl.captureImage;
    flip_image(img);

    arr X = M.V;
    arr P = gl.camera.getProjectionMatrix();
    arr Pinv = gl.camera.getInverseProjectionMatrix();
    cout <<"--" <<endl;
    for(uint i=0;i<X.d0;i++){
      //-- TEST 3D -> image projection
      //with own projection
      arr y = cat(X[i], {1.});
      arr x = P * y;
      double depth=x(2);
      x /= depth;
      int I=(x(1)+1.)*.5*(double)gl.height;
      int J=(x(0)+1.)*.5*(double)gl.width;
      dot(img, I, J); //drawing literally into the pixel image -> is shown in 2nd window

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
      x(0) = 2.*(double(J)/double(gl.width)) - 1.;
      x(1) = 2.*(double(I)/double(gl.width)) - 1.;
      x(2) = 1.;
      x *= depth;
      x(3) = 1.;
      x = Pinv * x;
      x.resizeCopy(3);
//      cout <<X[i] <<x <<endl;
      cout <<"back-to-3D-err=" <<sqrDistance(X[i], x) <<" (due to pixel discretization)" <<endl;

      //with glut
      px=J;
      py=I;
      pz=pz;
      gl.unproject(px,py,pz, true);
      cout <<"back-to-3D-err=" <<sqrDistance(X[i], {px, py, pz}) <<endl;

    }
  });

  for(;;){
    rai::wait(.1);
    if(gl2.update()=='q') break;
  }

}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  testProjection();

  return 0;
}

