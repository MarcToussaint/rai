
#include <Kin/frame.h>
#include <Kin/cameraview.h>
#include <Geo/depth2PointCloud.h>

//===========================================================================

void TEST(CameraView){
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  C.addFile("../../../../rai-robotModels/objects/kitchen.g");
  C.addFrame("mesh") ->setMeshFile("$RAI_PATH/g1/meshes/head_link.h5") .setColor({1.,.8,.5}) .setPosition({0,.5,.4});
  C.view();

  rai::CameraView V(C, true);

  V.setCamera(C["cameraWrist"]);

  byteA image;
  floatA depth;
  byteA segmentation;
  arr pts;

  V.computeImageAndDepth(image, depth);
  segmentation = V.computeSegmentationImage();
  depthData2pointCloud(pts, depth, V.getFxycxy());

  cout <<"depth min max:" <<min(depth) <<' ' <<max(depth) <<endl;
  depth *= float(255./max(depth));

  {
    OpenGL gl;
    gl.text="image";  gl.watchImage(image, true);
    gl.text="depth";  gl.watchImage(depth, true);
    gl.text="segmentation";  gl.watchImage(segmentation, true);
  }

  {
    rai::Configuration C;
    C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
    C.addFrame("pcl", "cameraWrist") ->setPointCloud(pts, {255, 200, 100});
    C.view(true);
  }

  rai::wait();
}

//===========================================================================

void simulateDepthNoise(floatA& depth, const floatA& depth2, double offset, const arr& fxycxy){
  //-- wierd noise
  floatA noise = rai::convert<float>(4.*randn(depth.d0, depth.d1));
  //smooth noise
  for(uint k=0;k<3;k++){
    noise = rai::integral(noise);
    noise = rai::differencing(noise, 20);
  }
  //local noise
  noise += rai::convert<float>(.4*randn(depth.d0, depth.d1));
  for(uint k=0;k<3;k++){
    noise = rai::integral(noise);
    noise = rai::differencing(noise, 3);
  }
  //fine noise
  noise += rai::convert<float>(.04*randn(depth.d0, depth.d1));

  // { OpenGL gl;  noise *= 255.f;  gl.watchImage(noise, true); }

  //-- smoothed depth
  for(uint k=0;k<1;k++){
    depth = rai::integral(depth);
    depth = rai::differencing(depth, 5);
  }

  //-- shadows
  byteA shadowImg(depth.d0, depth.d1);
  shadowImg = 255;
  for(uint i=0;i<depth.d0;i++){
    for(uint j=0;j<depth.d1; j++){
      float& d = depth(i,j);
      int i2 = int(i) + fxycxy(0)*d*offset;
      bool shadow=false;
      if(i2>=depth.d0) shadow=true;
      else if(fabs(depth2(i2,j) - d)>.05) shadow=true;

      d += .05*noise(i,j);

      if(shadow){
        d=-1.;
        shadowImg(i,j) = 0;
      }
    }
  }

  // { OpenGL gl;  gl.watchImage(shadowImg, true); }
}

// =============================================================================

void TEST(NoisyDepth){
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  C.addFile("../../../../rai-robotModels/objects/kitchen.g");
  C.addFrame("mesh") ->setMeshFile("$RAI_PATH/g1/meshes/head_link.h5") .setColor({1.,.8,.5}) .setPosition({.1,.5,.4});
  C.addFrame("box") ->setShape(rai::ST_box, {.1,.1,.1}) .setColor({1.,.8,.5}) .setPosition({-.2,.5,.7});
  C.view();

  rai::CameraView V(C, true);

  byteA image1, image2;
  floatA depth1, depth2;
  arr pts1, pts2;

  rai::Configuration Cdisp;
  Cdisp.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  Cdisp.addFrame("pcl1", "cameraWrist");

  for(uint t=0;t<100;t++){
    auto& cam = V.setCamera(C["cameraWrist"]);
    V.computeImageAndDepth(image1, depth1);

    cam.offset.set(.05, .0, .0);
    V.computeImageAndDepth(image2, depth2);

    simulateDepthNoise(depth1, depth2, cam.offset.x, V.getFxycxy());

    depthData2pointCloud(pts1, depth1, V.getFxycxy());
    depthData2pointCloud(pts2, depth2, V.getFxycxy());
    pts2.reshape(-1,3);
    for(uint i=0;i<pts2.d0;i++) pts2[i] += cam.offset.getArr();

    {
      Cdisp.getFrame("pcl1") ->setPointCloud(pts1, {255, 200, 100});
      int key = Cdisp.view(false);
      if(key=='q') break;
      rai::wait(.05);
    }

  }
  Cdisp.view(true);
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  // testCameraView();

  testNoisyDepth();

  return 0;
}
