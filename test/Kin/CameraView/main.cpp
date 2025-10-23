
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

  V.computeImageAndDepth(image, depth, false);
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
  Cdisp.addFrame("pcl2", "cameraWrist");

  auto opt = make_shared<rai::DepthNoiseOptions>();

  for(uint t=0;t<100;t++){
    //the same method is now integrated in computeImageAndDepth -- this is only to allow debugging
    auto& cam = V.setCamera(C["cameraWrist"]);
    V.computeImageAndDepth(image1, depth1);

    cam.offset.set(.05, .0, .0);
    V.computeImageAndDepth(image2, depth2);

    // opt->noise_all = .02;
    // opt->depth_smoothing = 0;
    rai::simulateDepthNoise(depth1, depth2, cam.offset.x, V.getFxycxy(), opt);

    depthData2pointCloud(pts1, depth1, V.getFxycxy());
    depthData2pointCloud(pts2, depth2, V.getFxycxy());
    pts2.reshape(-1,3);
    for(uint i=0;i<pts2.d0;i++) pts2[i] += cam.offset.getArr();

    {
      Cdisp.getFrame("pcl1") ->setPointCloud(pts1, {255, 200, 100});
      Cdisp.getFrame("pcl2") ->setPointCloud(pts2, {200, 255, 100});
      int key = Cdisp.view(false);
      if(key=='q') break;
      rai::wait(.05);
      // rai::wait();
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
