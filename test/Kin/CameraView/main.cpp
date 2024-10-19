
#include <Kin/frame.h>
#include <Kin/cameraview.h>
#include <Geo/depth2PointCloud.h>

//===========================================================================

void TEST(CameraView){
  rai::Configuration C;
  C.addFile("../../../../rai-robotModels/pr2/pr2.g");
  C.addFile("../../../../rai-robotModels/objects/kitchen.g");
  C.optimizeTree();
  C.view();

  rai::CameraView V(C, true);

  V.addSensor(C["endeffKinect"], 640, 480, 580./480., -1., {.1, 50.} );
//  V.selectSensor("kinect");

  byteA image;
  floatA depth;
  byteA segmentation;
  arr pts;

  V.computeImageAndDepth(image, depth);
  segmentation = V.computeSegmentationImage();
  depthData2pointCloud(pts, depth, V.getFxycxy());

  arr D = rai::convert<double>(depth);
  cout <<"depth min max:" <<min(D) <<' ' <<max(D) <<endl;
  depth *= float(255./max(D));

  OpenGL gl;
  gl.text="image";  gl.watchImage(image, true);
  gl.text="depth";  gl.watchImage(depth, true);
  gl.text="segmentation";  gl.watchImage(segmentation, true);

   rai::Mesh M;
   M.V = pts.reshape(-1,3);
   M.C = rai::convert<double>(image).reshape(-1, 3);
   M.C /= 255.;
   gl.data().clear();
   gl.data().addStandardScene();
   gl.data().add().pointCloud(M.V, M.C);
   gl.update(true, "point cloud");

  rai::wait();

}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testCameraView();

  return 0;
}
