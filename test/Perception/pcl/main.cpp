#include <Core/array.h>
#include <Geo/depth2PointCloud.h>
#include <Gui/viewer.h>
#include <Gui/opengl.h>
#include <Perception/pcl.h>
#include <Perception/pclPlaneExtraction.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <Geo/mesh.h>

void loadPcl(arr& pts, byteA& color){
  floatA depth;
  arr fxypxy;

  ifstream fil("z.depthImage");
  color.readTagged(fil, "color");
  depth.readTagged(fil, "depth");
  fxypxy.readTagged(fil, "fxypxy");

  depthData2pointCloud(pts, depth, fxypxy);

  uint cL=30, cR=30, cT=30, cB=30;
  color = color.sub(cT,-cB,cL,-cR,0,-1);
  pts = pts.sub(cT,-cB,cL,-cR,0,-1);
}

void testPcl(){
  arr pts;
  byteA color;
  loadPcl(pts, color);

  PointCloudViewer V;
  V.rgb.set() = color;
  V.pts.set() = pts;

  auto pclInput = conv_ArrCloud_PclCloud(pts, color);

  pcl::PointCloud<pcl::Normal>::Ptr normal_extracted(new pcl::PointCloud<pcl::Normal>);
  normalEstimator(pclInput, normal_extracted, 50);

  V.normals.set() = conv_PclNormals_Arr(normal_extracted);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*pclInput, *normal_extracted, *cloud_with_normals);

  pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> hoppe;
  hoppe.setInputCloud(cloud_with_normals);
  hoppe.setGridResolution(10,10,10);
  pcl::PolygonMesh mesh;
  hoppe.reconstruct(mesh);

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  pcl::fromPCLPointCloud2( mesh.cloud, point_cloud);
  rai::Mesh M;
  conv_PclCloud_ArrCloud(M.V, M.C, point_cloud);
  M.T.resize(mesh.polygons.size(), 3);
  M.T.setZero();
  for(uint i=0;i<M.T.d0;i++){
    CHECK_EQ(mesh.polygons[i].vertices.size(), 3, "");
    M.T(i,0) = mesh.polygons[i].vertices[0];
    M.T(i,1) = mesh.polygons[i].vertices[1];
    M.T(i,2) = mesh.polygons[i].vertices[2];
  }

  OpenGL gl2;
  gl2.add(glStandardScene);
  gl2.add(M);
  gl2.watch();


  cout <<"bye bye" <<endl;
  rai::wait();
}


int main(int argc,char **argv) {
  rai::initCmdLine(argc,argv);

  testPcl();

  return 0;
}
