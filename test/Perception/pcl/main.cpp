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
  //-- grab point cloud
  arr pts;
  byteA color;
  loadPcl(pts, color);
  pts.reshape(-1,3);
  color.reshape(-1,3);

  //-- filter by depth
  double depthNear=-1., depthFar=-2.;
  uintA select;
  for(uint i=0;i<pts.d0;i++) if(pts(i,2)<depthNear && pts(i,2)>depthFar) select.append(i);
  pts = pts.sub(select);
  color = color.sub(select);

  //-- make to mesh
  rai::Mesh M0;
  M0.V = pts;
  copy(M0.C, color);
  M0.C *= 1./255.;

  //-- watch
  OpenGL gl;
  gl.drawOptions.drawWires=true;
  gl.add(M0);
  gl.watch();


  //-- estimate normals
  auto cloud = conv_ArrCloud_PclCloud(pts);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(20);
    ne.compute(*normals);
  }

  //-- watch
  M0.Vn = conv_PclNormals_Arr(normals);
  gl.watch();

  //-- compute 3D signed distance grid
  arr ptsMin = min(pts,0);
  arr ptsMax = max(pts,0);
  int res = 100;
  arr delta = (ptsMax - ptsMin)/double(res);
  arr F(res+1, res+1, res+1);
  F = 1.;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  tree->setInputCloud(cloud);
  for (int x = 0; x <= res; ++x)
    for (int y = 0; y <= res; ++y)
      for (int z = 0; z <= res; ++z){
        arr pt = {x*delta(0)+ptsMin(0), y*delta(1)+ptsMin(1), z*delta(2)+ptsMin(2)};
        pcl::PointXYZ p(pt(0), pt(1), pt(2));
        uint neigh=5;
        tree->nearestKSearch(p, neigh, k_indices, k_sqr_distances);
        uint posCount=0, negCount=0;
        double posD=0., negD=0.;
        for(uint i=0;i<neigh;i++){
          arr del = pt - pts[k_indices[i]];
          arr normal = M0.Vn[k_indices[i]];
          double err = sumOfSqr(del) - k_sqr_distances[i];
          CHECK_LE(err, 1e-6, "");
          double len = length(del);

          double align = scalarProduct(normal, del);
          if(align > .1*len){ posCount++; posD+=len; }
          if(align < -.1*len){ negCount++; negD+=len; }
        }
        if(posCount>2 || negCount>2){
          if(posCount>negCount) F(x,y,z) = posD/double(posCount);
          else F(x,y,z) = -negD/double(negCount);
        }
      }


  rai::Mesh M;
  M.setImplicitSurface(F-.0, ptsMin, ptsMax);
  M.C = {.9,.9,.9,.6};
  // Concatenate the XYZ and normal fields*
//  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);



  gl.add(M);
  gl.watch();

  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv) {
  rai::initCmdLine(argc,argv);

  testPcl();

  return 0;
}

#if 0
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
#endif
