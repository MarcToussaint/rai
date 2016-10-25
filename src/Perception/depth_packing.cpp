#include "depth_packing.h"

//REGISTER_MODULE(KinectDepthPacking)

namespace mlr {
  void pack_kindepth2rgb(const uint16A& depth, byteA& buffer) {
    buffer.resize(depth.N, 3);

    uint16_t d;
    byte* rgb;
    for(uint i=0;i<depth.N;i++){
      d=depth.p[i];
      rgb=buffer.p+3*i;
      rgb[0] = 0xff & (d>>4);
      rgb[1] = 0xff & (d>>4);
      rgb[2] = 0x3f & (d<<0); //blue cycles with 64mm depth
#if 0
      //decode:
      uint16_t A=((uint16_t)rgb[0]+rgb[1])<<3;
      uint16_t B=((A&0x30) + (rgb[2]&0x30))>>1;
      uint16_t depth = (A&0xfc0) | B | (rgb[2]&0x00f);
      CHECK_EQ(d,depth,"ups "<<A <<' ' <<B <<' ' <<(int)(rgb[2]&0x00f));
#endif
    }
    buffer.reshape(depth.d0, depth.d1, 3);
  }
}

void KinectDepthPacking::open(){}
void KinectDepthPacking::close(){}

void KinectDepthPacking::step(){
  kinect_depth.readAccess();
  kinect_depthRgb.writeAccess();


  mlr::pack_kindepth2rgb(kinect_depth(), kinect_depthRgb());

  kinect_depthRgb().reshape(kinect_depth().d0, kinect_depth().d1, 3);
  kinect_depthRgb.v->revision_time = kinect_depth.v->revision_time;

  kinect_depthRgb.deAccess();
  kinect_depth.deAccess();

}
