#include "depth_packing.h"

REGISTER_MODULE(KinectDepthPacking)

namespace MLR {
	void pack_kindepth2rgb(const uint16A& depth, byteA& buffer) {
		buffer.resize(depth.N, 3);

		uint16_t d;
		byte* rgb;
		for(uint i=0;i<depth.N;i++){
			d=depth.p[i];
			rgb=buffer.p+3*i;
			rgb[0] = 0xff & (d>>4);
			rgb[1] = 0x03 & (d>>2);
			rgb[2] = 0x03 & d;
		}
		buffer.reshape(depth.d0, depth.d1, 3);
	}
}

void KinectDepthPacking::open(){}
void KinectDepthPacking::close(){}

void KinectDepthPacking::step(){
    kinect_depth.readAccess();
    kinect_depthRgb.writeAccess();

    double tstamp = kinect_depth.tstamp();

    MLR::pack_kindepth2rgb(kinect_depth(), kinect_depthRgb());

    kinect_depthRgb().reshape(kinect_depth().d0, kinect_depth().d1, 3);
    kinect_depthRgb.tstamp() = tstamp;

    kinect_depthRgb.deAccess();
    kinect_depth.deAccess();

}
