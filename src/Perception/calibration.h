#pragma once

#include <Kin/kin.h>

#include "aruco.h"

namespace rai {

//===========================================================================

byteAA undistort_images(rai::Configuration& C, const byteAA& org);
byteAA render_images(rai::Configuration& C, const arr& qs, uint T=0);

//defined in aruco.h:
// void undistort_point(arr& p, const arr& fxycxy, const arr& distortion);
// byteA undistort_image(const byteA& img, const arr& fxycxy, const arr& distortion);
// std::tuple<intAA, arrA> detect_arucos(const byteAA& imgs, int verbose=0);
// std::tuple<arrA, arrA> calibrate_intrinsics(const byteAA& imgs, uint distortionDofs, int verbose=2, float square_len_m=0.055, float marker_len_m=0.041);

//===========================================================================

struct CalibrationScene {
    Configuration& C;

    FrameL cams;
    FrameL arucos;
    FrameL calibs;
    FrameL calibs_joints;

    arrA Fxycxy;
    arrA Distortion;
    Frame * obj;
    uintA obj_aruco_ids;

    CalibrationScene(Configuration& C, const char* obj_name=0);

    //-- setup calib dof frames
    void addCalibDofs_arucos();
    void addCalibDofs_cameras();
    void addCalibDofs_joints(const uintA& jointIds);

    str report();
};

//===========================================================================

void komo_calibrate(Configuration& C,
                    const intAA& ids, const arrA& pts, const arr& qs,
                    const uintA& exclude_times,
                    bool calibrate_cams = true,
                    bool calibrate_arucos = true,
                    bool calibrate_joints = true,
                    bool calibrate_objPoses = false,
                    double calib_joint_regularization = 1e1,
                    int verbose=1);

//===========================================================================

} //namespace
