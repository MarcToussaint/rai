#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Control/CtrlMsgs.h>

#include "calibration.h"

namespace rai {

struct ArucoThread;

//===========================================================================

//===========================================================================

struct NaiveTrackerFilter {
    arr q, qdel;
    double good_ratio=0;
    double threshold;
    double alpha=.7, beta=.3, gamma=.1;
    double err_filtered;

    NaiveTrackerFilter(double threshold=.02);

    void update(const arr& q_measured);
};

//===========================================================================

struct KomoArucoTracker{
  CalibrationScene CS;

  std::shared_ptr<KOMO> komo;
  std::shared_ptr<SolverReturn> ret;
  NaiveTrackerFilter filter;

  KomoArucoTracker(Configuration& C, const char* obj_name) : CS(C, obj_name) {}

  void reset(bool force_contructor=false);
  void addArucoDetected(uint cam_id, uint aruco_id);
  void addPointView(arr p, uint cam_id, uint aruco_id, uint corner_id);
  void addMultiPointView(const intA& ids, const arr& pts, uint cam_id, bool undistort_points = true);

  void solve(int verbose=0, double tolerance=1e-4);
};

//===========================================================================

struct KomoArucoTracker_Thread : Thread {
    Array<std::shared_ptr<ArucoThread>> aruco_threads;
    Var<CtrlStateMsg>& state;
    Var<arr> obj_pose;
    KomoArucoTracker tracker;

    KomoArucoTracker_Thread(const Array<std::shared_ptr<ArucoThread>>& aruco_threads,
                            Var<CtrlStateMsg>& state,
                            Configuration& C, const char* obj_name);
    ~KomoArucoTracker_Thread();

    void step();
};

} //namespace
