#include <Kin/kin.h>
#include <KOMO/komo.h>

//===========================================================================

struct CalibrationScene {
  rai::Configuration& C;
  FrameL cams;
  arrA Fxycxy;
  arrA Distortion;
  rai::Frame * obj;
  uintA obj_aruco_ids;

  FrameL arucos;

  CalibrationScene(rai::Configuration& C, const char* obj_name=0);

  str report();
};

//===========================================================================

struct KomoArucoTracker{
  CalibrationScene& CS;

  std::shared_ptr<KOMO> komo;
  std::shared_ptr<SolverReturn> ret;

  KomoArucoTracker(CalibrationScene& CS) : CS(CS) {}

  void reset(rai::Configuration& C, bool force_contructor=false);
  void addArucoDetected(uint cam_id, uint aruco_id);
  void addPointView(arr p, uint cam_id, uint aruco_id, uint corner_id);
  void solve(int verbose=0, double tolerance=1e-4);
};
