#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>

#ifdef MLR_ROS
#  include "roscom.h"
#  ifdef MLR_ROS_KINETIC
#    include <ar_track_alvar_msgs/AlvarMarkers.h>
     namespace ar = ar_track_alvar_msgs;
#  endif
#  ifdef MLR_ROS_INDIGO
#    include <ar_track_alvar_msgs/AlvarMarkers.h>
     namespace ar = ar_track_alvar_msgs;
#  endif
#  if MLR_ROS_GROOVY
#    include <ar_track_alvar/AlvarMarkers.h>
     namespace ar = ar_track_alvar_msgs;
#  endif
#else
  struct AlvarMarker{ AlvarMarker(){NICO} };
  struct AlvarMarkers{ AlvarMarkers(){NICO} };
#endif


//===========================================================================
/// Generic subscriber to the AR maker alvar
//ROSSUB("/ar_pose_marker", AlvarMarkers, ar_pose_marker)


//===========================================================================
// using mlr::KinematicWorld;  // this is necessary to make the macro work.

// /// Simple syncing of the ors world "modelWorld" with ar_pose_marker
// BEGIN_ROSMODULE("/ar_pose_marker", AlvarMarkers, markers)
//   ACCESS(KinematicWorld, modelWorld)
// END_ROSMODULE()


//===========================================================================
// Helper functions

/**
 * Set the transformation of the body to the transformation of the alvar maker.
 */
void setBody(mlr::Body& body, const ar::AlvarMarker& marker);

/**
 * Sync all markers from the msg with the ors world.
 *
 * Note: this never deletes old markers.
 */
void syncMarkers(mlr::KinematicWorld& world, const ar::AlvarMarkers& markers);

struct AlvarSyncer : Thread {
  Access<mlr::KinematicWorld> modelWorld;
  Access<ar::AlvarMarkers> ar_pose_markers;
  AlvarSyncer()
   : Thread("AlvarSyncer"),
    modelWorld(this, "modelWorld", true),
    ar_pose_markers(this, "ar_pose_markers") {}
  void open() {}
  void step() {
    syncMarkers(modelWorld.set(), ar_pose_markers.get());
  }
  void close() {}
};

struct SubscribeAlvar{
  Access<ar::AlvarMarkers> ar_pose_markers;
  Subscriber<ar::AlvarMarkers> sub;

  SubscribeAlvar()
    : ar_pose_markers(NULL, "ar_pose_markers"),
      sub("/ar_pose_marker", ar_pose_markers) {
  }
  ~SubscribeAlvar(){
  }

};


