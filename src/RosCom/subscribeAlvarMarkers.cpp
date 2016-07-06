#include "subscribeAlvarMarkers.h"

#ifdef MLR_ROS

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#endif

// ============================================================================
// void ROSMODULE_markers::step() {
//   modelWorld.writeAccess();
//   AlvarMarkers markers_ = markers.get();
//   syncMarkers(modelWorld.set()(), markers_);
//   modelWorld.deAccess();
// }

// ============================================================================
void setBody(ors::Body& body, const ar::AlvarMarker& marker) {
  body.X = conv_pose2transformation(marker.pose.pose);
}


void syncMarkers(ors::KinematicWorld& world, const ar::AlvarMarkers& markers) {
  bool createdNewMarkers = false;

  // transform: torso_lift_link is the reference frame_id
  ors::Body *torso = world.getBodyByName("torso_lift_link", false);
  if(!torso) return; //TODO: make this more general!
  ors::Transformation refFrame = torso->X;

  for (const ar::AlvarMarker& marker : markers.markers) {
    mlr::String marker_name = STRING("marker" << marker.id);

    ors::Body *body = world.getBodyByName(marker_name);
    if (not body) {
      createdNewMarkers = true;
      cout << marker_name << " does not exist yet; adding it..." << endl;
      body = new ors::Body(world);
      body->name = marker_name;
      ors::Shape *shape = new ors::Shape(world, *body);
      shape->name = marker_name;
      shape->type = ors::markerST;
      shape->size[0] = .01; shape->size[1] = .0; shape->size[2] = .0; shape->size[3] = .0;
    }
    setBody(*body, marker);

    body->X.appendTransformation(refFrame);
    body->X.addRelativeRotationDeg(90.,0.,1.,0.);

    world.getShapeByName(marker_name)->X = body->X;

  }

  if (createdNewMarkers) {
    world.swiftDelete();
  }
}
#else
void setBody(ors::Body& body, const AlvarMarker& marker) {}
void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers) {}
#endif
