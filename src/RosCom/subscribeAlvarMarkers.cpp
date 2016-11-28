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
void setBody(mlr::Body& body, const ar::AlvarMarker& marker) {
  body.X = conv_pose2transformation(marker.pose.pose);
}


void syncMarkers(mlr::KinematicWorld& world, const ar::AlvarMarkers& markers) {
  bool createdNewMarkers = false;

  // transform: torso_lift_link is the reference frame_id
  mlr::Body *torso = world.getBodyByName("torso_lift_link", false);
  if(!torso) return; //TODO: make this more general!
  mlr::Transformation refFrame = torso->X;

  for (const ar::AlvarMarker& marker : markers.markers) {
    mlr::String marker_name = STRING("marker" << marker.id);

    mlr::Body *body = world.getBodyByName(marker_name,false);
    if (not body) {
      createdNewMarkers = true;
      cout << marker_name << " does not exist yet; adding it..." << endl;
      body = new mlr::Body(world);
      body->name = marker_name;
      mlr::Shape *shape = new mlr::Shape(world, *body);
      shape->name = marker_name;
      shape->type = mlr::ST_marker;
      shape->size[0] = .3; shape->size[1] = .0; shape->size[2] = .0; shape->size[3] = .0;
    }
    mlr::Vector Y_old;
    mlr::Vector Z_old;
    Z_old = world.getShapeByName(marker_name)->X.rot.getZ();
    Y_old = world.getShapeByName(marker_name)->X.rot.getY();
    setBody(*body, marker);
    mlr::Transformation T;

    T.setZero();
    T.addRelativeRotationDeg(90.,0.,1.,0.);

    body->X = refFrame * T * body->X;
    body->X.addRelativeRotationDeg(-90.,0.,1.,0.);
    body->X.addRelativeRotationDeg(-90.,1.,0.,0.);

   // while (body->X.rot.getX().theta()  < M_PI / 2. || body->X.rot.getY().theta()  < M_PI / 2.){
  //   body->X.addRelativeRotationDeg(-90.,0.,0.,1.);
      //cout << "test" << endl;
   // }
    /*int i = 0;
     while ( body->X.rot.getZ().angle(Z_old) > 1.3 || body->X.rot.getY().angle(Y_old) > 1.3){

     body->X.addRelativeRotationDeg(-90.,1.,0.,0.);
     //cout << body->X.rot.getX().angle(X_old) << "new" << i << endl;
      if (i == 3)break;
      i++;
   }*/

    world.getShapeByName(marker_name)->X = body->X;

  }

  if (createdNewMarkers) {
    world.swiftDelete();
  }
}
#else
void setBody(mlr::Body& body, const AlvarMarker& marker) {}
void syncMarkers(mlr::KinematicWorld& world, AlvarMarkers& markers) {}
#endif
