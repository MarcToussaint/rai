#include <RosCom/roscom.h>
#include "perceptionCollection.h"

Collector::Collector(const bool simulate):
    Module("Collector", simulate ? 0.05:-1),
    tabletop_clusters(this, "tabletop_clusters", !simulate),
    ar_pose_markers(this, "ar_pose_markers", !simulate)
{
  tabletop_srcFrame.set()->setZero();
  alvar_srcFrame.set()->setZero();
  this->simulate = simulate;
//  tf.setZero();
}

void Collector::step()
{
  FilterObjects percepts;
  percepts.clear();

  if (!simulate)
  {

    int cluster_rev = tabletop_clusters.readAccess();
    if ( cluster_rev > tabletop_clusters_revision ){ //only if new cluster info is available
      tabletop_clusters_revision = cluster_rev;
      const visualization_msgs::MarkerArray msg = tabletop_clusters();

      if (msg.markers.size() > 0){
#if 0
        if (!has_transform) { //get the transform from ROS
          //MT: markers and clusters have the same transformation??
          // Convert into a position relative to the base.
          tf::TransformListener listener;
          tf::StampedTransform baseTransform;
          try{
            //MT: why not use ros_getTransform?
            listener.waitForTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), baseTransform);
            tf = conv_transform2transformation(baseTransform);
            //MT: really add the meter here? This seems hidden magic numbers in the code. And only for Baxter..?
            ors::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            tf = inv;
            has_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }
#endif

        for(auto & marker : msg.markers){
          Cluster* new_cluster = new Cluster(conv_ROSMarker2Cluster( marker ));
          new_cluster->frame = tabletop_srcFrame.get(); //tf
          percepts.append( new_cluster );
        }
      }
    }
    tabletop_clusters.deAccess();

    int ar_rev = ar_pose_markers.readAccess();
    if ( ar_rev > ar_pose_markers_revision){ //new alwar objects are available
      ar_pose_markers_revision = ar_rev;
      const ar::AlvarMarkers msg = ar_pose_markers();

      for(auto & marker : msg.markers) {
#if 0
        if (!has_transform)
        {
          // Convert into a position relative to the base.
          tf::TransformListener listener;
          tf::StampedTransform baseTransform;
          try{
            listener.waitForTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), baseTransform);
            tf = conv_transform2transformation(baseTransform);
            ors::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            tf = inv;
            has_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }
#endif

        Alvar* new_alvar = new Alvar( conv_ROSAlvar2Alvar(marker) );
        new_alvar->frame = alvar_srcFrame.get(); //tf;
        percepts.append( new_alvar );
      }
    }
    ar_pose_markers.deAccess(); //MT: make the access shorter, only around the blocks above
	
	int body_rev = opti_bodies.readAccess();
    if ( body_rev > opti_bodies_revision)
    {
      opti_bodies_revision = body_rev;
      const std::vector<geometry_msgs::TransformStamped> msgs = opti_bodies();
      for(uint i=0; i<msgs.size(); i++){
          geometry_msgs::TransformStamped msg = msgs.at(i);
          if (!has_transform)
          {
           // Convert into a position relative to the base.
            tf::TransformListener listener;
            tf::StampedTransform baseTransform;
            try{
                listener.waitForTransform("/world", msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/world", msg.header.frame_id, ros::Time(0), baseTransform);
                tf = conv_transform2transformation(baseTransform);
                ors::Transformation inv;
                inv.setInverse(tf);
                inv.addRelativeTranslation(0,0,-1);
                inv.setInverse(inv);
                tf = inv;
                has_transform = true;
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
            }
          }
          OptitrackBody* new_optitrack_body = new OptitrackBody(conv_tf2OptitrackBody( msg ));
          new_optitrack_body->frame = tf;
          percepts.append( new_optitrack_body );
        }
    }
	opti_bodies.deAccess();

	int marker_rev = opti_markers.readAccess();
    if ( marker_rev > opti_markers_revision)
    {
      opti_markers_revision = marker_rev;
      const std::vector<geometry_msgs::TransformStamped> msgs = opti_markers();

      for(uint i=0; i<msgs.size(); i++){
          geometry_msgs::TransformStamped msg = msgs.at(i);

          if (!has_transform)
          {
           // Convert into a position relative to the base.
            tf::TransformListener listener;
            tf::StampedTransform baseTransform;
            try{
                listener.waitForTransform("/world", msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/world", msg.header.frame_id, ros::Time(0), baseTransform);
                tf = conv_transform2transformation(baseTransform);
                ors::Transformation inv;
                inv.setInverse(tf);
                inv.addRelativeTranslation(0,0,-1);
                inv.setInverse(inv);
                tf = inv;
                has_transform = true;
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
            }
          }
          OptitrackMarker* new_optitrack_marker = new OptitrackMarker(conv_tf2OptitrackMarker( msg ));
          new_optitrack_marker->frame = tf;
          percepts.append( new_optitrack_marker );
        }
    }
	opti_markers.deAccess();

  }
  else // If 'simulate', make a fake cluster and alvar
  {
    ors::Mesh box;
    box.setBox();
    box.subDivide();
    box.subDivide();
    box.subDivide();
    rndUniform(box.V, -0.05, 0.05, true);
    box.scale(0.1);

    Cluster* fake_cluster = new Cluster( ARR(0.6, 0., 0.05),  // mean
                                         box.V,               // points
                                         "/base_footprint");  // frame
    fake_cluster->frame.setZero();
    fake_cluster->frame.addRelativeTranslation(0.6, 0., 1.05);
    ors::Quaternion rot;
    rot.setDeg(30, ors::Vector(0.1, 0.25, 1));
    fake_cluster->frame.addRelativeRotation(rot);
    percepts.append( fake_cluster );

    Alvar* fake_alvar = new Alvar("/base_footprint");
    fake_alvar->frame.setZero();

    arr pos = { 0.6, -0.3, 1.05 };
    rndUniform(pos, -0.005, 0.005, true);
    fake_alvar->frame.addRelativeTranslation(pos(0), pos(1), pos(2));

    arr alv_rot = { 0, 0, 0.785 };
    rndUniform(alv_rot, -0.01, 0.01, true);
    rot.setRpy(alv_rot(0), alv_rot(1), alv_rot(2));
    fake_alvar->frame.addRelativeRotation(rot);
    fake_alvar->id = 2;
    percepts.append( fake_alvar );
    mlr::wait(0.01);
  }

  if (percepts.N > 0){
    perceptual_inputs.set() = percepts;
  }
}


Cluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker)
{
  arr points = conv_points2arr(marker.points);
  arr mean = sum(points,0)/(double)points.d0;
  Cluster new_object(mean, points, marker.header.frame_id);

  return new_object;
}

Alvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker)
{
  Alvar new_alvar(marker.header.frame_id);
  new_alvar.id = marker.id;
  new_alvar.transform = conv_pose2transformation(marker.pose.pose);
  return new_alvar;
}

OptitrackMarker conv_tf2OptitrackMarker(const geometry_msgs::TransformStamped& msg)
{
  OptitrackMarker new_optitrackmarker(msg.header.frame_id);
//  new_optitrackmarker.id = marker.id;
  new_optitrackmarker.transform = conv_transform2transformation(msg.transform);
  return new_optitrackmarker;
}

OptitrackBody conv_tf2OptitrackBody(const geometry_msgs::TransformStamped& msg)
{
  OptitrackBody new_optitrackbody(msg.header.frame_id);
//  new_alvar.id = body.id;
  new_optitrackbody.transform = conv_transform2transformation(msg.transform);
  return new_optitrackbody;
}


