#include <RosCom/roscom.h>
#include "perceptionCollection.h"

Collector::Collector()
  : Thread("Collector", 0){}

void Collector::step()
{
  FilterObjects perceps;

  int cluster_rev = tabletop_clusters.readAccess();
  int ar_rev = ar_pose_markers.readAccess();

  if (this->useRos)
  {
    if ( cluster_rev > tabletop_clusters_revision )
    {
      tabletop_clusters_revision = cluster_rev;
      const visualization_msgs::MarkerArray msg = tabletop_clusters();

      if (msg.markers.size() > 0)
      {
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

        for(auto & marker : msg.markers){
          Cluster* new_cluster = new Cluster(conv_ROSMarker2Cluster( marker ));
          new_cluster->frame = tf;
          perceps.append( new_cluster );
        }
      }
    }

    if ( ar_rev > ar_pose_markers_revision)
    {
      ar_pose_markers_revision = ar_rev;
      const ar::AlvarMarkers msg = ar_pose_markers();
      for(auto & marker : msg.markers)
      {
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
        Alvar* new_alvar = new Alvar(conv_ROSAlvar2Alvar( marker ));
        new_alvar->frame = tf;
        perceps.append( new_alvar );
      }
    }
  }
  else // If useRos==0, make a fake cluster and alvar
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
    perceps.append( fake_cluster );

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
    perceps.append( fake_alvar );
    mlr::wait(0.01);
  }
  if (perceps.N > 0)
  {
    perceptual_inputs.writeAccess();
    perceptual_inputs() = perceps;
    perceptual_inputs.deAccess();
  }

  ar_pose_markers.deAccess();
  tabletop_clusters.deAccess();

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

