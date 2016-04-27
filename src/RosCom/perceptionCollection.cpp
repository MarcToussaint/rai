#include <RosCom/roscom.h>
#include "perceptionCollection.h"

Collector::Collector():
    Module("Collector", 0){}

void Collector::step()
{
  FilterObjects perceps;

  int tabletop_rev = tabletop_clusters.readAccess();
  int ar_rev = ar_pose_markers.readAccess();

  if ( tabletop_rev > tabletop_revision )
  {
    tabletop_revision = tabletop_rev;
    const visualization_msgs::MarkerArray msg = tabletop_clusters.get();

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
    const ar::AlvarMarkers msg = ar_pose_markers.get();
    for(auto & marker : msg.markers)
    {
      Alvar* new_alvar = new Alvar(conv_ROSAlvar2Alvar( marker ));
      perceps.append( new_alvar );
    }
  }

  if (perceps.N > 0)
  {
    //std::cout << "Collector update" << std::endl;
    perceptual_inputs.set() = perceps;
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

  tf::TransformListener listener;
  tf::StampedTransform baseTransform;
  try{
    listener.waitForTransform("/base_footprint", marker.header.frame_id, ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/base_footprint", marker.header.frame_id, ros::Time(0), baseTransform);
  }
  catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  new_alvar.frame = conv_transform2transformation(baseTransform);

  return new_alvar;
}

