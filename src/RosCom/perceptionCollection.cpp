#include <RosCom/roscom.h>
#include <RosCom/perceptionCollection.h>

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
    for(auto & marker : msg.markers){
      perceps.append(conv_Marker2FilterObject( marker ));
    }
  }

  if ( ar_rev > ar_pose_markers_revision)
  {
    ar_pose_markers_revision = ar_rev;
    const ar::AlvarMarkers msg = ar_pose_markers.get();
    for(auto & marker : msg.markers)
    {
      perceps.append(conv_Alvar2FilterObject( marker ) );
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


FilterObject conv_Marker2FilterObject(const visualization_msgs::Marker& marker)
{
  arr pts = conv_points2arr(marker.points);
  arr mean = sum(pts,0)/(double)pts.d0;
  // Put it into our list

  FilterObject new_object;
  new_object.Cluster::mean = mean;
  //std::cout << "Mean: " << mean(0) << ' ' << mean(1) << ' ' << mean(2) << std::endl;

  new_object.Cluster::points = pts;
  new_object.id = -1;
  new_object.relevance = 1;
  new_object.Cluster::frame_id = marker.header.frame_id;
  new_object.type = FilterObject::FilterObjectType::cluster;
  return new_object;
}

FilterObject conv_Alvar2FilterObject(const ar::AlvarMarker& marker)
{
  FilterObject new_object;
  new_object.id = marker.id;
  new_object.Alvar::frame_id = marker.header.frame_id;
  new_object.relevance = 1;
  new_object.type = FilterObject::FilterObjectType::alvar;

  new_object.quaternion.x = marker.pose.pose.orientation.x;
  new_object.quaternion.y = marker.pose.pose.orientation.y;
  new_object.quaternion.z = marker.pose.pose.orientation.z;
  new_object.quaternion.w = marker.pose.pose.orientation.w;

#if 0
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
  tf::Vector3 position(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z);
  position = baseTransform * position;

  new_object.position = {position.x, position.y, position.z};

#else
  new_object.position = {marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z};
#endif

  return new_object;
}

