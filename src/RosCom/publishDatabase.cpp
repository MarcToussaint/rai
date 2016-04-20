/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#include "publishDatabase.h"
#include <geometry_msgs/PoseArray.h>

#ifdef MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#else // Assuming INDIGO or later
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif


PublishDatabase::PublishDatabase():
    Module("PublishDatabase", 0){}

void PublishDatabase::open(){
  ros::init(mlr::argc, mlr::argv, "publish_database", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
  tabletop_pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
  alvar_pub = nh->advertise<geometry_msgs::PoseArray>("/tracked_ar_pose_marker", 1);
}

void PublishDatabase::close()
{
  nh->shutdown();
  delete nh;
}

visualization_msgs::Marker conv_FilterObject2Marker(const FilterObject& object)
{
  visualization_msgs::Marker new_marker;
  new_marker.type = visualization_msgs::Marker::POINTS;
  new_marker.points = conv_arr2points( dynamic_cast<const Cluster&>(object).points);
  new_marker.id = object.id;
  new_marker.scale.x = .001;
  new_marker.scale.y = .001;
  new_marker.lifetime = ros::Duration(0.5);
  new_marker.header.stamp = ros::Time(0.);
  new_marker.header.frame_id = dynamic_cast<const Cluster&>(object).frame_id;

  new_marker.color.a = object.relevance;
  new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
  new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
  new_marker.color.b = (double)((new_marker.id*10000)%89)/89;

  return new_marker;
}

ar::AlvarMarker conv_FilterObject2Alvar(const FilterObject& object)
{
  ar::AlvarMarker new_marker;
  new_marker.header.frame_id = dynamic_cast<const Alvar&>(object).frame_id;
  new_marker.pose.pose = conv_transformation2pose(object.transform);
  new_marker.id = object.id;
  return new_marker;
}

geometry_msgs::Pose conv_FilterObject2AlvarVis(const FilterObject& object)
{
  geometry_msgs::Pose new_marker;
  new_marker = conv_transformation2pose(object.transform);
  return new_marker;
}


void PublishDatabase::step()
{
  object_database.waitForNextRevision();

  object_database.readAccess();
  FilterObjects objectDatabase = object_database.get();


  visualization_msgs::MarkerArray cluster_markers;
  ar::AlvarMarkers ar_markers;

  int alvar_count = 0, cluster_count = 0;

  for (uint i = 0; i < objectDatabase.N; i++)
  {
    switch ( objectDatabase(i)->type )
    {
      case FilterObject::FilterObjectType::alvar:
        alvar_count++;
        ar_markers.markers.push_back(conv_FilterObject2Alvar(*objectDatabase(i)));
        ar_markers.header.frame_id = dynamic_cast<Alvar*>(objectDatabase(i))->frame_id;
        break;
      case FilterObject::FilterObjectType::cluster:
        cluster_count++;
        cluster_markers.markers.push_back(conv_FilterObject2Marker(*objectDatabase(i)));
        break;
      default:
        NIY;
        break;
    }
  }
  object_database.deAccess();

  if (cluster_markers.markers.size() > 0)
    tabletop_pub.publish(cluster_markers);

  if (ar_markers.markers.size() > 0)
    alvar_pub.publish(ar_markers);
}
