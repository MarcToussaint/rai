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
#include <object_recognition_msgs/TableArray.h>

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
  //ros::init(mlr::argc, mlr::argv, "publish_database", ros::init_options::NoSigintHandler);
  if(mlr::getParameter<bool>("useRos", false))
    nh = new ros::NodeHandle;
  if(nh){
    cluster_pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
    alvar_pub = nh->advertise<ar::AlvarMarkers>("/tracked_ar_pose_marker", 1);
    plane_pub = nh->advertise<object_recognition_msgs::TableArray>("/tracked_table_array", 1);
  }
}

void PublishDatabase::close()
{
  if(nh){
    //  nh->shutdown();
    delete nh;
  }
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

void PublishDatabase::syncCluster(const Cluster* cluster)
{
  modelWorld.writeAccess();
  mlr::String cluster_name = STRING("cluster_" << cluster->id);

  ors::Body *body = modelWorld().getBodyByName(cluster_name, false);
  if (not body) {
    //cout << cluster_name << " does not exist yet; adding it..." << endl;
    body = new ors::Body(modelWorld());
    body->name = cluster_name;
    ors::Shape *shape = new ors::Shape(modelWorld(), *body);
    shape->name = cluster_name;
    shape->type = ors::pointCloudST;
    shape = new ors::Shape(modelWorld(), *body);
    shape->name = cluster_name;
    shape->type = ors::markerST;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
    stored_clusters.append(cluster->id);
  }
  body->X = cluster->frame;
  //cluster->frame = body->X;
  body->shapes(0)->mesh.V = cluster->points;

  ors::Vector cen = body->shapes(0)->mesh.center();
  body->X.addRelativeTranslation(cen);
  body->shapes(0)->rel.rot = body->X.rot;
  body->X.rot.setZero();

  ((Cluster*)cluster)->transform = body->X;
  //((Cluster*)cluster)->mean = ARR(cen.x, cen.y, cen.z);
  /* If we change the mean, we compare the transformed mean to an untransformed mean later...*/
  modelWorld.deAccess();
}

void PublishDatabase::syncAlvar(const Alvar* alvar)
{
  modelWorld.writeAccess();
  mlr::String alvar_name = STRING("alvar_" << alvar->id);

  ors::Body *body = modelWorld().getBodyByName(alvar_name, false);
  if (not body) {
//    cout << alvar_name << " does not exist yet; adding it..." << endl;
    body = new ors::Body(modelWorld());
    body->name = alvar_name;
    ors::Shape *shape = new ors::Shape(modelWorld(), *body);
    shape->name = alvar_name;
    shape->type = ors::markerST;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
    stored_alvars.append(alvar->id);
  }

  body->X = alvar->frame * alvar->transform;

  //((Alvar*)alvar)->transform = body->X;
  modelWorld.deAccess();
}

void PublishDatabase::step()
{
  int rev = object_database.writeAccess();

  if (rev == revision)
  {
    object_database.deAccess();
    return;
  }
  revision = rev;

  FilterObjects objectDatabase = object_database();

  visualization_msgs::MarkerArray cluster_markers;
  ar::AlvarMarkers ar_markers;

  mlr::Array<uint> new_clusters, new_alvars;

  for (uint i = 0; i < objectDatabase.N; i++)
  {
    switch ( objectDatabase(i)->type )
    {
      case FilterObject::FilterObjectType::alvar:
      {
        ar::AlvarMarker alvar = conv_FilterObject2Alvar(*objectDatabase(i));
        ar_markers.markers.push_back(alvar);
        ar_markers.header.frame_id = alvar.header.frame_id;
        syncAlvar(dynamic_cast<Alvar*>(objectDatabase(i)));
        new_alvars.append(objectDatabase(i)->id);
        break;
      }
      case FilterObject::FilterObjectType::cluster:
      {
        visualization_msgs::Marker marker = conv_FilterObject2Marker(*objectDatabase(i));
        cluster_markers.markers.push_back(marker);
        syncCluster(dynamic_cast<Cluster*>(objectDatabase(i)));
        new_clusters.append(objectDatabase(i)->id);
        break;
      }
      default:
      {
        NIY;
        break;
      }
    }
  }
  object_database.deAccess();

  // Publish ROS messages
  if(nh){
    if (cluster_markers.markers.size() > 0)
      cluster_pub.publish(cluster_markers);

    if (ar_markers.markers.size() > 0)
      alvar_pub.publish(ar_markers);
  }


  // Sync the modelWorld
  modelWorld.writeAccess();
  for (uint id : stored_clusters)
  {
    if (new_clusters.contains(id) == 0)
    {
      // Remove ID from the world
      stored_clusters.removeValueSafe(id);
      delete modelWorld().getBodyByName(STRING("cluster_" << id));
    }
  }
  for (uint id : stored_alvars)
  {
    if (new_alvars.contains(id) == 0)
    {
      // Remove ID from the world
      stored_alvars.removeValueSafe(id);
      delete modelWorld().getBodyByName(STRING("alvar_" << id));
    }
  }
  modelWorld.deAccess();
}