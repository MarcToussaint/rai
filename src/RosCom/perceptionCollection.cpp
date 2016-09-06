#include <RosCom/roscom.h>
#include "perceptionCollection.h"

Collector::Collector(const bool simulate):
    Module("Collector", simulate ? 0.05:-1),
    tabletop_clusters(this, "tabletop_clusters", !simulate),
    ar_pose_markers(this, "ar_pose_markers", !simulate),
    tabletop_tableArray(this, "tabletop_tableArray", !simulate)
{
  tabletop_srcFrame.set()->setZero();
  alvar_srcFrame.set()->setZero();
  this->simulate = simulate;
//  tf.setZero();
}

void Collector::step()
{
  FilterObjects percepts;

  if (!simulate)
  {

    int cluster_rev = tabletop_clusters.readAccess();
    if ( cluster_rev > tabletop_clusters_revision ){ //only if new cluster info is available
      tabletop_clusters_revision = cluster_rev;
      const visualization_msgs::MarkerArray msg = tabletop_clusters();

      if (msg.markers.size() > 0){
        if (!has_cluster_transform) { //get the transform from ROS

          // Convert into a position relative to the base.
          tf::TransformListener listener;
          try{
            ors::Transformation tf = ros_getTransform("/base_footprint", msg.markers[0].header.frame_id, listener);

            //MT: really add the meter here? This seems hidden magic numbers in the code. And only for Baxter..?
            ors::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            tabletop_srcFrame.set() = inv;
            has_cluster_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }

        for(auto & marker : msg.markers){
          Cluster* new_cluster = new Cluster(conv_ROSMarker2Cluster( marker ));
          new_cluster->frame = tabletop_srcFrame.get();
          percepts.append( new_cluster );
        }
      }
    }
    tabletop_clusters.deAccess();

    int tableArray_rev = tabletop_tableArray.readAccess();
    if ( tableArray_rev > tabletop_tableArray_revision ){ //only if new cluster info is available
      tabletop_tableArray_revision = tableArray_rev;
      const object_recognition_msgs::TableArray msg = tabletop_tableArray();

      if (msg.tables.size() > 0){

        for(auto & table : msg.tables){
          Plane* new_plane = new Plane(conv_ROSTable2Plane( table ));
          new_plane->frame = tabletop_srcFrame.get(); //tf
          percepts.append( new_plane );
        }
      }
    }
    tabletop_tableArray.deAccess();

    int ar_rev = ar_pose_markers.readAccess();
    if ( ar_rev > ar_pose_markers_revision){ //new alwar objects are available
      ar_pose_markers_revision = ar_rev;
      const ar::AlvarMarkers msg = ar_pose_markers();

      for(auto & marker : msg.markers) {
        if (!has_alvar_transform) { //get the transform from ROS

          // Convert into a position relative to the base.
          tf::TransformListener listener;
          try{
            ors::Transformation tf = ros_getTransform("/base_footprint", msg.markers[0].header.frame_id, listener);

            //MT: really add the meter here? This seems hidden magic numbers in the code. And only for Baxter..?
            ors::Transformation inv;
            inv.setInverse(tf);
            inv.addRelativeTranslation(0,0,-1);
            inv.setInverse(inv);
            alvar_srcFrame.set() = inv;
            has_alvar_transform = true;
          }
          catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              exit(0);
          }
        }


        Alvar* new_alvar = new Alvar( conv_ROSAlvar2Alvar(marker) );
        new_alvar->frame = alvar_srcFrame.get();
//        new_alvar->frame.setInverse(alvar_srcFrame.get());
//        new_alvar->frame.addRelativeTranslation(0,0,-1);
//        new_alvar->frame.setInverse(new_alvar->frame);
        percepts.append( new_alvar );
      }
    }
    ar_pose_markers.deAccess(); //MT: make the access shorter, only around the blocks above

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

    arr pos = { 0.9, 0.3, 1.5 };
    rndUniform(pos, -0.005, 0.005, true);
    fake_alvar->frame.addRelativeTranslation(pos(0), pos(1), pos(2));

    arr alv_rot = { MLR_PI/2, 0, -MLR_PI/2};
    rndUniform(alv_rot, -0.01, 0.01, true);
    rot.setRpy(alv_rot(0), alv_rot(1), alv_rot(2));
    fake_alvar->frame.addRelativeRotation(rot);
    fake_alvar->id = 10;
    percepts.append( fake_alvar );
  }

  if (percepts.N > 0){
    perceptual_inputs.set() = percepts;
  }
}


Cluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker){
  arr points = conv_points2arr(marker.points);
  arr mean = sum(points,0)/(double)points.d0;
  return Cluster(mean, points, marker.header.frame_id);
}

Plane conv_ROSTable2Plane(const object_recognition_msgs::Table& table){
  arr hull = conv_points2arr(table.convex_hull);
  ors::Transformation t = conv_pose2transformation(table.pose);
  arr center = t.pos.getArr();
  arr normal = (t * Vector_z).getArr();
  return Plane(normal, center, hull, table.header.frame_id);
}

Alvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker){
  Alvar new_alvar(marker.header.frame_id);
  new_alvar.id = marker.id;
  new_alvar.transform = conv_pose2transformation(marker.pose.pose);
  return new_alvar;
}

