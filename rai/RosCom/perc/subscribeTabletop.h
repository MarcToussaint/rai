#include "roscom.h"

struct SubscribeTabletop{
  VARname(visualization_msgs::MarkerArray, tabletop_clusters)
  Subscriber<visualization_msgs::MarkerArray> sub;

  SubscribeTabletop()
    : sub("/tabletop/clusters", tabletop_clusters) {
  }
  ~SubscribeTabletop(){
  }

};
