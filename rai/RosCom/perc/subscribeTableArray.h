#include "roscom.h"

#include <object_recognition_msgs/TableArray.h>

struct SubscribeTableArray{
  VARname(object_recognition_msgs::TableArray, tabletop_tableArray)
  Subscriber<object_recognition_msgs::TableArray> sub;

  SubscribeTableArray()
    : sub("/table_array", tabletop_tableArray) {
  }
  ~SubscribeTableArray(){
  }

};
