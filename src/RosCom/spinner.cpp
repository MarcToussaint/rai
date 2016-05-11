#include "spinner.h"

#ifdef MLR_ROS

#include "roscom.h"

RosCom_Spinner::RosCom_Spinner(const char* nodeName):Module("RosCom_Spinner", .001){
  useRos = mlr::getParameter<bool>("useRos", false);
  if(useRos) rosCheckInit(nodeName);
}

void RosCom_Spinner::step(){ if(useRos) ros::spinOnce(); }

#else

RosCom_Spinner::RosCom_Spinner(const char* nodeName):Module("RosCom_Spinner", -1){}

void RosCom_Spinner::step(){  }

#endif
