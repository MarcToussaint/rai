/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "spinner.h"

#ifdef RAI_ROS

#include "roscom.h"

RosCom_Spinner::RosCom_Spinner(const char* nodeName) : Thread("RosCom_Spinner", .001) {
  useRos = rai::getParameter<bool>("useRos", true);
  if(useRos) {
    rosCheckInit(nodeName);
    threadLoop();
  }
}

RosCom_Spinner::~RosCom_Spinner() {
  if(useRos) threadClose();
}

void RosCom_Spinner::step() {
  if(useRos) ros::spinOnce();
}

#else

RosCom_Spinner::RosCom_Spinner(const char* nodeName) : Thread("RosCom_Spinner", -1) {}
RosCom_Spinner::~RosCom_Spinner() {}

void RosCom_Spinner::step() {}

#endif
