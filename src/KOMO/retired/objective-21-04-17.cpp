/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void Objective::setCostSpecs(int fromStep, int toStep, bool tuples) {
  CHECK_GE(fromStep, 0, "");
//  CHECK_GE(toStep, fromStep, "");
  if(!tuples) {
    if(toStep>=fromStep)
      configs.resize(toStep+1).setZero();
    else configs.clear();
    for(int t=fromStep; t<=toStep; t++) configs(t) = 1;
  } else {
    if(toStep>=fromStep)
      configs.resize(1+toStep-fromStep, feat->order+1);
    else configs.resize(0, feat->order+1);
    for(int t=fromStep; t<=toStep; t++)
      for(uint j=0; j<configs.d1; j++) configs(t-fromStep, j) = t+j-int(feat->order);
  }
}

void Objective::setCostSpecs(const arr& times, int stepsPerPhase, uint T,
                             int deltaFromStep, int deltaToStep, bool tuples) {

  double fromTime=0, toTime=-1.;
  if(!times.N) {
  } else if(times.N==1) {
    fromTime = toTime = times(0);
  } else {
    CHECK_EQ(times.N, 2, "");
    fromTime = times(0);
    toTime = times(1);
  }

  if(toTime>double(T)/stepsPerPhase+1.) {
    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  }

  CHECK_GE(stepsPerPhase, 0, "");

  int fromStep = (fromTime<0.?0:conv_time2step(fromTime, stepsPerPhase));
  int toStep   = (toTime<0.?T-1:conv_time2step(toTime, stepsPerPhase));

  if(fromTime>=0 && deltaFromStep) fromStep+=deltaFromStep;
  if(toTime>=0 && deltaToStep) toStep+=deltaToStep;

  if(fromStep<0) fromStep=0;
//  if(toStep<0) toStep=0;
  if(toStep>=(int)T && T>0) toStep=T-1;

  setCostSpecs(fromStep, toStep, tuples);
}
