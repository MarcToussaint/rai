/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "benchmarks.h"
#include "komo.h"
//#include <Optim/kOrderMarkov.h>
#include <Optim/convert.h>

void setTasks(KOMO& MP,
              mlr::Shape &endeff,
              mlr::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

struct sPR2EndPoseProblem{
  mlr::KinematicWorld world;
  KOMO MP;
  Convert *CP;
  sPR2EndPoseProblem()
    :world ("model.kvg"), MP(world), CP(NULL){}
};

PR2EndPoseProblem::PR2EndPoseProblem()
  : s(*(new sPR2EndPoseProblem())){

  for(mlr::Shape *sh:s.world.shapes) sh->cont=true;

  setTasks(s.MP, *s.world.getShapeByName("endeff"), *s.world.getShapeByName("target"), 0, 1, 0, 5.);

  s.CP = new Convert(s.MP.komo_problem);
  ConstrainedProblem::operator=( *s.CP );//conv_KOrderMarkovFunction2ConstrainedProblem(s.MP.komo_problem) );
}

arr PR2EndPoseProblem::getInitialization(){
  arr x = s.MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config
  return x;
}

void PR2EndPoseProblem::report(){
  cout <<s.MP.getReport();
  s.world.watch(true);
}

void PR2EndPoseProblem::setState(const arr& x){
  s.world.setJointState(x);
}

