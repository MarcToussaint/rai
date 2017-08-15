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


#include "taskMap_QuaternionNorms.h"
#include "taskMap_qItself.h"
#include "frame.h"

void TaskMap_QuaternionNorms::phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t){
    uint n=dim_phi(G);
    y.resize(n);
    if(&J) J.resize(n, G.q.N).setZero();
    uint i=0;
    mlr::Joint *j;
    for(const mlr::Frame* f: G.frames) if((j=f->joint()) && (j->type==mlr::JT_quatBall || j->type==mlr::JT_free || j->type==mlr::JT_XBall)){
        arr q;
        if(j->type==mlr::JT_quatBall) q.referToRange(G.q, j->qIndex+0, j->qIndex+3);
        if(j->type==mlr::JT_XBall)    q.referToRange(G.q, j->qIndex+1, j->qIndex+4);
        if(j->type==mlr::JT_free)     q.referToRange(G.q, j->qIndex+3, j->qIndex+6);
        double norm = sumOfSqr(q);
        y(i) = norm - 1.;

        if(&J){
            if(j->type==mlr::JT_quatBall) J(i, {j->qIndex+0, j->qIndex+3}) = 2.*q;
            if(j->type==mlr::JT_XBall)    J(i, {j->qIndex+1, j->qIndex+4}) = 2.*q;
            if(j->type==mlr::JT_free)     J(i, {j->qIndex+3, j->qIndex+6}) = 2.*q;
        }
        i++;
    }
}

uint TaskMap_QuaternionNorms::dim_phi(const mlr::KinematicWorld &G){
    uint i=0;
    const mlr::Joint *j;
    for(const mlr::Frame* f:G.frames) if((j=f->joint())){
        if(j->type==mlr::JT_quatBall || j->type==mlr::JT_free || j->type==mlr::JT_XBall) i++;
    }
    return i;
}
