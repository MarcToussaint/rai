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


#pragma once

#include <Core/array.h>

void plotQhullState(uint D);
extern int QHULL_DEBUG_LEVEL;
const char* qhullVersion();
void qHullSave();

double distanceToConvexHull(const arr &X,        //points
                            const arr &y,        //query point
                            arr *projectedPoint, //return query point projected on closest facet
                            uintA *faceVertices, //return indices of vertices of closest facet
                            bool freeqhull);     //free allocated qhull engine after request [true]

double distanceToConvexHullGradient(arr& dDdX,       //gradient (or same dim as X)
                                    const arr &X,    //points
                                    const arr &y,    //query point
                                    bool freeqhull); //free allocated qhull engine after request [true]

double forceClosure(const arr& X,  //contact points (size Nx3)
                    const arr& Xn, //contact normals (size Nx3)
                    const mlr::Vector& center, //object center
                    double mu=.5,     //friction coefficient
                    double discountTorques=1.,   //friction coefficient
                    arr *dFdX=NULL);    //optional: also compute gradient


arr getHull(const arr& V, uintA& T=NoUintA);

void getDelaunayEdges(uintA& E, const arr& V);
