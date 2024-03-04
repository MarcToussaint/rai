/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "geo.h"

void plotQhullState(uint D);
extern int QHULL_DEBUG_LEVEL;
const char* qhullVersion();
void qHullSave();
void qhull_free();

double distanceToConvexHull(const arr& X,        //points
                            const arr& y,        //query point
                            arr& distances=NoArr,
                            arr& projectedPoints=NoArr, //return query point projected on closest facet
                            uintA* faceVertices=nullptr, //return indices of vertices of closest facet
                            bool freeqhull=true);     //free allocated qhull engine after request [true]

double distanceToConvexHullGradient(arr& dDdX,       //gradient (or same dim as X)
                                    const arr& X,    //points
                                    const arr& y,    //query point
                                    bool freeqhull=true); //free allocated qhull engine after request [true]

double forceClosure(const arr& X,  //contact points (size Nx3)
                    const arr& Xn, //contact normals (size Nx3)
                    const rai::Vector& center, //object center
                    double mu=.5,     //friction coefficient
                    double discountTorques=1.,   //friction coefficient
                    arr* dFdX=nullptr);    //optional: also compute gradient

arr getHull(const arr& V, uintA& T=uintA().setNoArr());

void getDelaunayEdges(uintA& E, const arr& V);

void pullPointsIntoHull(arr& P, const arr& X);

arr convconv_intersect(const arr& A, const arr& B);
