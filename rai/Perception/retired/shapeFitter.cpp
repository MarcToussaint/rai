/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "shapeFitter.h"
#include "../Optim/optimization.h"
//#include <RAI/vision.h>
#include "../Kin/kin.h"
//#include <RAI/calibration.h>

#if 0

struct ShapeFitter: Process {
  FloatImage* eviL, *eviR;
  PerceptionOutput* percOut;

  uintA objectType;
  arr Pl, Pr;
  rai::Array<RigidObjectRepresentation> objs;

  ShapeFitter(FloatImage& _eviL, FloatImage& _eviR, PerceptionOutput& _perc): Process("ShapeFitter"), eviL(&_eviL), eviR(&_eviR), percOut(&_perc) {}
  void open() {
    ifstream fil;
    rai::open(fil, "../../data/configurations/calib_P");
    Pl.readTagged(fil, "PL");
    Pr.readTagged(fil, "PR");
    fil.close();
    objectType = biros().getParameter<uintA>("percObjectType", this);
  }
  void step();
  void close() {}
};

//===========================================================================
//
// fwd declared helper routines
//

void generateShapePoints(arr& points, arr& weights, arr* grad, uint type, uint N, const arr& params);

//===========================================================================
//
// implementation of helper routines
//

void generateShapePoints(arr& points, arr& weights, arr* grad, uint type, uint N, const arr& params) {
  if(type==0) { //cirlce
    CHECK_EQ(params.N, 3, ""); //posx posy radius
    points.resize(N, 2);
    weights.resize(N); weights=1.;
    if(grad) { grad->resize(N, 2, 3);  grad->setZero(); }
    for(uint i=0; i<N; i++) {
      points(i, 0) = params(0) + cos(RAI_2PI/N*i)*params(2);  if(grad) {(*grad)(i, 0, 0) = 1.; (*grad)(i, 0, 2) = cos(RAI_2PI/N*i); }
      points(i, 1) = params(1) + sin(RAI_2PI/N*i)*params(2);  if(grad) {(*grad)(i, 1, 1) = 1.; (*grad)(i, 1, 2) = sin(RAI_2PI/N*i); }
    }
    return;
  }
  if(type==1) { //cylinder
    CHECK_EQ(params.N, 5, "cylinder needs 5 params"); //posx posy diameter height curve-height
    points.resize(4*N, 2);
    weights.resize(4*N);
    if(grad) { grad->resize(4*N, 2, 5);  grad->setZero(); }
    //left bar
    for(uint i=0; i<N; i++) {
      points(i, 0) = params(0) - .5*params(2);                      if(grad) {(*grad)(i, 0, 0) = 1.; (*grad)(i, 0, 2) =  -.5; }
      points(i, 1) = params(1) - .5*params(3) + params(3)*i/(N-1);  if(grad) {(*grad)(i, 1, 1) = 1.; (*grad)(i, 1, 3) =  -.5+double(i)/(N-1); }
      weights(i) = params(3);
    }
    //right bar
    for(uint i=0; i<N; i++) {
      points(N+i, 0) = params(0) + .5*params(2);                      if(grad) {(*grad)(N+i, 0, 0) = 1.; (*grad)(N+i, 0, 2) =  +.5; }
      points(N+i, 1) = params(1) - .5*params(3) + params(3)*i/(N-1);  if(grad) {(*grad)(N+i, 1, 1) = 1.; (*grad)(N+i, 1, 3) =  -.5+double(i)/(N-1); }
      weights(N+i) = params(3);
    }
    //top curve
    for(uint i=0; i<N; i++) {
      points(2*N+i, 0) = params(0) + cos(RAI_PI*(i+1)/(N+1))*.5*params(2);              if(grad) {(*grad)(2*N+i, 0, 0) = 1.; (*grad)(2*N+i, 0, 2) = cos(RAI_PI*(i+1)/(N+1))*.5; }
      points(2*N+i, 1) = params(1) + .5*params(3) + sin(RAI_PI*(i+1)/(N+1))*params(4);  if(grad) {(*grad)(2*N+i, 1, 1) = 1.; (*grad)(2*N+i, 1, 3) = +.5; (*grad)(2*N+i, 1, 4) = +sin(RAI_PI*(i+1)/(N+1)); }
      weights(2*N+i) = params(2); //RAI_PI * sqrt(*params(2)/8. + params(4)*params(4)/2.);
    }
    //bottom curve
    for(uint i=0; i<N; i++) {
      points(3*N+i, 0) = params(0) + cos(RAI_PI*(i+1)/(N+1))*.5*params(2);              if(grad) {(*grad)(3*N+i, 0, 0) = 1.; (*grad)(3*N+i, 0, 2) = cos(RAI_PI*(i+1)/(N+1))*.5; }
      points(3*N+i, 1) = params(1) - .5*params(3) - sin(RAI_PI*(i+1)/(N+1))*params(4);  if(grad) {(*grad)(3*N+i, 1, 1) = 1.; (*grad)(3*N+i, 1, 3) = -.5; (*grad)(3*N+i, 1, 4) = -sin(RAI_PI*(i+1)/(N+1)); }
      weights(3*N+i) = params(2);
    }
    return;
  }
  if(type==7) { //capped cylinder
    CHECK_EQ(params.N, 5, "cylinder needs 5 params"); //posx posy diameter height curve-height
    points.resize(4*N, 2);
    weights.resize(4*N);
    if(grad) { grad->resize(4*N, 2, 5);  grad->setZero(); }
    //left bar
    for(uint i=0; i<N; i++) {
      points(i, 0) = params(0) - .5*params(2);                      if(grad) {(*grad)(i, 0, 0) = 1.; (*grad)(i, 0, 2) =  -.5; }
      points(i, 1) = params(1) - .5*params(3) + params(3)*i/(N-1);  if(grad) {(*grad)(i, 1, 1) = 1.; (*grad)(i, 1, 3) =  -.5+double(i)/(N-1); }
      weights(i) = params(3);
    }
    //right bar
    for(uint i=0; i<N; i++) {
      points(N+i, 0) = params(0) + .5*params(2);                      if(grad) {(*grad)(N+i, 0, 0) = 1.; (*grad)(N+i, 0, 2) =  +.5; }
      points(N+i, 1) = params(1) - .5*params(3) + params(3)*i/(N-1);  if(grad) {(*grad)(N+i, 1, 1) = 1.; (*grad)(N+i, 1, 3) =  -.5+double(i)/(N-1); }
      weights(N+i) = params(3);
    }
    //top curve
    for(uint i=0; i<N; i++) {
      points(2*N+i, 0) = params(0) + cos(RAI_PI*(i+1)/(N+1))*.5*params(2);              if(grad) {(*grad)(2*N+i, 0, 0) = 1.; (*grad)(2*N+i, 0, 2) = cos(RAI_PI*(i+1)/(N+1))*.5; }
      points(2*N+i, 1) = params(1) + .5*params(3) - sin(RAI_PI*(i+1)/(N+1))*params(4);  if(grad) {(*grad)(2*N+i, 1, 1) = 1.; (*grad)(2*N+i, 1, 3) = +.5; (*grad)(2*N+i, 1, 4) = -sin(RAI_PI*(i+1)/(N+1)); }
      weights(2*N+i) = params(2); //RAI_PI * sqrt(params(2)*params(2)/8. + params(4)*params(4)/2.);
    }
    //bottom curve
    for(uint i=0; i<N; i++) {
      points(3*N+i, 0) = params(0) + cos(RAI_PI*(i+1)/(N+1))*.5*params(2);              if(grad) {(*grad)(3*N+i, 0, 0) = 1.; (*grad)(3*N+i, 0, 2) = cos(RAI_PI*(i+1)/(N+1))*.5; }
      points(3*N+i, 1) = params(1) - .5*params(3) - sin(RAI_PI*(i+1)/(N+1))*params(4);  if(grad) {(*grad)(3*N+i, 1, 1) = 1.; (*grad)(3*N+i, 1, 3) = -.5; (*grad)(3*N+i, 1, 4) = -sin(RAI_PI*(i+1)/(N+1)); }
      weights(3*N+i) = params(2);
    }
    return;
  }
#if 0
  if(type==2) { //box
    CHECK_EQ(params.N, 6, "box needs 6 params"); //posx posy width height dx dy
    points.resize(4*N, 2);
    weights.resize(4*N);
    if(grad) { grad->resize(4*N, 2, 6);  grad->setZero(); }
    //if(params(4)<0.){ params(4) *= -1.; params(5) *= -1.; }
    //vertical bars
    for(uint i=0; i<N; i++) {
      points(i, 0) = params(0) - .5*params(2) - .5*rai::sign(params(4))*params(4);                      if(grad) {(*grad)(i, 0, 0)=1.; (*grad)(i, 0, 2)=-.5; (*grad)(i, 0, 4)=-.5*rai::sign(params(4)); }
      points(i, 1) = params(1) - .5*params(3) - .5*rai::sign(params(4))*params(5) + params(3)*i/(N-1);  if(grad) {(*grad)(i, 1, 1)=1.; (*grad)(i, 1, 3)=-.5+double(i)/(N-1); (*grad)(i, 1, 5)=-.5*rai::sign(params(4));  }
      weights(i) = params(3);
    }
    for(uint i=0; i<N; i++) {
      points(N+i, 0) = params(0) + .5*params(2) + .5*rai::sign(params(4))*params(4);                      if(grad) {(*grad)(N+i, 0, 0)=1.; (*grad)(N+i, 0, 2)=+.5; (*grad)(N+i, 0, 4)=+.5*rai::sign(params(4)); }
      points(N+i, 1) = params(1) - .5*params(3) + .5*rai::sign(params(4))*params(5) + params(3)*i/(N-1);  if(grad) {(*grad)(N+i, 1, 1)=1.; (*grad)(N+i, 1, 3)=-.5+double(i)/(N-1); (*grad)(N+i, 1, 5)=+.5*rai::sign(params(4)); }
      weights(N+i) = params(3);
    }
    //horizontal bars
    for(uint i=0; i<N; i++) {
      points(2*N+i, 0) = params(0) - .5*params(2) - .5*rai::sign(params(5))*params(4) + params(2)*i/(N-1);   if(grad) {(*grad)(2*N+i, 0, 0)=1.; (*grad)(2*N+i, 0, 2)=-.5+double(i)/(N-1); (*grad)(2*N+i, 0, 4)=-.5*rai::sign(params(5));  }
      points(2*N+i, 1) = params(1) - .5*params(3) - .5*rai::sign(params(5))*params(5);                       if(grad) {(*grad)(2*N+i, 1, 1)=1.; (*grad)(2*N+i, 1, 3)=-.5; (*grad)(2*N+i, 1, 5)=-.5*rai::sign(params(5)); }
      weights(2*N+i) = params(3);
    }
    for(uint i=0; i<N; i++) {
      points(3*N+i, 0) = params(0) - .5*params(2) + .5*rai::sign(params(5))*params(4) + params(2)*i/(N-1);   if(grad) {(*grad)(3*N+i, 0, 0)=1.; (*grad)(3*N+i, 0, 2)=-.5+double(i)/(N-1); (*grad)(3*N+i, 0, 4)=+.5*rai::sign(params(5));  }
      points(3*N+i, 1) = params(1) + .5*params(3) + .5*rai::sign(params(5))*params(5);                       if(grad) {(*grad)(3*N+i, 1, 1)=1.; (*grad)(3*N+i, 1, 3)=+.5; (*grad)(3*N+i, 1, 5)=+.5*rai::sign(params(5)); }
      weights(3*N+i) = params(3);
    }
    return;
  }
#endif
  if(type==2) { //box
    CHECK_EQ(params.N, 8, "box needs 8 params"); //posx posy dx1 dy1 dx2 dy2 dx3 dy3
    uint K=6, k;
    points.resize(K*N, 2);
    weights.resize(K*N);
    if(grad) { grad->resize(K*N, 2, 8);  grad->setZero(); }
    //base -> 1
    k=0;
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(2)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=1.; (*grad)(k*N+i, 0, 2)=double(i)/(N-1); }
      points(k*N+i, 1) = params(1) + params(3)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=1.; (*grad)(k*N+i, 1, 3)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(2)*params(2)+params(3)*params(3));
    }
    //base -> 1 -> 2
    k=1;
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(2) + params(4)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=(*grad)(k*N+i, 0, 2)=1.; (*grad)(k*N+i, 0, 4)=double(i)/(N-1);  }
      points(k*N+i, 1) = params(1) + params(3) + params(5)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=(*grad)(k*N+i, 1, 3)=1.; (*grad)(k*N+i, 1, 5)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(4)*params(4)+params(5)*params(5));
    }
    //base -> 1 -> 2 -> 3
    k=2;
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(2) + params(4) + params(6)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=(*grad)(k*N+i, 0, 2)=(*grad)(k*N+i, 0, 4)=1.; (*grad)(k*N+i, 0, 6)=double(i)/(N-1); }
      points(k*N+i, 1) = params(1) + params(3) + params(5) + params(7)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=(*grad)(k*N+i, 1, 3)=(*grad)(k*N+i, 1, 5)=1.; (*grad)(k*N+i, 1, 7)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(6)*params(6)+params(7)*params(7));
    }
    k=3;
    //base -> 3
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(6)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=1.; (*grad)(k*N+i, 0, 6)=double(i)/(N-1); }
      points(k*N+i, 1) = params(1) + params(7)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=1.; (*grad)(k*N+i, 1, 7)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(6)*params(6)+params(7)*params(7));
    }
    k=4;
    //base -> 3 -> 2
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(6) + params(4)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=(*grad)(k*N+i, 0, 6)=1.; (*grad)(k*N+i, 0, 4)=double(i)/(N-1); }
      points(k*N+i, 1) = params(1) + params(7) + params(5)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=(*grad)(k*N+i, 1, 7)=1.; (*grad)(k*N+i, 1, 5)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(4)*params(4)+params(5)*params(5));
    }
    //base -> 3 -> 2 -> 1
    k=5;
    for(uint i=0; i<N; i++) {
      points(k*N+i, 0) = params(0) + params(6) + params(4) + params(2)*i/(N-1);  if(grad) {(*grad)(k*N+i, 0, 0)=(*grad)(k*N+i, 0, 4)=(*grad)(k*N+i, 0, 6)=1.; (*grad)(k*N+i, 0, 2)=double(i)/(N-1); }
      points(k*N+i, 1) = params(1) + params(7) + params(5) + params(3)*i/(N-1);  if(grad) {(*grad)(k*N+i, 1, 1)=(*grad)(k*N+i, 1, 5)=(*grad)(k*N+i, 1, 7)=1.; (*grad)(k*N+i, 1, 3)=double(i)/(N-1); }
      weights(k*N+i)  = sqrt(params(2)*params(2)+params(3)*params(3));
    }
    return;
  }
  if(type==3) { //6-polygon
    CHECK_EQ(params.N, 12, "6-plygon needs 12 params"); //posx posy dx1 dy1 dx2 dy2 dx3 dy3
    uint K=6, k, kp;
    points.resize(K*N, 2);
    weights.resize(K*N);
    if(grad) { grad->resize(K*N, 2, 12);  grad->setZero(); }
    for(k=0; k<6; k++) {
      kp=k+1; kp=kp%6;
      for(uint i=0; i<N; i++) {
        double a=double(i)/(N-1);
        points(k*N+i, 0) = (1.-a)*params(2*k+0) + a*params(2*kp+0);  if(grad) {(*grad)(k*N+i, 0, 2*k+0)=1.-a; (*grad)(k*N+i, 0, 2*kp+0)=a; }
        points(k*N+i, 1) = (1.-a)*params(2*k+1) + a*params(2*kp+1);  if(grad) {(*grad)(k*N+i, 1, 2*k+1)=1.-a; (*grad)(k*N+i, 1, 2*kp+1)=a; }
        weights(k*N+i)  = sqrt(rai::sqr(params(2*k+0)-params(2*kp+0))+rai::sqr(params(2*k+1)-params(2*kp+1)));
      }
    }
    return;
  }
  HALT("don't know that shape type");
};

struct ShapeFitProblem:public ScalarFunction {
  floatA distImage;
  uint type, N;
  arr x, points;
  bool display;
  double radius; //andreas: dirty radius hack

  double fs(arr& grad, arr& H, const arr& x) {
    double cost=0.;
    arr weights, dfdpoints;
    generateShapePoints(points, weights, &grad, type, N, x);
    if(!!grad) { dfdpoints.resizeAs(points);  dfdpoints.setZero();  }
    if(!!H) NIY;
    for(uint i=0; i<points.d0; i++) { //interpolate...
      uint x=points(i, 0), y=points(i, 1);
      if(x>=distImage.d1-1) x=distImage.d1-2;
      if(y>=distImage.d0-1) y=distImage.d0-2;
      double a=fmod(points(i, 0), 1.), b=fmod(points(i, 1), 1.);
      if(a+b<1.) {
        cost += weights(i)*((1.-a-b)*distImage(y, x) + a*distImage(y, x+1) + b*distImage(y+1, x));
        if(!!grad) {
          dfdpoints(i, 0) = weights(i)*(distImage(y, x+1) - distImage(y, x));
          dfdpoints(i, 1) = weights(i)*(distImage(y+1, x) - distImage(y, x));
        }
      } else {
        cost += weights(i)*((a+b-1.)*distImage(y+1, x+1) + (1.-a)*distImage(y+1, x) + (1.-b)*distImage(y, x+1));
        if(!!grad) {
          dfdpoints(i, 0) = weights(i)*(distImage(y+1, x+1) - distImage(y+1, x));
          dfdpoints(i, 1) = weights(i)*(distImage(y+1, x+1) - distImage(y, x+1));
        }
      }
    }
    if(!!grad) {
      dfdpoints.reshape(dfdpoints.N);
      grad.reshape(dfdpoints.N, x.N);
      grad = ~dfdpoints*grad;
    }
    if(display) {
      byteA img;
      copy(img, distImage);
      cvDrawPoints(img, points);
      //if(grad) cout <<*grad <<endl;
      cvShow(img, "shape optimization", false);
    }
    return cost;
  }
};

bool
pmPreprocessImage(floatA& img, int iterations=5) {

//  CvMatDonor cvMatDonor;
  //CV_MOP_OPEN(iterations) equals to erode(iterations);dilate(iterations)
  //cvSmooth(CVMAT(img), CVMAT(img), CV_MEDIAN);
  //cvMorphologyEx(CVMAT(img), CVMAT(img), nullptr, nullptr, CV_MOP_OPEN, iterations);

  return true;
}

#ifdef RAI_OPENCV
bool getShapeParamsFromEvidence(arr& params, arr& points,
                                const uint& type, const floatA& theta,
                                byteA* disp=nullptr, bool reuseParams=false) {
  ENABLE_CVMAT
  if(disp) {
    *disp=evi2rgb(theta);
    //cvShow(*disp, "getShapeParamsFromEvidence", false);
    //write_ppm(*disp, "earlyvision.ppm");
  }

  //----------------------------------------------------------
  //start experimental stuff from AO

  floatA img = theta;

  pmPreprocessImage(img, 5);

  //search for the first contour, if the dimensions match the type, return.
  //otherwise, delete contour, and continue. If a contour is not bright enough,
  //return with false from function

  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = 0;
  CvConnectedComp component;

  bool found = false;

  while(!found) {
    //flood fill tolerance (upper and lower threshold with respect to
    //start pixel)
    float tolerance = 0.5f;

    //search for maximum value in current image
    uint peakIndex = img.maxIndex();
    float peak = img.elem(peakIndex);

    //printf("max value: %d: %f\n", peakIndex, peak);
    if(peak < 0.5) return false;

    byteA mask(img.d0+2, img.d1+2);
    mask.setZero();

    uint img_width = img.d1;
    CvPoint peak_cvPoint;

    //linear allocated array
    peak_cvPoint.x = peakIndex%img_width;
    peak_cvPoint.y = peakIndex/img_width;

    //printf("%d %d peak\n", peak_cvPoint.x, peak_cvPoint.y);

    cvFloodFill(CVMAT(img),
                peak_cvPoint,
                cvScalar(1), //ignored if mask_only is set
                cvScalar(tolerance), //tolerance up
                cvScalar(tolerance), //tolerance down
                &component,
                CV_FLOODFILL_FIXED_RANGE|CV_FLOODFILL_MASK_ONLY,
                CVMAT(mask));

    //cvShow(byte(255)*mask, "mask");
    cvFindContours(CVMAT(mask),
                   storage,
                   &contour,
                   sizeof(CvContour),
                   CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE,
                   cvPoint(-1, -1));

    CvRect bRect = cvBoundingRect(contour, 1); //CvContour*, bool update

    //currently, we only discriminate between sphere and cylinder
    //and use the width and height of a bounding box

    //\TODO: use shape fitting problem below to determine the most likely
    // type of shape (cyl,sphere,box)

    switch(type) {
      case 0: { //sphere
        if(fabs(bRect.width-bRect.height) < 4 || bRect.width > bRect.height) {
          //printf("detected sphere! %dx%d\n", bRect.width, bRect.height);
          found=true;
        } else {
          //printf("NO SPHERE: %dx%d", bRect.width, bRect.height);
        }
        break;
      }
      case 1: { //cylinder
        if(bRect.width < bRect.height) {
          found=true;
        }
        break;
      }
      case 7: { //capped cylinder
        if(bRect.width < bRect.height) {
          found=true;
        }
        break;
      }
      default:
        printf("perceptionModule can at the moment only discriminate "
               "between cylinder(type 1) and sphere(type 0)\n");
        NIY;
        break;
    }

    //draw a black filled rectangle on top of the found contour
    //this will remove the peak values for the next iteration
    cvRectangle(CVMAT(img),
                cvPoint(bRect.x, bRect.y),
                cvPoint(bRect.x+bRect.width, bRect.y+bRect.height),
                CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
    //color, line thickness, line type, shift
    if(!found) {
      cvClearSeq(contour);
    }

  }
  byteA contourImage; resizeAs(contourImage, theta); contourImage.setZero(255);

  cvDrawContours(CVMAT(contourImage),
                 contour,
                 cvScalar(0.f),
                 cvScalar(128.f), 0);
  if(disp) {
    cvDrawContours(CVMAT(*disp),
                   contour,
                   cvScalar(255, 100, 100),
                   cvScalar(200, 100, 100), 0);
    //cvShow(*disp, "getShapeParamsFromEvidence", false);
    //cvShow(*s, "getShapeParamsFromEvidence", false);
  }
  cvClearMemStorage(storage);
  cvClearSeq(contour);
  //------------------------------------------------------------

  /*
  //write_ppm(camera.output->rgbL,"left.ppm");
  #if 1
  //flood fill
  uint W=theta.d1;
  uint i=theta.maxIndex(); float max1 = theta.elem(i);
  if(max1 < 0.6) return false;//assume no detection when unsure....
  CvConnectedComp component;
  byteA mask(theta.d0+2, theta.d1+2); mask.setZero();
  cvFloodFill(CVMAT(theta), cvPoint(i%W, i/W), cvScalar(1),
              cvScalar(0.8f), cvScalar(0.8f),
              &component, CV_FLOODFILL_FIXED_RANGE|CV_FLOODFILL_MASK_ONLY, CVMAT(mask)); //4th and 5th parameter are flood tolerance, 0.3 originally
  #else
  //threshold
  uint maxi=theta.maxIndex(); float max1 = theta.elem(maxi); if(max1 < 0.6) return;//assume no detection when unsure....
  //uint W=theta.d1;
  byteA mask(theta.d0, theta.d1);
  cvThreshold(CVMAT(theta), CVMAT(mask), max1-.3f, 1.f, CV_THRESH_BINARY);
  #endif

  //cvShow(byte(255)*mask, "mask", true);

  //draw a contour image
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = 0;
  cvFindContours(CVMAT(mask), storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(-1, -1));
  byteA contourImage; resizeAs(contourImage, theta); contourImage.setZero(255);
  cvDrawContours(CVMAT(contourImage), contour, cvScalar(0.f), cvScalar(128.f), 0);
  if(disp){
    cvDrawContours(CVMAT(*disp), contour, cvScalar(255, 100, 100), cvScalar(200, 100, 100), 0);
    //cvShow(*s, "getShapeParamsFromEvidence", false);
  }
  cvClearMemStorage(storage);
  cvClearSeq(contour);
  */

  //distance image
  floatA distImage; distImage.resizeAs(theta); distImage.setZero();
  cvDistTransform(CVMAT(contourImage), CVMAT(distImage), CV_DIST_L2, 5);
  //cvShow(.01f*(distImage), "distance", true);

  //multiple restarts for fitting
  arr bestParams;
  double bestCost=0.;
  ShapeFitProblem problem;
  for(uint k=0; k<2; k++) {
    if(k || !reuseParams) switch(type) {
        case 0: //sphere
          params = ARR(component.rect.x + 0.5*component.rect.width,
                       component.rect.y + 0.5*component.rect.height,
                       (component.rect.width+component.rect.width)/4.);
          break;
        case 1: //cylinder
          params = ARR(component.rect.x + 0.5*component.rect.width,
                       component.rect.y + 0.5*component.rect.height,
                       component.rect.width,
                       .7*component.rect.height,
                       .2*component.rect.height);
          break;
        case 2: //box
          params = ARR(component.rect.x + .5*component.rect.width,
                       component.rect.y,
                       .5*component.rect.width, .2*component.rect.height,
                       0, .8*component.rect.height,
                       -.5*component.rect.width, .2*component.rect.height);
          break;
        case 3: //polygon
          params.setText("[ 0, 0, .5, -.2,  1, 0,  1, 1,  .5, 1.2,  0, 1]");
          for(uint k=0; k<6; k++) {
            params(2*k+0) = component.rect.x + component.rect.width *params(2*k+0);
            params(2*k+1) = component.rect.y + component.rect.height*params(2*k+1);
          }
        case 7: //capped cylinder
          params = ARR(component.rect.x + 0.5*component.rect.width,
                       component.rect.y + 0.5*component.rect.height,
                       component.rect.width,
                       .7*component.rect.height,
                       .2*component.rect.height);
          break;
        default: HALT("");
      }

    rndUniform(params, -5., 5., true);//Andreas: was -5,5

    problem.type=type;
    problem.N=20;
    problem.distImage = pow(distImage, 2.f);
    problem.display = biros().getParameter<bool>("shapeFitter_display", nullptr);;
    if(type==0) problem.radius = params(0);
    else problem.radius = 0;

    rai::timerStart();
    double cost;
    Rprop rprop;
    rprop.init();//Andreas: was 3.,5.
    rprop.loop(params, problem, &cost, 1.e-1, 1., 100, 0); //Andreas: was 1.e-1
    // cout <<"*** cost=" <<cost <<" params=" <<params <<" time=" <<rai::timerRead() <<endl;

    problem.fs(NoArr, NoArr, params);
    byteA img; copy(img, 10.f*problem.distImage);
    cvDrawPoints(img, problem.points);

    if(!bestParams.N || cost<bestCost) {
      bestCost=cost;  bestParams=params;
    }
  }

  //cout <<"best cost=" <<bestCost <<" params=" <<bestParams <<endl;
  //type=2;
  params=bestParams;
  points=problem.points;
  if(disp) {
    cvRectangle(CVMAT(*disp),
                cvPoint(component.rect.x, component.rect.y),
                cvPoint(component.rect.x+component.rect.width, component.rect.y+component.rect.height),
                CV_RGB(0, 255, 0), 1, 8, 0);
    cvDrawPoints(*disp, problem.points);
    //cvShow(*disp, "getShapeParamsFromEvidence", false);
  }

  return true;
}
#endif

//===========================================================================
//
// cost step routine
//

#ifdef RAI_OPENCV
void ShapeFitter::step() {
  CvMatDonor cvMatDonor;

  //get hsv evidence images from early vision modul
  floatA hsvL, hsvR;
  eviL->get_img(hsvL, this);
  eviR->get_img(hsvR, this);

  if(!hsvL.N) return;
  if(!hsvR.N) return;

  RigidObjectRepresentation* obj;
  bool suc;
  byteA disp = evi2rgb(hsvL[0]);

  objs.resize(hsvL.d0);

  //\todo iterate over all colors and over all objects inside those colors
  for(uint h=0; h<hsvL.d0; h++) {
    obj=&objs(h);
    obj->shapeType = (uint)objectType(h);

    arr oldshapePointsL(obj->shapePointsL), oldshapePointsR(obj->shapePointsR);

    if(obj->shapeType <= 2 || obj->shapeType == 7) {
      suc=getShapeParamsFromEvidence(obj->shapeParamsL,
                                     obj->shapePointsL,
                                     obj->shapeType,
                                     hsvL[h], &disp, obj->found);
      if(!suc) { obj->found=0; continue; }
      suc=getShapeParamsFromEvidence(obj->shapeParamsR,
                                     obj->shapePointsR,
                                     obj->shapeType,
                                     hsvR[h], nullptr, obj->found);
      if(!suc) { obj->found=0; continue; }

      //-- smooth!
      if(obj->found) {
        if(maxDiff(oldshapePointsL, obj->shapePointsL) > 15 ||
            maxDiff(oldshapePointsR, obj->shapePointsR) > 15) {
          obj->found=0;
        } else { //smooth
          obj->shapePointsL*=.2;  obj->shapePointsL+=.8*oldshapePointsL;
          obj->shapePointsR*=.2;  obj->shapePointsR+=.8*oldshapePointsR;
        }
      }

      //-- 3D projection
      obj->shapePoints3d.resize(obj->shapePointsR.d0, 3);
      for(uint i = 0; i < obj->shapePoints3d.d0;  i++) {
        arr vision(4);
        vision(0) = obj->shapePointsL(i, 0);
        vision(1) = obj->shapePointsL(i, 1);
        vision(2) = obj->shapePointsR(i, 0);
        vision(3) = obj->shapePointsR(i, 1);
        stereoTriangulation_nonhom(obj->shapePoints3d[i](), vision, Pl, Pr);
        //obj->shapePoints3d[i] =  Find3dPoint(Pl, Pr, vision);
      }

      //-- Object's ors params (height, radius, length, etc)
      uint n=obj->shapePoints3d.d0;
      obj->center3d = (1./n)*sum(obj->shapePoints3d, 0);
      if(obj->shapeType == 7) {
        obj->center3d(2) =obj->center3d(2)+ 0.015;//this capped cylinder has bias below
        obj->center3d(1)=obj->center3d(1)- 0.01;
      }
      if(obj->shapeType == 1 || 7) { //cylinder
        double h=
          .5*(length(obj->shapePoints3d[0*n/2]-obj->shapePoints3d[1*n/2-1]) + //rigth side bar
              length(obj->shapePoints3d[1*n/2]-obj->shapePoints3d[2*n/2-1])); //left side bar
        double r=0;
        for(uint i=0; i<n/2; i++) r += length(obj->shapePoints3d[i]-obj->shapePoints3d[n/2+i])/2.;
        r /= n/2;
        obj->orsShapeParams=ARR(0., 0., h, r);

        //printf("z before: %f ", obj->center3d.p[2]);
        //obj->center3d(2) += fabs(0.108-h);//move object center up, if the seen height is smaller than the original height
        //printf("z after: %f\n", obj->center3d.p[2]);
      }
      if(obj->shapeType == 2) { //box
        double h=
          .5*(length(obj->shapePoints3d[1*n/6]-obj->shapePoints3d[2*n/6-1]) +  //rigth side bar
              length(obj->shapePoints3d[4*n/6]-obj->shapePoints3d[5*n/6-1]));  //left side bar
        double x=
          .5*(length(obj->shapePoints3d[0*n/6]-obj->shapePoints3d[1*n/6-1]) +  //rigth side bar
              length(obj->shapePoints3d[3*n/6]-obj->shapePoints3d[4*n/6-1]));  //left side bar
        double y=
          .5*(length(obj->shapePoints3d[2*n/6]-obj->shapePoints3d[3*n/6-1]) +  //rigth side bar
              length(obj->shapePoints3d[5*n/6]-obj->shapePoints3d[6*n/6-1]));  //left side bar

        obj->diagDiff = obj->shapePoints3d[3*n/6] - obj->shapePoints3d[4*n/6];
        //for(uint d = 0; d < 6; d++)
        //  cout <<obj->shapePoints3d[d*n/6] <<endl;
        obj->orsShapeParams=ARR(x, y, h, 0);
      }
      if(obj->shapeType == 0) { //sphere
        double r=0;
        for(uint i=0; i<n; i++) r+=length(obj->shapePoints3d[0]-obj->center3d); //mitteln ueber alle punkte!
        r/=n;
        obj->orsShapeParams=ARR(0., 0., 0., r);
      }
      obj->found++;
    } else { //other index is just point mass, just single contour point
      uintA boxL, boxR; floatA axis, points;
      findMaxRegionInEvidence(boxL, &points, nullptr, hsvL[h], .5);//just use this information
      if(points.N && boxL.N) {
        obj->shapePointsL = arr(1, 2);
        obj->shapePointsL(0, 0) = points(0);
        obj->shapePointsL(0, 1) = points(1);
        cvRectangle(CVMAT(disp), cvPoint(boxL(0), boxL(1)), cvPoint(boxL(2), boxL(3)), cvScalar(255, 0, 0), 3);
      }
      findMaxRegionInEvidence(boxR, &points, nullptr, hsvR[h], .5);//just use this information
      if(points.N && boxR.N) {
        obj->shapePointsR = arr(1, 2);
        obj->shapePointsR(0, 0) = points(0);
        obj->shapePointsR(0, 1) = points(1);
        cvRectangle(CVMAT(disp), cvPoint(boxR(0), boxR(1)), cvPoint(boxR(2), boxR(3)), cvScalar(255, 255, 0), 3);
      }
      if(boxL.N && boxR.N) {
        arr vision(4);
        vision(0) = obj->shapePointsL(0, 0);
        vision(1) = obj->shapePointsL(0, 1);
        vision(2) = obj->shapePointsR(0, 0);
        vision(3) = obj->shapePointsR(0, 1);
        stereoTriangulation_nonhom(obj->center3d, vision, Pl, Pr);
      }
    }
  }

  percOut->writeAccess(this);
  percOut->display = disp;
  percOut->objects = objs; //this guy is not stateless!! Make all state stuff part of the percOut variable
  percOut->deAccess(this);
}
#endif

void realizeObjectsInOrs(rai::Configuration& ors, const rai::Array<RigidObjectRepresentation>& objects) {
  RigidObjectRepresentation* obj;  uint i;
  rai::Body* o = ors.getBodyByName("o1");
  uint indFirst = o->index;//hack to get consecutive bodies
  for(i=0; i<objects.N; i++) {
    obj=&objects(i);
    if(!obj->found) continue;
    rai::Body* o = ors.bodies(i+indFirst);
    rai::Shape* s = o->shapes(0);
    rai::ShapeType type=rai::ST_none;//SSD: getting rid of a warning. ok?
    if(obj->shapeType == 0) type=rai::ST_sphere;
    if(obj->shapeType == 1) type=rai::ST_cylinder;
    if(obj->shapeType == 2) { //box
      type = rai::ST_box;
      //  rai::Vector diag(obj->diagDiff(0), obj->diagDiff(1), obj->diagDiff(2)*0);//z is the smallest entry, but still not 0
      //double phi=acos(diag(0)/diag.length());//arccos btw 100 and diag
      // rai::Quaternion q;
      //q.setDiff(rai::Vector(1, 0, 0), diag);
      //q.setRad(phi, rai::Vector(0, 0, 1));
      // s->rel.r = q;
    }
    if(type!=s->type) {
      s->type=type;
      s->mesh.clear();
    }
    // s->rel.p = -o->X.p + rai::Vector(obj->center3d(0), obj->center3d(1), obj->center3d(2));
    o->X.pos =  rai::Vector(obj->center3d(0), obj->center3d(1), obj->center3d(2));
    memmove(s->size, obj->orsShapeParams.p, 4*sizeof(double));
    //cout <<" object " <<o->name <<" pos " <<o->X.p <<" " <<obj->center3d <<endl;
  }
}

/*void copyShapeInfos(rai::Configuration& A, const rai::Configuration& B){
  uint i; rai::Shape *s, *sa;
  for_list(Type,  s,  B.shapes){
    sa = A.shapes(i);
    if(sa->type!=s->type){
      sa->type=s->type;
      sa->mesh.clear();
    }
    sa->rel=s->rel;
    memmove(sa->size, s->size, 4*sizeof(double));
  }
}*/

void copyBodyInfos(rai::Configuration& A, const rai::Configuration& B) {
  uint i; rai::Body* b, *ba;
  rai::Shape* s, *sa;
  for_list(Type,  b,  B.bodies) if(b->shapes.N) {
    s = b->shapes(0);
    ba = A.bodies(i);
    sa = ba->shapes(0);
    if(sa->type!=s->type) {
      sa->type=s->type;
      sa->mesh.clear();
    }
    ba->X= b->X;
    memmove(sa->size, s->size, 4*sizeof(double));   // if(b->index >= 17) cout <<" pos " <<ba->name <<" " <<ba->X.p <<endl;
  }
}

#ifndef RAI_OPENCV
void ShapeFitter::step() { NICO }
#endif

#endif
