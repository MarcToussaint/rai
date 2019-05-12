/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "roboticsCourse.h"
#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Kin/kin_physx.h>
#include <Gui/opengl.h>
#include <Plot/plot.h>
#include <Algo/algos.h>

void drawEnv(void*, OpenGL& gl) { glStandardLight(NULL, gl); glDrawFloor(10., .9, .9, .9); }
void drawBase(void*, OpenGL& gl) { glDrawAxes(1.); }

struct sSimulator {
  rai::KinematicWorld G;
  double margin;
  double dynamicNoise;
  bool gravity;
  
  //state
  arr qddot;
  
  sSimulator() { margin=.1; dynamicNoise=0.; gravity=true; } //default margin = 10cm
};

void Simulator::anchorKinematicChainIn(const char* bodyName) {
  s->G.reconfigureRootOfSubtree(s->G.getFrameByName(bodyName));
//  s->G.calc_fwdPropagateFrames();

  NIY;
//  if(s->G.swift().isOpen){
//    s->G.swift().close();
//    s->G.swift().init(s->G);
//    s->G.swift().setCutoff(.5);
//  }

//#ifdef RAI_ODE
//  if(s->ode.isOpen){
//    s->ode.clear();
//    s->ode.createOde(s->G);
//  }
//#endif
}

Simulator::Simulator(const char* orsFile) {
  s = new sSimulator;
  
  //RAI
  s->G.init(orsFile);
  /*  if(s->G.getBodyByName("rfoot")){
    s->G.reconfigureRoot(s->G.getBodyByName("rfoot"));
    s->G.calcBodyFramesFromJoints();
    }*/
  
  //G.makeLinkTree();
  makeConvexHulls(s->G.frames);
  
  //OPENGL
  s->G.glAdd(glDrawPlot, &plotModule);
  
  //SWIFT
  s->G.swift().setCutoff(.5);
}

Simulator::~Simulator() {
  delete s;
}

void Simulator::watch(bool pause, const char* txt) {
  s->G.watch(pause, txt);
}

void Simulator::getJointAngles(arr& q) {
  q = s->G.q;
}

void Simulator::getJointAnglesAndVels(arr& q, arr& qdot) {
  q = s->G.q;
  qdot = s->G.qdot;
}

uint Simulator::getJointDimension() {
  return s->G.getJointStateDimension();
}

void Simulator::setJointAngles(const arr& q, bool updateDisplay) {
  s->G.setJointState(q);
  s->G.stepSwift();
  if(updateDisplay) s->G.watch(false);
}

void Simulator::setJointAnglesAndVels(const arr& q, const arr& qdot, bool updateDisplay) {
  s->G.setJointState(q, qdot);
  s->G.stepSwift();
  if(updateDisplay) s->G.watch(false);
}

void Simulator::kinematicsPos(arr& y, const char* shapeName, const arr* rel) {
  if(rel) {
    rai::Vector v;  v.set(rel->p);
    s->G.kinematicsPos(y, NoArr, s->G.getFrameByName(shapeName), v);
  } else {
    s->G.kinematicsPos(y, NoArr, s->G.getFrameByName(shapeName));
  }
}

void Simulator::kinematicsVec(arr& y, const char* shapeName, const arr* vec) {
  if(vec) {
    rai::Vector v;  v.set(vec->p);
    s->G.kinematicsVec(y, NoArr, s->G.getFrameByName(shapeName), v);
  } else {
    s->G.kinematicsVec(y, NoArr, s->G.getFrameByName(shapeName));
  }
}

void Simulator::jacobianPos(arr& J, const char* shapeName, const arr* rel) {
  if(rel) {
    rai::Vector v;  v.set(rel->p);
    s->G.kinematicsPos(NoArr, J, s->G.getFrameByName(shapeName), v);
  } else {
    s->G.kinematicsPos(NoArr, J, s->G.getFrameByName(shapeName));
  }
}

void Simulator::jacobianVec(arr& J, const char* shapeName, const arr* vec) {
  if(vec) {
    rai::Vector v;  v.set(vec->p);
    s->G.kinematicsVec(NoArr, J, s->G.getFrameByName(shapeName), v);
  } else {
    s->G.kinematicsVec(NoArr, J, s->G.getFrameByName(shapeName));
  }
}

void Simulator::kinematicsCOM(arr& y) {
  s->G.getCenterOfMass(y);
  y.resizeCopy(2);
}

void Simulator::jacobianCOM(arr& J) {
  s->G.getComGradient(J);
  J.resizeCopy(2, J.d1);
}

void Simulator::reportProxies() {
  s->G.reportProxies();
}

void Simulator::setContactMargin(double margin) {
  s->margin = margin;
}

void Simulator::kinematicsContacts(arr& y) {
  s->G.kinematicsProxyCost(y, NoArr, s->margin);
}

void Simulator::jacobianContacts(arr& J) {
  arr y;
  s->G.kinematicsProxyCost(y, J, s->margin);
}

double Simulator::getEnergy() {
  return s->G.getEnergy();
}

void Simulator::setDynamicSimulationNoise(double noise) {
  s->dynamicNoise = noise;
}

void Simulator::setDynamicGravity(bool gravity) {
  s->gravity = gravity;
}

void Simulator::getDynamics(arr& M, arr& F) {
  s->G.equationOfMotion(M, F);
}

void Simulator::stepDynamics(const arr& Bu, double tau) {
  s->G.stepDynamics(Bu, tau, s->dynamicNoise);
}

void Simulator::stepOde(const arr& qdot, double tau) {
#ifdef RAI_ODE
  s->G.ode().setMotorVel(qdot, 100.);
  s->G.ode().step(tau);
  s->G.ode().importStateFromOde();
#endif
}

void Simulator::stepPhysx(const arr& qdot, double tau) {
  s->G.physx().step(tau);
}

rai::KinematicWorld& Simulator::getOrsGraph() {
  return s->G;
}

struct sVisionSimulator {
  OpenGL gl;
  arr P;
  sVisionSimulator() { }
};

VisionSimulator::VisionSimulator() {
  s = new sVisionSimulator;
  
  s->P.resize(3, 4);
  
  //OPENGL
  s->gl.add(drawEnv, 0);
  s->gl.add(drawBase, 0);
  s->gl.setClearColors(1., 1., 1., 1.);
  s->gl.camera.setPosition(10., -15., 8.);
  s->gl.camera.focus(0, 0, 0);
  s->gl.camera.upright();
  s->gl.update();
  s->gl.add(glDrawPlot, &plotModule);
  
}

VisionSimulator::~VisionSimulator() {
  delete s;
}

void VisionSimulator::watch() {
  s->gl.watch();
}

void VisionSimulator::getRandomWorldPoints(arr& X, uint N) {
  //generate N random 3D world points
  X.resize(N, 4);
  rndUniform(X, -1., 1., false);  //each point is random in [-1, 1]^4
  for(uint i=0; i<N; i++) {
    X(i, 3)=1.;                 //initialize 4th coordinate to 1
  }
}

arr VisionSimulator::getCameraTranslation() {
  return conv_vec2arr(s->gl.camera.X.pos);
}

void VisionSimulator::projectWorldPointsToImagePoints(arr& x, const arr& X, double noiseInPixel) {
#ifdef FREEGLUT
  uint N=X.d0;
  x.resize(N, 3);
  
  //*
  arr y(3);
  arr Mmodel(4, 4), Mproj(4, 4); intA Mview(4);
  glGetDoublev(GL_MODELVIEW_MATRIX, Mmodel.p);
  glGetDoublev(GL_PROJECTION_MATRIX, Mproj.p);
  glGetIntegerv(GL_VIEWPORT, Mview.p);
  //cout <<Mview <<endl;
  //cout <<Mmodel <<endl;
  //cout <<Mproj <<s->P <<endl;
  //*/
  intA view(4);
  glGetIntegerv(GL_VIEWPORT, view.p);
  
  //project the points using the OpenGL matrix
  s->P = s->gl.P;
  s->P /= s->P(0, 0);
  cout <<"VisionSimulator:"
       <<"\n  projection matrix used: " <<s->P
       <<"\n  camera position and quaternion: " <<s->gl.camera.X.pos <<"  " <<s->gl.camera.X.rot
       <<"\n  camera f=" <<.5*view(2) <<" x0=" <<view(0)+.5*view(2) <<" y0=" <<view(1)+.5*view(2)
       <<endl;
  for(uint i=0; i<N; i++) {
    x[i] = s->P*X[i];
    x[i]() /= x(i, 2);
    //gluProject(X(i, 0), X(i, 1), X(i, 2), Mmodel.p, Mproj.p, Mview.p, &y(0), &y(1), &y(2));
    //cout <<"y=" <<y <<" x=" <<x[i] <<endl;
  }
  rndGauss(x, noiseInPixel, true); //add Gaussian noise
  for(uint i=0; i<N; i++) x(i, 2)=1.;
  
  plotPoints(X);
  //s->gl.watch();
#endif
}

void glDrawCarSimulator(void *classP, OpenGL&);

CarSimulator::CarSimulator() {
  //car parameters
  x=y=theta=0;
  tau=1.; //one second time steps
  L=2.; //2 meters between the wheels
  dynamicsNoise = .03;
  observationNoise = .5;
  
  //landmarks
  landmarks.resize(2,2);
  rndGauss(landmarks, 10.);
  //landmarks=ARR(10,0); landmarks.reshape(1,2);
  
  gl=new OpenGL;
  gl->add(drawEnv, this);
  gl->add(glDrawCarSimulator, this);
  gl->add(glDrawPlot,&plotModule);
  
  gl->camera.setPosition(10., -50., 100.);
  gl->camera.focus(0, 0, .5);
  gl->camera.upright();
  gl->update();
}

void CarSimulator::step(const arr& u) {
  double v=u(0), phi=u(1);
  x += tau*v*cos(theta);
  y += tau*v*sin(theta);
  theta += tau*(v/L)*tan(phi);
  
  if(dynamicsNoise) {
    x += dynamicsNoise*rnd.gauss();
    y += dynamicsNoise*rnd.gauss();
    theta += dynamicsNoise*rnd.gauss();
  }
  
  plotClear();
  for(uint i=0; i<gaussiansToDraw.N; i++) plotCovariance(gaussiansToDraw(i).a, gaussiansToDraw(i).A);
  gl->update();
}

void CarSimulator::getRealNoisyObservation(arr& Y) {
  getMeanObservationAtState(Y, ARR(x,y,theta));
  rndGauss(Y,observationNoise,true);
}

void CarSimulator::getMeanObservationAtState(arr& Y, const arr& X) {
  Y=landmarks;
  arr R = ARR(cos(X(2)), -sin(X(2)), sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  arr p = ones(landmarks.d0,1)*~ARR(X(0),X(1));
  Y -= p;
  Y = Y*R;
  Y.reshape(Y.N);
}

void CarSimulator::getLinearObservationModelAtState(arr& C, arr& c, const arr& X) {
  uint N=landmarks.d0;
  arr R = ARR(cos(X(2)), sin(X(2)), -sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  C.resize(2*N,2*N);  C.setZero();
  for(uint i=0; i<N; i++) C.setMatrixBlock(R, 2*i, 2*i);
  cout <<C <<endl;
  c.resize(2*N);
  for(uint i=0; i<N; i++) c.setVectorBlock(ARR(X(0),X(1)), 2*i);
  c = - C * c;
}

void CarSimulator::getObservationJacobianAtState(arr& dy_dx, const arr& X) {
  uint N=landmarks.d0;
  dy_dx = arr(2*N,3); dy_dx.setZero();
  for(uint i=0; i<N; i++) {
    arr J(2,3); J.setZero();
    //by x
    J(0,0) = -cos(X(2));
    J(1,0) = sin(X(2));
    //by y
    J(0,1) = -sin(X(2));
    J(1,1) = -cos(X(2));
    //by theta
    J(0,2) = -sin(X(2))*(landmarks(i,0)-X(0)) + cos(X(2))*(landmarks(i,1)-X(1));
    J(1,2) = -cos(X(2))*(landmarks(i,0)-X(0)) - sin(X(2))*(landmarks(i,1)-X(1));
    dy_dx[i*2] = J[0];//copy in big J
    dy_dx[i*2+1] = J[1];
  }
}

void glDrawCarSimulator(void *classP, OpenGL&) {
#ifdef FREEGLUT
  CarSimulator *s=(CarSimulator*)classP;
  rai::Transformation f;
  f.setZero();
  f.addRelativeTranslation(s->x,s->y,.3);
  f.addRelativeRotationRad(s->theta, 0., 0., 1.);
  f.addRelativeTranslation(1.,0.,0.);
  
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(3., 1.5, .5);
  
  for(uint l=0; l<s->landmarks.d0; l++) {
    f.setZero();
    f.addRelativeTranslation(s->landmarks(l,0),s->landmarks(l,1), .5);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(.2,.8,.2);
    glDrawCylinder(.1,1.);
  }
  
  glLoadIdentity();
  glColor(.2,.2,.8);
  for(uint l=0; l<s->particlesToDraw.d0; l++) {
    glPushMatrix();
    glTranslatef(s->particlesToDraw(l,0), s->particlesToDraw(l,1), .6);
    glDrawDiamond(.1, .1, .1);
    glPopMatrix();
  }
  
  for(uint l=0; l<s->particlesToDraw.d0; l++) {
  }
#endif
}

#include <Core/array.tpp>
template rai::Array<CarSimulator::Gaussian>& rai::Array<CarSimulator::Gaussian>::resize(uint);
