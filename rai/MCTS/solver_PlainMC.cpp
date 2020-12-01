/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "solver_PlainMC.h"

void MCStatistics::add(double R, uint topSize) {
  n++;
  if(X.N<topSize) X.insertInSorted(R, rai::greaterEqual<double>);
  else if(R>X.last()) {
    X.insertInSorted(R, rai::greaterEqual<double>);
    X.popLast();
  }
}

PlainMC::PlainMC(MCTS_Environment& world)
  : world(world), gamma(.9), verbose(2), topSize(10) {
  reset();
  gamma = world.get_info_value(MCTS_Environment::getGamma);
  rai::FileToken fil("PlainMC.blackList");
  if(fil.exists()) {
    blackList.read(fil.getIs());
  }
}

void PlainMC::reset() {
  A = world.get_actions();
//  if(verbose>1){ cout <<"START decisions: [" <<A.N <<']'; listWrite(A); cout <<endl; }
  D.clear();
  D.resize(A.N);
}

double PlainMC::initRollout(const rai::Array<MCTS_Environment::Handle>& prefixDecisions) {
  world.reset_state();

  //reset rollout 'return' variables
  rolloutStep=0;
  rolloutR=0.;
  rolloutDiscount=1.;
  rolloutDecisions.clear();

  MCTS_Environment::TransitionReturn ret;

  //-- follow prefixDecisions
  for(uint i=0; i<prefixDecisions.N; i++) {
    MCTS_Environment::Handle& a = prefixDecisions(i);
    if(verbose>1) cout <<"****************** MC: prefix decision " <<i <<": " <<*a <<endl;
    rolloutDecisions.append(a);
    ret = world.transition(a);
    rolloutR += rolloutDiscount * ret.reward;
    rolloutDiscount *= pow(gamma, ret.duration);  //  discount *= gamma;
  }

  return rolloutR;
}

double PlainMC::finishRollout(int stepAbort) {
  MCTS_Environment::TransitionReturn ret;

  //-- continue with random rollout
  while(!world.is_terminal_state() && (stepAbort<0 || rolloutStep++<(uint)stepAbort)) {
    rai::Array<MCTS_Environment::Handle> actions;
    actions = world.get_actions(); //WARNING: conv... returns a reference!!
    if(verbose>2) { cout <<"Possible decisions: "; listWrite(actions); cout <<endl; }
    if(!actions.N) {
      if(verbose>1) { cout <<" -- no decisions left -> terminal" <<endl; }
      break;
    }
    uint a = rand()%actions.N;
    if(verbose>1) cout <<"****************** MC: random decision: " <<*actions(a) <<endl;
    rolloutDecisions.append(actions(a));
    ret = world.transition(actions(a));
    rolloutR += rolloutDiscount * ret.reward;
    rolloutDiscount *= pow(gamma, ret.duration);    //    discount *= gamma;
  }

  if(stepAbort>=0 && rolloutStep>=(uint)stepAbort) rolloutR -= 100.;
  if(verbose>0) cout <<"****************** MC: terminal state reached; step=" <<rolloutStep <<" Return=" <<rolloutR <<endl;

  return rolloutR;
}

double PlainMC::generateRollout(int stepAbort, const rai::Array<MCTS_Environment::Handle>& prefixDecisions) {
#if 1
  initRollout(prefixDecisions);
  return finishRollout(stepAbort);
#else
  world.reset_state();

  //reset rollout 'return' variables
  rolloutStep=0;
  rolloutR=0.;
  rolloutDiscount=1.;
  rolloutDecisions.clear();

  MCTS_Environment::TransitionReturn ret;

  //-- follow prefixDecisions
  for(uint i=0; i<prefixDecisions.N; i++) {
    MCTS_Environment::Handle& a = prefixDecisions(i);
    if(verbose>1) cout <<"****************** MC: prefix decision " <<i <<": " <<*a <<endl;
    rolloutDecisions.append(a);
    ret = world.transition(a);
    rolloutR += rolloutDiscount * ret.reward;
    rolloutDiscount *= pow(gamma, ret.duration);  //  discount *= gamma;
  }

  //-- continue with random rollout
  while(!world.is_terminal_state() && (stepAbort<0 || rolloutStep++<(uint)stepAbort)) {
    rai::Array<MCTS_Environment::Handle> actions;
    actions = conv_stdvec2arr(world.get_actions()); //WARNING: conv... returns a reference!!
    if(verbose>2) { cout <<"Possible decisions: "; listWrite(actions); cout <<endl; }
    if(!actions.N) {
      if(verbose>1) { cout <<" -- no decisions left -> terminal" <<endl; }
      break;
    }
    uint a = rand()%actions.N;
    if(verbose>1) cout <<"****************** MC: random decision: " <<*actions(a) <<endl;
    rolloutDecisions.append(actions(a));
    ret = world.transition(actions(a));
    rolloutR += rolloutDiscount * ret.reward;
    rolloutDiscount *= pow(gamma, ret.duration);    //    discount *= gamma;
  }

  if(stepAbort>=0 && rolloutStep>=(uint)stepAbort) rolloutR -= 100.;
  if(verbose>0) cout <<"****************** MC: terminal state reached; step=" <<rolloutStep <<" Return=" <<rolloutR <<endl;

  return rolloutR;
#endif
}

double PlainMC::addRollout(int stepAbort) {
  // random first choice
  uint a = rnd(A.N);

  //generate rollout
  generateRollout(stepAbort, {A(a)});

  if(blackList.N) {
    rai::String decisionsString;
    for(const auto& a:rolloutDecisions) decisionsString <<*a <<' ';
    for(const rai::String& black:blackList) {
      if(decisionsString.startsWith(black)) {
        if(verbose>0) cout <<"****************** MC: rollout was on BLACKLIST: " <<*black <<endl;
        return 0.;
      }
    }
  }

  //-- collect data
  addReturnToStatistics(rolloutR, A(a), a);

  return rolloutR;
}

void PlainMC::addReturnToStatistics(double rolloutR, MCTS_Environment::Handle decision, int decisionIndex) {
  if(decisionIndex>=0) {
    CHECK_EQ(A(decisionIndex), decision, "")
  } else { //search for index..
//    cout <<*decision <<endl;
//    listWrite(A);
    uint i;
    for(i=0; i<A.N; i++) if(*A(i)==*decision) break;
    CHECK(i<A.N, "");
    decisionIndex=i;
  }
  D(decisionIndex).add(rolloutR);
  Droot.add(rolloutR);
}

void PlainMC::report() {
  cout <<"MC Planner report:" <<endl;
  for(uint a=0; a<A.N; a++) {
    cout <<"action=" <<*A(a) <<" n=" <<D(a).n <<" returns=" <<D(a).X <<endl;
  }
}

uint PlainMC::getBestActionIdx() {
  arr Q(A.N);
  double Qmin=0.;
  if(world.get_info(world.hasMinReward)) Qmin = world.get_info_value(world.getMinReward);
  for(uint a=0; a<A.N; a++) {
    if(D(a).X.N)
      Q(a) = D(a).X.first();
    else
      Q(a) = Qmin;
  }
  return Q.argmax();
}

MCTS_Environment::Handle PlainMC::getBestAction() {
  return A(getBestActionIdx());
}

