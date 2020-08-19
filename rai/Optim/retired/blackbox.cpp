/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================

LocalModelBasedOptim::LocalModelBasedOptim(arr& _x, const ScalarFunction& _f,  OptOptions _o)
  : x_init(_x), f(_f), best(nullptr), o(_o), it(0), evals(0), numTinySteps(0) {
  alpha = o.initStep;
}

LocalModelBasedOptim::~LocalModelBasedOptim() {
  listDelete(D);
}

void LocalModelBasedOptim::step() {
  if(!D.N) { //no data yet
    if(o.verbose>1) cout <<"*** LocalModelBasedOptim:" <<endl;
    evaluate(x_init);

    //startup verbose
//    if(o.verbose>0) fil.open("z.opt");
//    if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<best->f <<' ' <<alpha;
//    if(o.verbose>2) fil <<' ' <<x;
//    if(o.verbose>0) fil <<endl;

    return;
  }
  if(D.N==1) { //only 1 data point
    arr x = x_init;
    rndGauss(x, alpha, true);
    evaluate(x);
    return;
  }

  arr X, y; // construct data set
  for(uint i=0; i<D.N; i++) {
//    if(i>=2*x_init.N) break; //do not add more points than needed...but how many are needed?
    X.append(D(i)->x);
    y.append(D(i)->f);
  }

  X.reshape(y.N, x_init.N);
  X = catCol(ones(y.N), X); //add bias term
  arr beta = inverse_SymPosDef(~X* X + 1e-1*eye(X.d1))* ~X * y;
  arr grad = beta.sub(1, -1); //remove bias term

  arr delta = grad / length(grad); //always normalize gradient
  //add 'exploration' (determinante component...)

  arr x = best->x - alpha*delta;

  evaluate(x);
}

void LocalModelBasedOptim::run(uint maxIt) {
  numTinySteps=0;
  for(uint i=0; i<maxIt; i++) {
    step();
//    if(stopCriterion==stopStepFailed) continue;
//    if(stopCriterion==stopCritLineSteps){ reinit();   continue; }
//    if(stopCriterion>=stopCrit1) break;
  }
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, false);
//  if(o.fmin_return) *o.fmin_return= fx;
//  return stopCriterion;
}

bool DatumSortCompare(LocalModelBasedOptim::Datum* const& a, LocalModelBasedOptim::Datum* const& b) {
  return a->distToBest <= b->distToBest;
}

void LocalModelBasedOptim::evaluate(const arr& x, bool sort) {
  if(o.verbose>2) cout <<" \tprobing y=" <<x <<flush;
  double dist = best ? euclideanDistance(x, best->x) : 0.;
  double fx = f(NoArr, NoArr, x);  evals++;
  if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fx <<endl;
  Datum d = {x, fx, dist};
  D.append(new Datum(d));
  if(!best) best = D.last(); //first data point
  if(d.f<best->f) { //a new best...
    best = D.last();
    for(Datum* d:D) d->distToBest=euclideanDistance(d->x, best->x);
  }
//  cout <<D <<endl;
//  if(sort) D.sort(DatumSortCompare);
}

