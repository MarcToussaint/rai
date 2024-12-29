arr BSpline::getCoeffs(double t, uint K, uint derivative) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    if(derivative>0) { db_0=db; db.setZero(); }
    if(derivative>1) { ddb_0=ddb; ddb.setZero(); }
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(!k && t<knotTimes(0)) b(k)=1.;
        else if(k==K && t>=knotTimes(k)) b(k)=1.;
        else if(knotTimes(k)<=t && t<knotTimes(k+1)) b(k)=1.;
      } else {
        if(k+p<knotTimes.N) {
          double xnom = t - knotTimes(k);
          double xden = knotTimes(k+p) - knotTimes(k);
          double x = DIV(xnom, xden, true);
          b(k) = x * b_0(k);
          if(derivative>0) db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
          if(derivative>1) ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        }
        if(k<K && k+p+1<knotTimes.N) {
          double ynom = knotTimes(k+p+1) - t;
          double yden = knotTimes(k+p+1) - knotTimes(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          if(derivative>0) db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          if(derivative>1) ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
    if(t<knotTimes(0) || t>=knotTimes.last()) break;
  }
  switch(derivative) {
    case 0:
      return b;
    case 1:
      return db;
    case 2:
      return ddb;
  }
  HALT("Derivate of order " << derivative << " not yet implemented.");
}

#define ZDIV(x,y) (y?x/y:0.)

void BSpline::getCoeffs2(arr& b, arr& db, arr& ddb, double t, uint degree, double* knots, uint nCtrls, uint nKnots, uint derivatives) {
  CHECK_EQ(nCtrls+degree+1, nKnots, "");

  b.resize(nCtrls).setZero();
  if(derivatives>0) db.resize(nCtrls).setZero();
  if(derivatives>1) ddb.resize(nCtrls).setZero();

  arr b_prev, db_prev, ddb_prev;
  for(uint p=0; p<=degree; p++) {
    b_prev=b; b.setZero();
    if(derivatives>0) { db_prev=db; db.setZero(); }
    if(derivatives>1) { ddb_prev=ddb; ddb.setZero(); }
    for(uint i=0; i<nCtrls; i++) {
      if(!p) {
        if(!i && t<knots[0]) b.elem(i)=1.;
        else if(i==nCtrls-1 && t>=knots[i]) b.elem(i)=1.;
        else if(knots[i]<=t && t<knots[i+1]) b.elem(i)=1.;
      } else {
        if(i+p<nKnots) {
          double xnom = t - knots[i];
          double xden = knots[i+p] - knots[i];
          double x = ZDIV(xnom, xden);
          b.elem(i) = x * b_prev.elem(i);
          if(derivatives>0) db.elem(i) = ZDIV(1., xden) * b_prev.elem(i) + x * db_prev.elem(i);
          if(derivatives>1) ddb.elem(i) = ZDIV(2., xden) * db_prev.elem(i) + x * ddb_prev.elem(i);
        }
        if(i<nCtrls-1 && i+p+1<nKnots) {
          double ynom = knots[i+p+1] - t;
          double yden = knots[i+p+1] - knots[i+1];
          double y = ZDIV(ynom, yden);
          b.elem(i) += y * b_prev.elem(i+1);
          if(derivatives>0) db.elem(i) += ZDIV(-1., yden) * b_prev.elem(i+1) + y * db_prev.elem(i+1);
          if(derivatives>1) ddb.elem(i) += ZDIV(-2., yden) * db_prev.elem(i+1) + y * ddb_prev.elem(i+1);
        }
      }
    }
    if(t<knots[0] || t>=knots[nKnots-1]) break;
  }
}


void BSpline::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
#if 0 //computing coeffs for ALL knot points (most zero...)
//  uint K = knotPoints.d0-1;
  arr coeffs = getCoeffs(t, knotPoints.d0-1, derivative);
//  arr coeffs = getCoeffs2(t, degree, knotTimes.p, knotPoints.d0, knotTimes.N, derivative);
  return (~coeffs * knotPoints).reshape(knotPoints.d1);
#else //pick out only the LOCAL knot points

  //find the first knotTime >t
  int offset = knotTimes.rankInSorted(t, rai::lowerEqual<double>, true);
  offset -= degree+1;
  if(offset<0) offset=0;

  uint knotN = degree+1;
  uint knotTimesN = knotN + 1+degree;
  if(offset+knotTimesN>knotTimes.N) offset = knotTimes.N - knotTimesN;

  //get coeffs
  arr b, db, ddb;
  uint derivative=0;
  if(!!xDot) derivative=1;
  if(!!xDDot) derivative=2;
  getCoeffs2(b, db, ddb, t, degree, knotTimes.p+offset, knotN, knotTimesN, derivative);

//linear combination
#if 0
  uint n = ctrlPoints.d1;
  if(!!x) x.resize(n).setZero();
  if(!!xDot) xDot.resize(n).setZero();
  if(!!xDDot) xDDot.resize(n).setZero();
  for(uint j=0; j<b.N; j++) if(offset+j>=0) {
    if(!!x) for(uint k=0; k<n; k++) x.elem(k) += b.elem(j)*ctrlPoints(offset+j, k);
    if(!!xDot) for(uint k=0; k<n; k++) xDot.elem(k) += db.elem(j)*ctrlPoints(offset+j, k);
    if(!!xDDot) for(uint k=0; k<n; k++) xDDot.elem(k) += ddb.elem(j)*ctrlPoints(offset+j, k);
  }
#else
  arr sel_ctrlPoints;
  if(offset<0){
    if(b.N) b=b.sub(-offset,-1);
    if(db.N) db=db.sub(-offset,-1);
    if(ddb.N) ddb=ddb.sub(-offset,-1);
    offset=0;
  }
  sel_ctrlPoints.referToRange(ctrlPoints, offset, offset+b.N-1);
  if(!!x){ x = ~b * sel_ctrlPoints; x.reshape(-1); }
  if(!!xDot){ xDot = ~db * sel_ctrlPoints; xDot.reshape(-1); }
  if(!!xDDot){ xDDot = ~ddb * sel_ctrlPoints; xDDot.reshape(-1); }
#endif
#endif
}
