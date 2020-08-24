/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

// this requires T to be defined!!!

template struct rai::Array<T>;

template rai::Array<T> rai::operator~(const Array<T>& y);
template rai::Array<T> rai::operator-(const Array<T>& y, const Array<T>& z);
#ifndef NOFLOAT
template rai::Array<T> rai::operator*(const Array<T>& y, const Array<T>& z);
#endif
template rai::Array<T> rai::operator^(const Array<T>& y, const Array<T>& z);
template rai::Array<T> rai::operator%(const Array<T>& y, const Array<T>& z);
template rai::Array<T> rai::operator*(T, const Array<T>& z);
template rai::Array<T> rai::operator*(const Array<T>& z, T);
template rai::Array<T> rai::operator-(const Array<T>& z);
template rai::Array<T> rai::operator-(T, const Array<T>& z);
template rai::Array<T> rai::operator-(const Array<T>& z, T);
template rai::Array<T> rai::operator+(const Array<T>& y, const Array<T>& z);
template rai::Array<T> rai::operator+(T, const Array<T>& z);
template rai::Array<T>& rai::operator+=(Array<T>& y, T);
template rai::Array<T>& rai::operator+=(Array<T>& y, const Array<T>& z);
template rai::Array<T>& rai::operator-=(Array<T>& y, T);
template rai::Array<T>& rai::operator-=(Array<T>& y, const Array<T>& z);
template rai::Array<T>& rai::operator*=(Array<T>& y, T);
template rai::Array<T>& rai::operator/=(Array<T>& y, T);
template bool rai::operator==(const Array<T>& v, const Array<T>& w);
template bool rai::operator==(const Array<T>& v, const T* w);
template std::istream& rai::operator>>(std::istream& is, Array<T>& x);
template std::ostream& rai::operator<<(std::ostream& os, const Array<T>& x);

//BinaryOperation
template void transpose(rai::Array<T>& x, const rai::Array<T>& y);
template void inverse2d(rai::Array<T>& invA, const rai::Array<T>& A);

template T absMax(const rai::Array<T>& v);
template T absMin(const rai::Array<T>& v);
template T entropy(const rai::Array<T>& v);
template T normalizeDist(rai::Array<T>& v);
template void makeConditional(rai::Array<T>& P);
template void checkNormalization(rai::Array<T>& v, double tol);
template void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d);
template void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d, uint e);
template void eliminatePartial(rai::Array<T>& x, const rai::Array<T>& y, uint d);
template void checkNan(rai::Array<T> const&);

template T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w);
template T maxDiff(const rai::Array<T>& v, const rai::Array<T>& w, uint* im);
template T maxRelDiff(const rai::Array<T>& v, const rai::Array<T>& w, T tol);
//template T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w, const rai::Array<bool>& mask);
template T sqrDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);
template T euclideanDistance(const rai::Array<T>& v, const rai::Array<T>& w);
template T metricDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);

template T sum(const rai::Array<T>& v);
template T scalar(const rai::Array<T>& v);
template rai::Array<T> sum(const rai::Array<T>& v, uint d);
template T sumOfAbs(const rai::Array<T>& v);
template T sumOfSqr(const rai::Array<T>& v);
template T length(const rai::Array<T>& v);

template T var(const rai::Array<T>& v);
template T trace(const rai::Array<T>& v);

template rai::Array<T> log(const rai::Array<T>& v);
template rai::Array<T> exp(const rai::Array<T>& v);
template rai::Array<T> atan(const rai::Array<T>& v);
template rai::Array<T> pow(const rai::Array<T>& v, T);

template T minDiag(const rai::Array<T>& v);

template T product(const rai::Array<T>& v);
#ifndef NOFLOAT
template void innerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
#endif
template void outerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template T scalarProduct(const rai::Array<T>& v, const rai::Array<T>& w);
template T scalarProduct(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);

template rai::Array<T> catCol(const rai::Array<rai::Array<T>* >& X);

template void tensorEquation(rai::Array<T>& X, const rai::Array<T>& A, const uintA& pickA, const rai::Array<T>& B, const uintA& pickB, uint sum);
template void tensorPermutation(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid);
template void tensorMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const rai::Array<uint>& Yid);
template void tensorMaxMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const rai::Array<uint>& Yid);
template void tensorMarginal_old(rai::Array<T>& y, const rai::Array<T>& x, const rai::Array<uint>& xd, const rai::Array<uint>& ids);
template void tensorMultiply(rai::Array<T>& X, const rai::Array<T>& Y, const rai::Array<uint>& Yid);
template void tensorMultiply_old(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<uint>& d, const rai::Array<uint>& ids);

template void rndInteger(rai::Array<T>& a, int low, int high, bool add);
template void rndUniform(rai::Array<T>& a, double low, double high, bool add);
template void rndGauss(rai::Array<T>& a, double stdDev, bool add);
//template void rndGauss(rai::Array<T>& a, bool add);
//template rai::Array<T>& rndGauss(double stdDev, uint dim);
template uint softMax(const rai::Array<T>& a, rai::Array<double>& soft, double beta);

#undef T
