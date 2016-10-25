/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */



// this requires T to be defined!!!

template struct mlr::Array<T>;

template mlr::Array<T> mlr::operator~(const Array<T>& y);
template mlr::Array<T> mlr::operator-(const Array<T>& y, const Array<T>& z);
#ifndef NOFLOAT
template mlr::Array<T> mlr::operator*(const Array<T>& y, const Array<T>& z);
#endif
template mlr::Array<T> mlr::operator^(const Array<T>& y, const Array<T>& z);
template mlr::Array<T> mlr::operator%(const Array<T>& y, const Array<T>& z);
template mlr::Array<T> mlr::operator*(T, const Array<T>& z);
template mlr::Array<T> mlr::operator*(const Array<T>& z, T);
template mlr::Array<T> mlr::operator-(const Array<T>& z);
template mlr::Array<T> mlr::operator-(T, const Array<T>& z);
template mlr::Array<T> mlr::operator-(const Array<T>& z, T);
template mlr::Array<T> mlr::operator+(const Array<T>& y, const Array<T>& z);
template mlr::Array<T> mlr::operator+(T, const Array<T>& z);
template mlr::Array<T>& mlr::operator+=(Array<T>& y, T);
template mlr::Array<T>& mlr::operator+=(Array<T>& y, const Array<T>& z);
template mlr::Array<T>& mlr::operator-=(Array<T>& y, T);
template mlr::Array<T>& mlr::operator-=(Array<T>& y, const Array<T>& z);
template mlr::Array<T>& mlr::operator*=(Array<T>& y, T);
template mlr::Array<T>& mlr::operator/=(Array<T>& y, T);
template bool mlr::operator==(const Array<T>& v, const Array<T>& w);
template bool mlr::operator==(const Array<T>& v, const T *w);
template std::istream& mlr::operator>>(std::istream& is, Array<T>& x);
template std::ostream& mlr::operator<<(std::ostream& os, const Array<T>& x);

//BinaryOperation
template void transpose(mlr::Array<T>& x, const mlr::Array<T>& y);
template void inverse2d(mlr::Array<T>& invA, const mlr::Array<T>& A);

template T absMax(const mlr::Array<T>& v);
template T absMin(const mlr::Array<T>& v);
template T entropy(const mlr::Array<T>& v);
template T normalizeDist(mlr::Array<T>& v);
template void makeConditional(mlr::Array<T>& P);
template void checkNormalization(mlr::Array<T>& v, double tol);
template void eliminate(mlr::Array<T>& x, const mlr::Array<T>& y, uint d);
template void eliminate(mlr::Array<T>& x, const mlr::Array<T>& y, uint d, uint e);
template void eliminatePartial(mlr::Array<T>& x, const mlr::Array<T>& y, uint d);
template void checkNan(mlr::Array<T> const&);

template T sqrDistance(const mlr::Array<T>& v, const mlr::Array<T>& w);
template T maxDiff(const mlr::Array<T>& v, const mlr::Array<T>& w, uint *im);
template T maxRelDiff(const mlr::Array<T>& v, const mlr::Array<T>& w, T tol);
//template T sqrDistance(const mlr::Array<T>& v, const mlr::Array<T>& w, const mlr::Array<bool>& mask);
template T sqrDistance(const mlr::Array<T>& g, const mlr::Array<T>& v, const mlr::Array<T>& w);
template T euclideanDistance(const mlr::Array<T>& v, const mlr::Array<T>& w);
template T metricDistance(const mlr::Array<T>& g, const mlr::Array<T>& v, const mlr::Array<T>& w);

template T sum(const mlr::Array<T>& v);
template T scalar(const mlr::Array<T>& v);
template mlr::Array<T> sum(const mlr::Array<T>& v, uint d);
template T sumOfAbs(const mlr::Array<T>& v);
template T sumOfSqr(const mlr::Array<T>& v);
template T length(const mlr::Array<T>& v);

template T var(const mlr::Array<T>& v);
template T trace(const mlr::Array<T>& v);

template mlr::Array<T> log(const mlr::Array<T>& v);
template mlr::Array<T> exp(const mlr::Array<T>& v);
template mlr::Array<T> atan(const mlr::Array<T>& v);
template mlr::Array<T> pow(const mlr::Array<T>& v,T);

template T minDiag(const mlr::Array<T>& v);

template T product(const mlr::Array<T>& v);
#ifndef NOFLOAT
template void innerProduct(mlr::Array<T>& x, const mlr::Array<T>& y, const mlr::Array<T>& z);
#endif
template void outerProduct(mlr::Array<T>& x, const mlr::Array<T>& y, const mlr::Array<T>& z);
template T scalarProduct(const mlr::Array<T>& v, const mlr::Array<T>& w);
template T scalarProduct(const mlr::Array<T>& g, const mlr::Array<T>& v, const mlr::Array<T>& w);

template mlr::Array<T> catCol(const mlr::Array<mlr::Array<T>* >& X);


template void tensorEquation(mlr::Array<T> &X, const mlr::Array<T> &A, const uintA &pickA, const mlr::Array<T> &B, const uintA &pickB, uint sum);
template void tensorPermutation(mlr::Array<T> &Y, const mlr::Array<T> &X, const uintA &Yid);
template void tensorMarginal(mlr::Array<T> &Y, const mlr::Array<T> &X, const mlr::Array<uint> &Yid);
template void tensorMaxMarginal(mlr::Array<T> &Y, const mlr::Array<T> &X, const mlr::Array<uint> &Yid);
template void tensorMarginal_old(mlr::Array<T> &y, const mlr::Array<T> &x, const mlr::Array<uint> &xd, const mlr::Array<uint> &ids);
template void tensorMultiply(mlr::Array<T> &X, const mlr::Array<T> &Y, const mlr::Array<uint> &Yid);
template void tensorMultiply_old(mlr::Array<T> &x, const mlr::Array<T> &y, const mlr::Array<uint> &d, const mlr::Array<uint> &ids);

template void rndInteger(mlr::Array<T>& a, int low, int high, bool add);
template void rndUniform(mlr::Array<T>& a, double low, double high, bool add);
template void rndGauss(mlr::Array<T>& a, double stdDev, bool add);
//template void rndGauss(mlr::Array<T>& a, bool add);
//template mlr::Array<T>& rndGauss(double stdDev, uint dim);
template uint softMax(const mlr::Array<T>& a, mlr::Array<double>& soft, double beta);


#undef T
