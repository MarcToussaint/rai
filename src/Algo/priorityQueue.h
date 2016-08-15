#pragma once

#include <Core/array.h>

template<class T> struct PriorityQueueEntry{
  double p;
  T x;

  void write(std::ostream& os) const{ os <<'[' <<p <<": " <<*x <<']' <<endl; }
  static bool cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b);
};
template<class T> stdOutPipe(PriorityQueueEntry<T>)

template<class T>
bool PriorityQueueEntry<T>::cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b){
  return a.p <= b.p;
};

template<class T> struct PriorityQueue : mlr::Array<PriorityQueueEntry<T> > {
  PriorityQueue(){
    mlr::Array<PriorityQueueEntry<T> >::memMove = true;
  }

  void add(double p, const T& x){
    PriorityQueueEntry<T> e = {p, x};
    mlr::Array<PriorityQueueEntry<T> >::insertInSorted(e, PriorityQueueEntry<T>::cmp);
  }

  T pop(){
    T x=mlr::Array<PriorityQueueEntry<T> >::first().x;
    remove(0);
    return x;
  }

  //  void add_replace(double p, const T& x){
  //    PriorityQueueEntry<T> e = {p, x};
  //    uint pos = rankInSorted(e, cmp_PriorityQueueEntry);
  //    PriorityQueueEntry<T>& e_after = elem(pos);
  //    if(e_after.x==e.x){
  //      if(e_after)
  //      e_after.p =
  //  }
};
