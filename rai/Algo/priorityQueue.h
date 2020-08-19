/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

template<class T> struct PriorityQueueEntry {
  double p;
  T x;

  void write(std::ostream& os) const { os <<'[' <<p <<": " <<*x <<']' <<endl; }
  static bool cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b);
};

template<class T> stdOutPipe(PriorityQueueEntry<T>)

template<class T>
bool PriorityQueueEntry<T>::cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b) {
  return a.p <= b.p;
}

template<class T> bool operator<=(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b) {
  return a.p <= b.p;
}

template<class T> struct PriorityQueue : rai::Array<PriorityQueueEntry<T>> {
  PriorityQueue() {
    rai::Array<PriorityQueueEntry<T>>::memMove = true;
  }

  void add(double p, const T& x, bool fromBackIfEqual=false) { //'fromBack=true' makes it a FIFO (breadth first search); otherwise LIFO (depth first search)
    PriorityQueueEntry<T> e = {p, x};
    rai::Array<PriorityQueueEntry<T>>::insertInSorted(e, PriorityQueueEntry<T>::cmp, fromBackIfEqual);
  }

  T pop() {
    T x=rai::Array<PriorityQueueEntry<T>>::first().x;
    rai::Array<PriorityQueueEntry<T>>::remove(0);
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
