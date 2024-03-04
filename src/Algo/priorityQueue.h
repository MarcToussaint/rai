/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

template<class T> struct PriorityQueueEntry {
  double f_prio;
  T x;

  void write(std::ostream& os) const { os <<'[' <<f_prio <<": " <<*x <<']'; }
  static bool cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b);
};

template<class T> stdOutPipe(PriorityQueueEntry<T>)

template<class T> bool PriorityQueueEntry<T>::cmp(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b) {
  return a.f_prio <= b.f_prio;
}

template<class T> bool operator<=(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b) {
  return a.f_prio <= b.f_prio;
}

template<class T> struct PriorityQueue : rai::Array<PriorityQueueEntry<T>> {
  PriorityQueue() {
    rai::Array<PriorityQueueEntry<T>>::memMove = 1;
  }

  void add(double f_prio, const T& x, bool FIFOifEqual=false) { //'FIFOifEqual=true' makes it a FIFO (breadth first search); otherwise LIFO (depth first search)
    PriorityQueueEntry<T> e = {f_prio, x};
    rai::Array<PriorityQueueEntry<T>>::insertInSorted(e, PriorityQueueEntry<T>::cmp, FIFOifEqual);
  }

  void append(const T& x) {
    PriorityQueueEntry<T> e = {0., x};
    rai::Array<PriorityQueueEntry<T>>::append(e);
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
