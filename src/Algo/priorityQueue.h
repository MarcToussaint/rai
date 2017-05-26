/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


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
}

template<class T> bool operator<=(const PriorityQueueEntry<T>& a, const PriorityQueueEntry<T>& b){
  return a.p <= b.p;
}

template<class T> struct PriorityQueue : mlr::Array<PriorityQueueEntry<T> > {
  PriorityQueue(){
    mlr::Array<PriorityQueueEntry<T> >::memMove = true;
  }

  void add(double p, const T& x, bool fromBackIfEqual=false){
    PriorityQueueEntry<T> e = {p, x};
    mlr::Array<PriorityQueueEntry<T> >::insertInSorted(e, PriorityQueueEntry<T>::cmp, fromBackIfEqual); //'fromBack' makes it a FIFO (breadth first search); otherwise LIFO (depth first search)
  }

  T pop(){
    T x=mlr::Array<PriorityQueueEntry<T> >::first().x;
    mlr::Array<PriorityQueueEntry<T> >::remove(0);
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
