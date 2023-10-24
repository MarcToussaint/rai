/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//////////// taken from http://stackoverflow.com/questions/4532281/how-to-test-whether-class-b-is-derived-from-class-a
typedef char(&yes)[1];
typedef char(&no)[2];
template <typename B, typename D> struct Host {
  operator B* () const;
  operator D* ();
};
template <typename B, typename D> struct RAI_is_base_of {
  template <typename T>
  static yes check(D*, T);
  static no check(B*, int);
  static const bool value = sizeof(check(Host<B, D>(), int())) == sizeof(yes);
};
///////////////STOP

struct RootType { virtual ~RootType() {} }; ///< if types derive from RootType, more tricks are possible

template <class T> rai::Array<T*> Graph::getDerivedValues() {
  rai::Array<T*> ret;
  for(Node* n: (*this)) {
    if(n->is_derived_from_RootType()) {
      T* val= dynamic_cast<T*>(((Node_typed<RootType>*)n)->value);
      if(val) ret.append(val);
    }
  }
  return ret;
}

template <class T> NodeL Graph::getDerivedNodes() {
  NodeL ret;
  for(Node* n: (*this)) {
    if(n->is_derived_from_RootType()) {
      T* val= dynamic_cast<T*>(&((Node_typed<RootType>*)n)->value);
      if(val) ret.append(n);
    }
  }
  return ret;
}

virtual bool is_derived_from_RootType() const {
  return RAI_is_base_of<RootType, T>::value;
}

