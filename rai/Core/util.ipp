/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_util_t_cpp
#define RAI_util_t_cpp

#include "util.h"
#include "graph.h"

#include <map>
#include <string>
#include <sstream>
#include <string.h>
#include <iomanip>
#ifndef RAI_MSVC
#  include <unistd.h>
#endif

namespace rai {
template<class T>
bool getParameterBase(T& x, const char* key, bool hasDefault, const T* Default) {
  if(getParameters()->get<T>(x, key)) {
    LOG(3) <<std::setw(20) <<key <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (graph!)";
    return true;
  }

  if(hasDefault) {
    if(Default) {
      x=*Default;
      LOG(3) <<std::setw(20) <<key <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (default!)";
    }
    return false;
  }

  HALT("could not initialize parameter `" <<key
       <<"': parameter has no default;\n     either use command option `-"
       <<key <<" ...' or specify `"
       <<key <<"= ...' in the config file (which might be `rai.cfg')");
}

template<class T> T getParameter(const char* tag) {
  T x;
  getParameterBase<T>(x, tag, false, (T*)NULL);
  return x;
}
template<class T> T getParameter(const char* tag, const T& Default) {
  T x;
  getParameterBase<T>(x, tag, true, &Default);
  return x;
}
template<class T> void getParameter(T& x, const char* tag) {
  getParameterBase(x, tag, false, (T*)NULL);
}
template<class T> void getParameter(T& x, const char* tag, const T& Default) {
  getParameterBase<T>(x, tag, true, &Default);
}
template<class T> bool checkParameter(const char* tag) {
  T x;
  return getParameterBase(x, tag, true, (T*)NULL);
}
}

#endif
