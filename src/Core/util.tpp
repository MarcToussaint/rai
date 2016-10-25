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

#ifndef MLR_util_t_cpp
#define MLR_util_t_cpp

#include "util.h"
#include <map>
#include <string>
#include <sstream>
#include <string.h>
#include <iomanip>
#ifndef MLR_MSVC
#  include <unistd.h>
#endif


bool getParameterFromGraph(const std::type_info& type, void* data, const char* key);


namespace mlr {
/** @brief Search for a command line option \c -tag and, if found, pipe the
 next command line option into \c value by the
 \c operator>>(istream&, type&). Returns false on failure. */
template<class T>
bool getFromCmdLine(T& x, const char *tag) {
  char *opt=getCmdLineArgument(tag);
  if(!opt) return false;
  std::istringstream s(opt);
  s >>x;
  if(s.fail()) HALT("error when reading parameter from command line: " <<tag);
  return true;
}



template<class T>
bool getParameterBase(T& x, const char *tag, bool hasDefault, const T* Default) {
#if 1
  if(getParameterFromGraph(typeid(T), &x, tag)){
    LOG(3) <<std::setw(20) <<tag <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (graph!)";
    return true;
  }
#else
  if(getFromMap<T>(x, tag)) {
    LOG(3) <<std::setw(20) <<tag <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (map!)";
    return true;
  }
  
  if(getFromCmdLine<T>(x, tag)) {
    LOG(3) <<std::setw(20) <<tag <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (cmd line!)";
    return true;
  }
  
  if(getFromCfgFile<T>(x, tag)) {
    LOG(3) <<std::setw(20) <<tag <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"]";
    return true;
  }
#endif

  if(hasDefault) {
    if(Default) {
      x=*Default;
      LOG(3) <<std::setw(20) <<tag <<" = " <<std::setw(5) <<x <<" [" <<typeid(x).name() <<"] (default!)";
    }
    return false;
  }
  
  HALT("could not initialize parameter `" <<tag
       <<"': parameter has no default;\n     either use command option `-"
       <<tag <<" ...' or specify `"
       <<tag <<"= ...' in the config file (which might be `MT.cfg')");
}

template<class T> T getParameter(const char *tag) {
  T x;
  getParameterBase<T>(x, tag, false, (T*)NULL);
  return x;
}
template<class T> T getParameter(const char *tag, const T& Default) {
  T x;
  getParameterBase<T>(x, tag, true, &Default);
  return x;
}
template<class T> void getParameter(T& x, const char *tag, const T& Default) {
  getParameterBase<T>(x, tag, true, &Default);
}
template<class T> void getParameter(T& x, const char *tag) {
  getParameterBase(x, tag, false, (T*)NULL);
}
template<class T> bool checkParameter(const char *tag) {
  T x;
  return getParameterBase(x, tag, true, (T*)NULL);
}
}

#endif
