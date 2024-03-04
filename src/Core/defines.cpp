/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "defines.h"

#include <iostream>
#include <sstream>
#include "util.h"

ostream& stdCout() { return std::cout; }

std::ostream& rai::LogToken::os() {
  if(!msg) msg = new String;
  return msg->stream();
}

namespace rai {
String& errStringStream() {
  static String _errString;
  return _errString;
}
const char* errString() { return errStringStream().p; }

const char* atomicTypeidName(const std::type_info& type) {
  if(type==typeid(int)) return "int32";
  if(type==typeid(int16_t)) return "int16";
  if(type==typeid(uint)) return "uint32";
  if(type==typeid(uint16_t)) return "uint16";
  if(type==typeid(unsigned char)) return "uint8";
  if(type==typeid(float)) return "float32";
  if(type==typeid(double)) return "float64";
  THROW("not yet defined string for type" <<type.name())
  return 0;
}

}
