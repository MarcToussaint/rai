#include "defines.h"

#include <iostream>
#include <sstream>
#include "util.h"

ostream& stdCout(){ return std::cout; }

std::ostream& rai::LogToken::os() {
  if(!msg) msg = new String;
  return msg->stream();
}

namespace rai{
  String& errStringStream(){
    static String _errString;
    return _errString;
  }
  const char* errString(){ return errStringStream().p; }
}
