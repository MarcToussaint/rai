#include "qt_integration.h"

QDebug operator<<(QDebug dbg, const MT::String &s){
  dbg.nospace() <<s;
  return dbg.space();
}
