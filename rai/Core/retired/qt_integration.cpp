#include "qt_integration.h"

QDebug operator<<(QDebug dbg, const rai::String &s){
  dbg.nospace() <<s;
  return dbg.space();
}
