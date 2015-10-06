#include "qt_integration.h"

QDebug operator<<(QDebug dbg, const mlr::String &s){
  dbg.nospace() <<s;
  return dbg.space();
}
