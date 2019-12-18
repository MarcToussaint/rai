/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "qt_integration.h"

QDebug operator<<(QDebug dbg, const rai::String& s) {
  dbg.nospace() <<s;
  return dbg.space();
}
