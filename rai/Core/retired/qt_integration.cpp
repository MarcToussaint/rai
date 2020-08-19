/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "qt_integration.h"

QDebug operator<<(QDebug dbg, const rai::String& s) {
  dbg.nospace() <<s;
  return dbg.space();
}
