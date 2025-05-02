/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "array.h"
#include "graph.h"

namespace H5 { class H5File; }

namespace rai {

//===========================================================================

struct H5_Writer {
  std::shared_ptr<H5::H5File> file;

  H5_Writer(const char* filename);

  template<class T> void add(const char* name, const rai::Array<T>& x);
  void addDict(const char* name, const Graph& dict);
  void addGroup(const char* group);
};

//===========================================================================

struct H5_Reader {
  std::shared_ptr<H5::H5File> file;
  int verbose=0;
  Graph G;

  H5_Reader(const char* filename);
  void readAll();
  template<class T> rai::Array<T> read(const char* name, bool ifExists=false);
  Graph readDict(const char* name, bool ifExists=false);
  bool exists(const char* name);
};

} //namespace
