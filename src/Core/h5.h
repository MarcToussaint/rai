#pragma once

#include "array.h"
#include "graph.h"

namespace H5{ class H5File; }

//===========================================================================

struct H5_Writer{
  std::shared_ptr<H5::H5File> file;

  H5_Writer(const char* filename);

  template<class T> void add(const char* name, const rai::Array<T>& x);
  void addGroup(const char* group);
};

//===========================================================================

struct H5_Reader {
  std::shared_ptr<H5::H5File> file;
  rai::Graph G;
  int verbose=0;

  H5_Reader(const char* filename);
  void readAll();
  template<class T> rai::Array<T> read(const char* name);
  bool exists(const char* name);
};

