#pragma once

#include <Core/array.h>

struct G4Data {
  struct sG4Data *s;

  G4Data();
  ~G4Data();

  void addSensor(const char *bname, const char *sname, int hid, int sid);
  void addSensor(const char *bname, int hid, int sid);
  void loadData(const char *fname);

  arr query(const char *bname, const char *sname, int t);
  arr query(const char *bname, const char *sname);
  arr query(const char *bname, int t);
  arr query(const char *bname);
  arr query(int t);
  arr query();
};
