#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <map>

#define G4DATA_MAKE_KEY(b, s) STRING((b) << "::" << (s))

struct sG4Data {
  std::map<String, intA> obj_inds;
  std::map<std::pair<int, int>, int> hub_sens_ind;
  arr data;

  intA getObjectIndex(const char *obj);
  int getHubSensIndex(int hid, int sid);
};

//==============================================================================

G4Data::G4Data() {
  s = new sG4Data();
}

G4Data::~G4Data() {
  delete s;
}

void G4Data::addSensor(const char *bname, const char *sname, int hid, int sid) {
  int i = s->getHubSensIndex(hid, sid);
  if(i == -1) HALT(STRING("No sensor " << hid << ":" << sid << " available."));

  String name = G4DATA_MAKE_KEY(bname, sname);
  if(s->obj_inds.count(name) == 0)
    s->obj_inds[name]; // creates entry automatically
  s->obj_inds[name].append(i);

  name = STRING(bname);
  if(s->obj_inds.count(name) == 0)
    s->obj_inds[name]; // creates entry automatically
  s->obj_inds[name].append(i);
}

void G4Data::addSensor(const char *bname, int hid, int sid) {
  int i = s->getHubSensIndex(hid, sid);
  if(i == -1) HALT(STRING("No sensor " << hid << ":" << sid << " available."));

  String name = STRING(bname);
  if(s->obj_inds.count(name) == 0)
    s->obj_inds[name]; // creates entry automatically
  s->obj_inds[name].append(i);
}

void G4Data::loadData(const char *fname) {
  ifstream fil;
  MT::open(fil, fname);
  arr x;

  fil >> x;
  x.reshape(x.N/2, 2);
  for(uint i = 0; i < x.N/2; i++)
    s->hub_sens_ind[std::pair<int, int>(x(i, 0), x(i, 1))] = i;

  uint t = 0;
  for(; ; t++) {
    fil >> x;
    if(!x.N || !fil.good()) break;
    s->data.append(x);
  }
  s->data.reshape(t, s->data.N/t/7, 7);
  cout << s->data.d0 << " frames " << s->data.d1 << " sensor loaded" << endl;
}

arr G4Data::query(const char *bname, const char *sname, int t) {
  String name = G4DATA_MAKE_KEY(bname, sname);
  return query((const char *)name, t);
}

arr G4Data::query(const char *bname, const char *sname) {
  String name = G4DATA_MAKE_KEY(bname, sname);
  return query((const char *)name);
}

arr G4Data::query(const char *bname, int t) {
  arr rData;

  intA a = s->obj_inds[STRING(bname)];
  rData.resize(1, a.N, s->data.d2);

  for(uint j = 0; j < rData.d1; j++)
    for(uint k = 0; k < rData.d2; k++)
      rData(0, j, k) = s->data(t, a(j), k);
  return rData;
}

arr G4Data::query(const char *bname) {
  arr rData;

  intA a = s->obj_inds[STRING(bname)];
  rData.resize(s->data.d0, a.N, s->data.d2);

  for(uint i = 0; i < rData.d0; i++)
    for(uint j = 0; j < rData.d1; j++)
      for(uint k = 0; k < rData.d2; k++)
        rData(i, j, k) = s->data(i, a(j), k);
  return rData;
}

arr G4Data::query(int t) {
  return s->data.sub(t, t, 0, -1, 0, -1);
}

arr G4Data::query() {
  return s->data;
}

//==============================================================================

intA sG4Data::getObjectIndex(const char *obj) {
  String s = STRING(obj);
  if(obj_inds.count(s) == 0)
    return -1;
  return obj_inds.find(s)->second;
}

int sG4Data::getHubSensIndex(int hid, int sid) {
  std::pair<int, int> p(hid, sid);
  if(hub_sens_ind.count(p) == 0)
    return -1;
  return hub_sens_ind.find(p)->second;
}

