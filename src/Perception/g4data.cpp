#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/keyValueGraph.h>

struct sG4Data {
  KeyValueGraph G;
  uint numS, numT;
  intA itohs, hstoi;
  arr data;

  sG4Data(): numS(0), numT(0) {};
};

//==============================================================================

G4Data::G4Data() {
  s = new sG4Data();
}

G4Data::~G4Data() {
  delete s;
}

void G4Data::loadData(const char *meta_fname, const char *poses_fname) {
  int hid, sid, hsi, hstoiN, hstoiNprev;

  MT::load(s->G, meta_fname);
  MT::Array<KeyValueGraph*> sensors = s->G.getTypedValues<KeyValueGraph>("sensor");
  s->numS = sensors.N;
  hstoiN = 0;
  for(uint i = 0; i < s->numS; i++) {
    hid = *sensors(i)->getValue<double>("hid");
    sid = *sensors(i)->getValue<double>("sid");
    hsi = 3*hid+sid;

    if(hsi+1 > hstoiN) { // s->hstoi.N) { to get rid of warning
      hstoiNprev = s->hstoi.N; // TODO remove prevN and use hsi
      s->hstoi.resizeCopy(hsi+1);
      s->hstoi.subRange(hstoiNprev, hsi).setUni(-1);
      hstoiN = hstoiNprev;
    }
    s->hstoi(hsi) = i;
    s->itohs.append(hsi);
  }

  ifstream fil;
  MT::open(fil, poses_fname);
  arr x;

  for(s->numT = 0; ; s->numT++) {
    fil >> x;
    if(!x.N || !fil.good()) break;

    for(uint i = 0; i < s->numS; i++) {
      hsi = s->itohs(i);
      if(hsi != -1)
        s->data.append(x[hsi]);
    }
  }
  s->data.reshape(s->numT, s->data.N/s->numT/7, 7);
}

int G4Data::getNumTimesteps() const {
  return s->numT;
}

int G4Data::getNumSensors(const char *key) const {
  if(key == NULL)
    return s->numS;
  return s->G.getTypedValues<KeyValueGraph>(key).N;
}

arr G4Data::query(int t, const char *key) const {
  if(key == NULL)
    return s->data[t].reshape(1, s->numS, 7);

  arr ret;
  uint hid, sid, hsi;

  arr tdata = s->data[t];
  MT::Array<KeyValueGraph*> sensors = s->G.getTypedValues<KeyValueGraph>(key);
  for(uint i = 0; i < sensors.N; i++) {
    hid = *sensors(i)->getValue<double>("hid");
    sid = *sensors(i)->getValue<double>("sid");
    hsi = 3*hid+sid;

    ret.append(tdata[s->hstoi(hsi)]);
  }
  ret.reshape(1, sensors.N, 7);

  return ret;
}

arr G4Data::query(const char *key) const {
  if(key == NULL)
    return s->data;

  arr ret;
  uint hid, sid, hsi;
  MT::Array<uint> iv;

  MT::Array<KeyValueGraph*> sensors = s->G.getTypedValues<KeyValueGraph>(key);
  for(uint i = 0; i < sensors.N; i++) {
    hid = *sensors(i)->getValue<double>("hid");
    sid = *sensors(i)->getValue<double>("sid");
    hsi = 3*hid+sid;
    iv.append(s->hstoi(hsi));
  }

  for(uint t = 0; t < s->numT; t++)
    for(uint i = 0; i < sensors.N; i++)
      ret.append(s->data.sub(t, t, iv(i), iv(i), 0, -1));
  ret.reshape(s->numT, sensors.N, 7);

  return ret;
}

