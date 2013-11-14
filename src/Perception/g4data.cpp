#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/keyValueGraph.h>

struct sG4Data {
  KeyValueGraph G;
  uint numS, numT;
  intA itohs, hstoi;
  arr data;
  boolA missing;
  MT::Array<intA> missingno;
  MT::Array<intA> missingt;

  sG4Data(): numS(0), numT(0) {};
};

//==============================================================================

G4Data::G4Data() {
  s = new sG4Data();
}

G4Data::~G4Data() {
  delete s;
}

void G4Data::loadData(const char *meta_fname, const char *poses_fname, bool interpolate) {
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

  bool m;
  boolA pm(s->numS);
  pm.setZero(false);
  s->missingno.resize(s->numS);
  s->missingt.resize(s->numS);
  uint T;
  for(T = 0; ; T++) {
    fil >> x;
    if(!x.N || !fil.good()) break;

    for(uint i = 0; i < s->numS; i++) {
      hsi = s->itohs(i);
      if(hsi != -1) {
        s->data.append(x[hsi]);
        m = norm(x[hsi]) == 0;
        s->missing.append(m);
        if(m && pm(i))
          s->missingno(i).last()++;
        else if(m && !pm(i)) {
          s->missingno(i).append(1);
          s->missingt(i).append(T);
        }
        pm(i) = m;
      }
    }
  }
  s->numT = T;
  s->data.reshape(T, s->data.N/T/7, 7);
  s->missing.reshape(T, s->data.N/T/7);

  if(interpolate) { // interpolating missing measures
    for(uint i = 0; i < s->numS; i++) {
      for(uint j = 0; j < s->missingno(i).N; j++) {
        uint t = s->missingt(i).elem(j);
        uint no = s->missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            s->data[tt][i] = s->data[no][i];
        else if(t+no < T) { // interpolate between t-1 and t+missingno(i)
          arr s0 = s->data[t-1][i];
          arr sF = s->data[t+no][i];
          arr diff = sF - s0;
          for(uint tt = 0; tt < no; tt++)
            s->data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
        }
        else // set all equal to last
          for(uint tt = 0; tt < no; tt++)
            s->data[T-tt-1][i] = s->data[T-no-1][i];
      }
    }
  }
}

int G4Data::getNumTimesteps() const {
  return s->numT;
}

int G4Data::getNumSensors(const char *key) const {
  if(key == NULL)
    return s->numS;
  return s->G.getTypedValues<KeyValueGraph>(key).N;
}

boolA G4Data::getMissing() const {
  return s->missing;
}

MT::Array<intA> G4Data::getMissingNo() const {
  return s->missingno;
}

MT::Array<intA> G4Data::getMissingT() const {
  return s->missingt;
}

arr G4Data::query(uint t, const char *key) const {
  if(key == NULL)
    return s->data[t].reshape(s->numS, 7);

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
  ret.reshape(sensors.N, 7);

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

arr G4Data::queryPos(uint t, const char *key) const {
  if(key == NULL)
    return s->data[t].sub(0, -1, 0, 2).reshape(1, s->numS, 3);

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
  ret.sub(0, -1, 0, 2).reshape(1, sensors.N, 3);

  return ret;
}

arr G4Data::queryPos(const char *key) const {
  if(key == NULL)
    return s->data.sub(0, -1, 0, -1, 0, 2);

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
  ret.sub(0, -1, 0, -1, 0, 2).reshape(s->numT, sensors.N, 3);

  return ret;
}

arr G4Data::queryQuat(uint t, const char *key) const {
  if(key == NULL)
    return s->data[t].sub(0, -1, 3, -1).reshape(s->numS, 4);

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
  ret.sub(0, -1, 3, -1).reshape(sensors.N, 4);

  return ret;
}

arr G4Data::queryQuat(const char *key) const {
  if(key == NULL)
    return s->data.sub(0, -1, 0, -1, 3, -1);

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
  ret.sub(0, -1, 0, -1, 3, -1).reshape(s->numT, sensors.N, 3);

  return ret;
}
