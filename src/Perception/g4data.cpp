#include "g4data.h"
#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/keyValueGraph.h>

struct sG4Data {
  KeyValueGraph G;
  uint numS, numF;
  intA itohs, hstoi;
  StringA names;
  arr data;
  boolA missing;
  MT::Array<intA> missingno;
  MT::Array<intA> missingf;

  sG4Data(): numS(0), numF(0) {};
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
    s->names.append(*sensors(i)->getValue<String>("name"));
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
  s->missingf.resize(s->numS);
  uint F;
  for(F = 0; ; F++) {
    fil >> x;
    // TODO this is to test whether the quaternions are normalized..
    /*
    cout << x.getDim() << endl;
    for(uint i = 0; i < s->numS; i++) {
      ors::Quaternion q(x(i, 3), x(i, 4), x(i, 5), x(i, 6));
      cout << q << endl;
      cout << " - norm: " << (q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z) << endl;
    }
    */

    if(!x.N || !fil.good()) break;

    for(uint i = 0; i < s->numS; i++) {
      hsi = s->itohs(i);
      if(hsi != -1) {
        s->data.append(x[hsi]);
        m = length(x[hsi]) == 0;
        s->missing.append(m);
        if(m && pm(i))
          s->missingno(i).last()++;
        else if(m && !pm(i)) {
          s->missingno(i).append(1);
          s->missingf(i).append(F);
        }
        pm(i) = m;
      }
    }
  }
  s->numF = F;
  s->data.reshape(F, s->data.N/F/7, 7);
  s->missing.reshape(F, s->data.N/F/7);

  if(interpolate) { // interpolating missing measures
    for(uint i = 0; i < s->numS; i++) {
      for(uint j = 0; j < s->missingno(i).N; j++) {
        uint t = s->missingf(i).elem(j);
        uint no = s->missingno(i).elem(j);
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            s->data[tt][i] = s->data[no][i];
        else if(t+no < F) { // interpolate between t-1 and t+missingno(i)
          arr s0 = s->data[t-1][i];
          arr sF = s->data[t+no][i];
          ors::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
          ors::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
          ors::Quaternion qt;

          arr diff = sF - s0;
          for(uint tt = 0; tt < no; tt++) {
            s->data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
            qt.setInterpolate((tt+1.)/(no+1.), q0, qF);
            // TODO remove couts
            //cout << " - norm: " << (qt.w*qt.w+qt.x*qt.x+qt.y*qt.y+qt.z*qt.z) << endl;
            //cout << " - w: " << qt.w << endl;
            s->data(t+tt, i, 3) = qt.w;
            s->data(t+tt, i, 4) = qt.x;
            s->data(t+tt, i, 5) = qt.y;
            s->data(t+tt, i, 6) = qt.z;
          }
        }
        else // set all equal to last
          for(uint tt = 0; tt < no; tt++)
            s->data[F-tt-1][i] = s->data[F-no-1][i];
      }
    }
  }
}

StringA& G4Data::getNames() const {
  return s->names;
}

String& G4Data::getName(uint i) const {
  CHECK(i < s->names.N, "Invalid index.");
  return s->names(i);
}

uint G4Data::getNumFrames() const {
  return s->numF;
}

uint G4Data::getNumSensors(const char *key) const {
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

MT::Array<intA> G4Data::getMissingF() const {
  return s->missingf;
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

  for(uint t = 0; t < s->numF; t++)
    for(uint i = 0; i < sensors.N; i++)
      ret.append(s->data.sub(t, t, iv(i), iv(i), 0, -1));
  ret.reshape(s->numF, sensors.N, 7);

  return ret;
}

arr G4Data::queryPos(uint t, const char *key) const {
  if(key == NULL)
    return s->data[t].cols(0, 3).reshape(s->numS, 3);

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
  return ret.reshape(sensors.N, 7).cols(0, 3);
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

  for(uint t = 0; t < s->numF; t++)
    for(uint i = 0; i < sensors.N; i++)
      ret.append(s->data.sub(t, t, iv(i), iv(i), 0, 2));
  return ret.reshape(s->numF, sensors.N, 3);
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
  return ret.reshape(sensors.N, 7).cols(3, 7);
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

  for(uint t = 0; t < s->numF; t++)
    for(uint i = 0; i < sensors.N; i++)
      ret.append(s->data.sub(t, t, iv(i), iv(i), 0, -1));
  ret.sub(0, -1, 0, -1, 3, -1).reshape(s->numF, sensors.N, 3);

  return ret;
}
