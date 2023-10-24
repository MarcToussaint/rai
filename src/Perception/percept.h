/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Geo/mesh.h"
#include <memory>

struct Percept;
typedef std::shared_ptr<Percept> PerceptPtr;
typedef rai::Array<PerceptPtr> PerceptL;

struct Percept : GLDrawer {
  enum Type { PT_cluster, PT_plane, PT_box, PT_mesh, PT_alvar, PT_optitrackmarker, PT_optitrackbody, PT_end };

  uint id = 0;
  int bodyId = -1;
  rai::Enum<Type> type;
  double precision = 1.;
  rai::Transformation pose;
  rai::Vector com=0;
  std::string frame_id;
  Percept(Type type);
  Percept(Type type, const rai::Transformation& _pose);
  virtual ~Percept() {}

  virtual void syncWith(rai::Configuration& K) = 0;
  virtual double idMatchingCost(const Percept& other);
  virtual double fuse(PerceptPtr& other);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL&) { NIY }
  virtual Percept* newClone() const = 0;

//private:
//  std::map<Type, std::string> m = {
//    std::make_pair(Type::cluster, "cluster"),
//    std::make_pair(Type::alvar, "alvar"),
//    std::make_pair(Type::plane, "plane"),
//    std::make_pair(Type::optitrackmarker, "optitrack_marker"),
//    std::make_pair(Type::optitrackbody, "optitrack_body")
//  };
};
stdOutPipe(Percept)

struct PercCluster : Percept {
  arr mean;
  arr points;

  PercCluster(arr mean, arr points, std::string frame_id);

  virtual void syncWith(rai::Configuration& K);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
  virtual Percept* newClone() const { return new PercCluster(*this); }
};

struct PercMesh : Percept {
  rai::Mesh mesh;

  PercMesh() : Percept(PT_mesh) {}
  PercMesh(const rai::Mesh& mesh) : Percept(PT_mesh), mesh(mesh) {}

  virtual void syncWith(rai::Configuration& K);
//  virtual double idMatchingCost(const Percept& other){ return 0.; }
  virtual double fuse(PerceptPtr& other);
  virtual void write(ostream& os) const { os <<"#V=" <<mesh.V.N; }
  virtual void glDraw(OpenGL& gl) { mesh.glDraw(gl); }
  virtual Percept* newClone() const { return new PercMesh(*this); }
};

struct PercPlane : Percept {
  rai::Mesh hull;

  PercPlane(const rai::Transformation& t, const rai::Mesh& hull);

  virtual void syncWith(rai::Configuration& K);
  virtual double idMatchingCost(const Percept& other);
  virtual double fuse(PerceptPtr& other);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL&);
  virtual Percept* newClone() const { return new PercPlane(*this); }
};

struct PercBox : Percept {
  arr size;
  arr color;

  PercBox(const rai::Transformation& t, const arr& size, const arr& color);

  virtual void syncWith(rai::Configuration& K);
//  virtual double idMatchingCost(const Percept& other) { return 0.; }
  virtual double fuse(PerceptPtr& other);
  virtual void write(ostream& os) const { Percept::write(os); os <<"size=" <<size; }
  virtual void glDraw(OpenGL&);
  virtual Percept* newClone() const { return new PercBox(*this); }
};

struct PercAlvar : Percept {
  uint alvarId;

  PercAlvar(uint alvarId, std::string _frame_id);

  virtual void syncWith(rai::Configuration& K);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
  virtual Percept* newClone() const { return new PercAlvar(*this); }
};

struct OptitrackMarker : Percept {
  OptitrackMarker(std::string _frame_id)
    : Percept(PT_optitrackmarker) {
    frame_id = _frame_id;
  }

  OptitrackMarker(const OptitrackMarker& obj)
    : Percept(PT_optitrackmarker) {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->precision = obj.precision;
    this->id = obj.id;
    this->pose = obj.pose;
  }

  virtual void syncWith(rai::Configuration& K);
//  virtual double idMatchingCost(const Percept& other){
//    if(other.type!=PT_optitrackmarker) return -1.;
//    rai::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackMarker*>(&other)->frame * dynamic_cast<const OptitrackMarker*>(&other)->transform.pos);
//    return dist.length();
//  }
  virtual Percept* newClone() const { return new OptitrackMarker(*this); }
};

struct OptitrackBody : Percept {
  OptitrackBody(std::string _frame_id)
    : Percept(Type::PT_optitrackbody) {
    frame_id = _frame_id;
  }

  OptitrackBody(const OptitrackBody& obj)
    : Percept(Type::PT_optitrackbody) {
    this->frame_id = obj.frame_id;
    this->precision = obj.precision;
    this->id = obj.id;
    this->pose = obj.pose;
  }

  virtual void syncWith(rai::Configuration& K);
//  virtual double idMatchingCost(const Percept& other){
//    if(other.type!=PT_optitrackbody) return -1.;
//    rai::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackBody*>(&other)->frame * dynamic_cast<const OptitrackBody*>(&other)->transform.pos);
//    return dist.length();
//  }
  virtual Percept* newClone() const { return new OptitrackBody(*this); }
};

