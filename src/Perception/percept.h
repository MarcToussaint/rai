#pragma once

#include <Core/array.h>
#include <Kin/kin.h>

struct Percept;
typedef mlr::Array<Percept*> Percepts;

struct Percept {
  enum Type { cluster, plane, alvar, optitrackmarker, optitrackbody };

  uint id;
  mlr::Enum<Type> type;
  double relevance = 1; //mt: what is relevance?
  mlr::Transformation transform; //mt: really two different transforms??? don't like that
  mlr::Transformation frame;
  Percept();
  virtual ~Percept(){}

  virtual double idMatchingCost(const Percept& other) = 0;
  virtual void write(ostream& os) const;

//private:
//  std::map<Type, std::string> m = {
//    std::make_pair(Type::cluster, "cluster"),
//    std::make_pair(Type::alvar, "alvar"),
//    std::make_pair(Type::plane, "plane"),
//    std::make_pair(Type::optitrackmarker, "optitrack_marker"),
//    std::make_pair(Type::optitrackbody, "optitrack_body")
//  };
};

struct Cluster : Percept {
  arr mean;
  arr points;
  std::string frame_id;

  Cluster(arr mean, arr points, std::string frame_id);
  Cluster(const Cluster &obj);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
};

struct Plane : Percept {
  arr normal;
  arr center; // not really necesarily center - but at least a point on the plane
  arr hull;
  std::string frame_id;

  Plane(arr normal, arr center, arr hull, std::string frame_id);
  Plane(const Plane &obj);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
};

struct Alvar : Percept {
  std::string frame_id;

  Alvar(std::string frame_id);
  Alvar(const Alvar &obj);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
};



struct OptitrackMarker : Percept {
  std::string frame_id;

  OptitrackMarker(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = Type::optitrackmarker;
  }

  OptitrackMarker(const OptitrackMarker &obj)
  {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const Percept& other){
    if(other.type!=optitrackmarker) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackMarker*>(&other)->frame * dynamic_cast<const OptitrackMarker*>(&other)->transform.pos);
    return dist.length();
  }
};

struct OptitrackBody : Percept {
  std::string frame_id;

  OptitrackBody(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = Type::optitrackbody;
  }

  OptitrackBody(const OptitrackBody &obj)
  {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const Percept& other){
    if(other.type!=optitrackbody) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackBody*>(&other)->frame * dynamic_cast<const OptitrackBody*>(&other)->transform.pos);
    return dist.length();
  }
};



