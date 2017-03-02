#pragma once
#include <Core/array.h>
#include <Kin/kin.h>

struct FilterObject {
  enum FilterObjectType { cluster, plane, alvar, optitrackmarker, optitrackbody };

  const std::string type_name() { return m.at(this->type); }

  uint id;
  double relevance = 1;
  mlr::Transformation transform;
  mlr::Transformation frame;
  FilterObjectType type;
  FilterObject();
  virtual ~FilterObject(){}

  virtual double idMatchingCost(const FilterObject& other) = 0;
  virtual void write(ostream& os) const;

private:
  std::map<FilterObjectType, std::string> m = {
    std::make_pair(FilterObjectType::cluster, "cluster"),
    std::make_pair(FilterObjectType::alvar, "alvar"),
    std::make_pair(FilterObjectType::plane, "plane"),
    std::make_pair(FilterObjectType::optitrackmarker, "optitrack_marker"),
    std::make_pair(FilterObjectType::optitrackbody, "optitrack_body")
  };

};

struct Cluster:FilterObject {
  arr mean;
  arr points;
  std::string frame_id;

  Cluster(arr mean, arr points, std::string frame_id);
  Cluster(const Cluster &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
};

struct Plane:FilterObject {
  arr normal;
  arr center; // not really necesarily center - but at least a point on the plane
  arr hull;
  std::string frame_id;

  Plane(arr normal, arr center, arr hull, std::string frame_id);
  Plane(const Plane &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
};

struct Alvar:FilterObject {
  std::string frame_id;

  Alvar(std::string frame_id);
  Alvar(const Alvar &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
};



struct OptitrackMarker:FilterObject {
  std::string frame_id;

  OptitrackMarker(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = FilterObjectType::optitrackmarker;
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

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=optitrackmarker) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackMarker*>(&other)->frame * dynamic_cast<const OptitrackMarker*>(&other)->transform.pos);
    return dist.length();
  }

//  virtual void write(ostream& os) const{
//    os <<"alvar" <<id <<": mean=" <<mean;
//    FilterObject::write(os);
//  }
};

struct OptitrackBody:FilterObject {
  std::string frame_id;

  OptitrackBody(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = FilterObjectType::optitrackbody;
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

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=optitrackbody) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackBody*>(&other)->frame * dynamic_cast<const OptitrackBody*>(&other)->transform.pos);
    return dist.length();
  }

//  virtual void write(ostream& os) const{
//    os <<"alvar" <<id <<": mean=" <<mean;
//    FilterObject::write(os);
//  }
};

//struct Plane:FilterObject {
////  Plane() {}
//  Plane(arr mean,
//        arr points,
//        std::string frame_id):
//      mean(mean),
//      points(points),
//      frame_id(frame_id)
//  {}
//  arr mean;
//  arr points;
//  std::string frame_id;
//};


typedef mlr::Array<FilterObject*> FilterObjects;


