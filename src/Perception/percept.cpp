#include "percept.h"


Percept::Percept(){
  transform.setZero();
  frame.setZero();
}

void Percept::write(ostream& os) const{
  os <<" trans=" <<transform <<" frame=" <<frame <<" type=" <<type;
}

//============================================================================

Cluster::Cluster(arr mean, arr points, std::string frame_id)
  : mean(mean),
    points(points),
    frame_id(frame_id) {
  this->type = Type::cluster;
}


Cluster::Cluster(const Cluster& obj) {
  this->frame_id = obj.frame_id;
  this->mean = obj.mean;
  this->points = obj.points;
  this->type = obj.type;
  this->relevance = obj.relevance;
  this->id = obj.id;
  this->transform = obj.transform;
  this->frame = obj.frame;
}

double Cluster::idMatchingCost(const Percept& other){
  if(other.type!=cluster) return -1.;
  mlr::Vector diff = (this->frame * mlr::Vector(this->mean)) -
                     (dynamic_cast<const Cluster*>(&other)->frame * mlr::Vector(dynamic_cast<const Cluster*>(&other)->mean));
  return diff.length();
}

void Cluster::write(ostream& os) const{
  os <<"cluster_" <<id <<": mean=" <<mean;
  Percept::write(os);
}

//============================================================================

Plane::Plane(arr normal, arr center, arr hull, std::string frame_id)
  : normal(normal),
    center(center),
    hull(hull),
    frame_id(frame_id){
  this->type = Type::plane;
}

Plane::Plane(const Plane& obj){
  this->frame_id = obj.frame_id;
  this->normal = obj.normal;
  this->center = obj.center;
  this->hull = obj.hull;
  this->type = obj.type;
  this->relevance = obj.relevance;
  this->id = obj.id;
  this->transform = obj.transform;
  this->frame = obj.frame;
}

double Plane::idMatchingCost(const Percept& other){
  if(other.type!=plane) return -1.;
  mlr::Vector diff = (this->frame * mlr::Vector(this->center)) -
                     (dynamic_cast<const Plane*>(&other)->frame * mlr::Vector(dynamic_cast<const Plane*>(&other)->center));
  return diff.length();

}

void Plane::write(ostream& os) const{
  os <<"plane_" <<id <<": center=" <<center <<" normal=" <<normal;
  Percept::write(os);
}

//============================================================================

Alvar::Alvar(std::string frame_id)
  : frame_id(frame_id) {
  this->type = Type::alvar;
}


Alvar::Alvar(const Alvar& obj){
  this->frame_id = obj.frame_id;
  this->type = obj.type;
  this->relevance = obj.relevance;
  this->id = obj.id;
  this->transform = obj.transform;
  this->frame = obj.frame;
}

double Alvar::idMatchingCost(const Percept& other){
  if(other.type!=alvar) return -1.;
  mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const Alvar*>(&other)->frame * dynamic_cast<const Alvar*>(&other)->transform.pos);
  return dist.length();
}

void Alvar::write(ostream& os) const{
  os <<"alvar_" <<id <<":";
  Percept::write(os);
}


