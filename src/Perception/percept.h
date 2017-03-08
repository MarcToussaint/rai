#pragma once

#include <Core/array.h>
#include <Kin/kin.h>

struct Percept;
typedef mlr::Array<Percept*> PerceptL;

struct Percept : GLDrawer{
  enum Type { PT_cluster, PT_plane, PT_box, PT_mesh, PT_alvar, PT_optitrackmarker, PT_optitrackbody, PT_end };

  uint id = 0;
  mlr::Enum<Type> type;
  double relevance = 1.; //mt: what is relevance? mt: replace -> timeSincePerceived
  mlr::Transformation transform; //mt: really two different transforms??? don't like that
  mlr::Transformation frame;
  std::string frame_id;
  Percept(Type type);
  Percept(Type type, const mlr::Transformation& t);
  virtual ~Percept(){}

  virtual void syncWith(mlr::KinematicWorld& K) = 0;
  virtual double idMatchingCost(const Percept& other) = 0;
  virtual double fuse(const Percept* other);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL&){ NIY }
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

struct Cluster : Percept {
  arr mean;
  arr points;

  Cluster(arr mean, arr points, std::string frame_id);
  Cluster(const Cluster &obj);
  virtual void syncWith(mlr::KinematicWorld& K);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
  virtual Percept* newClone() const{ return new Cluster(*this); }
};

struct PercMesh : Percept {
  mlr::Mesh mesh;

  PercMesh(const mlr::Mesh& mesh) : Percept(PT_mesh), mesh(mesh) {}

  virtual void syncWith(mlr::KinematicWorld& K){ }
  virtual double idMatchingCost(const Percept& other){ return 0.; }
  virtual double fuse(const Percept* other);
  virtual void write(ostream& os) const{ os <<"#V=" <<mesh.V.N; }
  virtual void glDraw(OpenGL& gl){ mesh.glDraw(gl); }
  virtual Percept* newClone() const{ return new PercMesh(*this); }
};

struct Plane : Percept {
  mlr::Mesh hull;

  Plane();
  Plane(const mlr::Transformation& t, const mlr::Mesh& hull);

  virtual void syncWith(mlr::KinematicWorld& K);
  virtual double idMatchingCost(const Percept& other);
  virtual double fuse(const Percept* other);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL&);
  virtual Percept* newClone() const{ return new Plane(*this); }
};

struct PercBox : Percept {
  arr size;

  PercBox(const mlr::Transformation& t, const arr& size);

  virtual void syncWith(mlr::KinematicWorld& K);
  virtual double idMatchingCost(const Percept& other) { return 0.; }
  virtual double fuse(const Percept* other);
  virtual void write(ostream& os) const { Percept::write(os); os <<"size=" <<size; }
  virtual void glDraw(OpenGL&);
  virtual Percept* newClone() const{ return new PercBox(*this); }
};

struct Alvar : Percept {

  Alvar(std::string _frame_id);
  Alvar(const Alvar &obj);

  virtual void syncWith(mlr::KinematicWorld& K);
  virtual double idMatchingCost(const Percept& other);
  virtual void write(ostream& os) const;
  virtual Percept* newClone() const{ return new Alvar(*this); }
};



struct OptitrackMarker : Percept {
  OptitrackMarker(std::string _frame_id)
    : Percept(PT_optitrackmarker){
    frame_id = _frame_id;
  }

  OptitrackMarker(const OptitrackMarker &obj)
    : Percept(PT_optitrackmarker) {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual void syncWith(mlr::KinematicWorld &K);
  virtual double idMatchingCost(const Percept& other){
    if(other.type!=PT_optitrackmarker) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackMarker*>(&other)->frame * dynamic_cast<const OptitrackMarker*>(&other)->transform.pos);
    return dist.length();
  }
  virtual Percept* newClone() const{ return new OptitrackMarker(*this); }
};

struct OptitrackBody : Percept {
  OptitrackBody(std::string _frame_id)
      : Percept(Type::PT_optitrackbody){
    frame_id = _frame_id;
  }

  OptitrackBody(const OptitrackBody &obj)
    : Percept(Type::PT_optitrackbody) {
    this->frame_id = obj.frame_id;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual void syncWith(mlr::KinematicWorld& K);
  virtual double idMatchingCost(const Percept& other){
    if(other.type!=PT_optitrackbody) return -1.;
    mlr::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackBody*>(&other)->frame * dynamic_cast<const OptitrackBody*>(&other)->transform.pos);
    return dist.length();
  }
  virtual Percept* newClone() const{ return new OptitrackBody(*this); }
};



