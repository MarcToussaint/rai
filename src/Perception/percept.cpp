#include "percept.h"
#include <Gui/opengl.h>

template<> const char* mlr::Enum<Percept::Type>::names []=
  { "cluster", "plane", "alvar", "optitrackmarker", "optitrackbody", NULL };


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

Plane::Plane(){
  this->type = Type::plane;
}

Plane::Plane(const arr& normal, const arr& center, const mlr::Mesh& hull, const std::string& frame_id)
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
  const Plane* otherPlane = dynamic_cast<const Plane*>(&other);
  if(!otherPlane){ MLR_MSG("WHY?????"); return -1.; }
  CHECK(otherPlane,"");
  mlr::Vector diff = (this->frame * mlr::Vector(this->center)) -
                     (otherPlane->frame * mlr::Vector(otherPlane->center));
  return diff.length();

}

void Plane::write(ostream& os) const{
  os <<"plane_" <<id <<": center=" <<center <<" normal=" <<normal;
  Percept::write(os);
}

void Plane::syncWith(mlr::KinematicWorld &K){
  mlr::String plane_name = STRING("perc_" << id);

  mlr::Body *body = K.getBodyByName(plane_name, false);
  if (not body) {
    //cout << plane_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(K);
    body->name = plane_name;
    mlr::Shape *shape = new mlr::Shape(K, *body);
    shape->name = plane_name;
    shape->type = mlr::ST_pointCloud;
    shape = new mlr::Shape(K, *body);
    shape->name = plane_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
//    stored_planes.append(id);
  }
  body->X = frame * transform;

  //frame = body->X;
  body->shapes(0)->mesh = hull;
//  body->shapes(0)->mesh.makeTriangleFan();

  mlr::Vector cen = body->shapes(0)->mesh.center();
  body->X.addRelativeTranslation(cen);
  body->shapes(0)->rel.rot = body->X.rot;
  body->X.rot.setZero();

//  ((Plane*)plane)->transform = body->X;
  //((plane*)plane)->mean = ARR(cen.x, cen.y, cen.z);
  /* If we change the mean, we compare the transformed mean to an untransformed mean later...*/
}

void Plane::glDraw(OpenGL& gl){
  hull.glDraw(gl);

//  if(hull.C.N==3){
//    glColor(hull.C(0), hull.C(1), hull.C(2), 1.f);
//  }

  mlr::Transformation t;
  t.pos.set(center);
  t.rot.setDiff(Vector_x, normal);
  glPushMatrix();
  glTransform(t);
  glLineWidth(3);
  glScalef(.3, .3, .3);
  glDrawAxis();
  glPopMatrix();
  glLineWidth(1);

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

//============================================================================

void Cluster::syncWith(mlr::KinematicWorld& K){
  mlr::String cluster_name = STRING("perc_" << id);

  mlr::Body *body = K.getBodyByName(cluster_name, false);
  if (not body) {
    //cout << cluster_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(K);
    body->name = cluster_name;
    mlr::Shape *shape = new mlr::Shape(K, *body);
    shape->name = cluster_name;
    shape->type = mlr::ST_pointCloud;
    shape = new mlr::Shape(K, *body);
    shape->name = cluster_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
//    stored_clusters.append(id);
  }
  body->X = frame;
  //frame = body->X;
  body->shapes(0)->mesh.V = points;

  mlr::Vector cen = body->shapes(0)->mesh.center();
  body->X.addRelativeTranslation(cen);
  body->shapes(0)->rel.rot = body->X.rot;
  body->X.rot.setZero();

  transform = body->X;
  //((Cluster*)cluster)->mean = ARR(cen.x, cen.y, cen.z);
  /* If we change the mean, we compare the transformed mean to an untransformed mean later...*/
}

//============================================================================

void Alvar::syncWith(mlr::KinematicWorld& K){
  mlr::String alvar_name = STRING("perc_" << id);

  mlr::Body *body = K.getBodyByName(alvar_name, false);
  if (not body) {
//    cout << alvar_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(K);
    body->name = alvar_name;
    mlr::Shape *shape = new mlr::Shape(K, *body);
    shape->name = alvar_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
//    stored_alvars.append(id);
  }

  body->X = frame * transform;
  body->shapes.first()->X = body->X;
}

void OptitrackBody::syncWith(mlr::KinematicWorld &K){
  mlr::String optitrackbody_name = STRING("perc_" << id);

  mlr::Body *body = K.getBodyByName(optitrackbody_name, false);
  if (not body) {
    cout << optitrackbody_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(K);
    body->name = optitrackbody_name;
    mlr::Shape *shape = new mlr::Shape(K, *body);
    shape->name = optitrackbody_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .1;
//    stored_optitrackbodies.append(id);
  }

  body->X = frame * transform;
  body->shapes.first()->X = body->X;
}

void OptitrackMarker::syncWith(mlr::KinematicWorld &K){
  mlr::String optitrackmarker_name = STRING("perc_" << id);

  mlr::Body *body = K.getBodyByName(optitrackmarker_name, false);
  if (not body) {
    cout << optitrackmarker_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(K);
    body->name = optitrackmarker_name;
    mlr::Shape *shape = new mlr::Shape(K, *body);
    shape->name = optitrackmarker_name;
    shape->type = mlr::ST_sphere;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .03;
//    stored_optitrackmarkers.append(id);
  }

  body->X = frame * transform;

  //((Alvar*)alvar)->transform = body->X;
}
