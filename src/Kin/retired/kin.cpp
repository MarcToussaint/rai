/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void rai::KinematicWorld::writePlyFile(const char* filename) const {
  ofstream os;
  rai::open(os, filename);
  uint nT=0, nV=0;
  uint j;
  rai::Mesh* m;
  for(Frame* f: frames) if(f->shape) { nV += f->shape->mesh().V.d0; nT += f->shape->mesh().T.d0; }

  os <<"\
       ply\n\
       format ascii 1.0\n\
       element vertex " <<nV <<"\n\
       property float x\n\
       property float y\n\
       property float z\n\
       property uchar red\n\
       property uchar green\n\
       property uchar blue\n\
       element face " <<nT <<"\n\
       property list uchar int vertex_index\n\
       end_header\n";

  uint k=0;
  rai::Transformation t;
  rai::Vector v;
  Shape* s;
  for(Frame* f: frames) if((s=f->shape)) {
      m = &s->mesh();
      arr col = m->C;
      CHECK_EQ(col.N, 3, "");
      t = s->frame.X;
      if(m->C.d0!=m->V.d0) {
        m->C.resizeAs(m->V);
        for(j=0; j<m->C.d0; j++) m->C[j]=col;
      }
      for(j=0; j<m->V.d0; j++) {
        v.set(m->V(j, 0), m->V(j, 1), m->V(j, 2));
        v = t*v;
        os <<' ' <<v.x <<' ' <<v.y <<' ' <<v.z
           <<' ' <<int(255.f*m->C(j, 0)) <<' ' <<int(255.f*m->C(j, 1)) <<' ' <<int(255.f*m->C(j, 2)) <<endl;
      }
      k+=j;
    }
  uint offset=0;
  for(Frame* f: frames) if((s=f->shape)) {
      m=&s->mesh();
      for(j=0; j<m->T.d0; j++) {
        os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
      }
      offset+=m->V.d0;
    }
}

