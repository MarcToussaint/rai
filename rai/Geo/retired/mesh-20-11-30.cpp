bool rai::Mesh::readStlFile(std::istream& is) {
  //first check if binary
  if(rai::parse(is, "solid", true)) { //is ascii
    rai::String name;
    is >>name;
    uint i, k=0, k0;
    double x, y, z;
//    cout <<"reading STL file -- object name '" <<name <<"'..." <<endl;
    V.resize(10000);
    //1st pass
    for(i=0, k=0;; i++) {
      k0=k;
      if(k>V.N-10) V.resizeCopy(2*V.N);
      if(!(i%100)) cout <<"\r" <<i <<' ' <<i*7;
      if(rai::peerNextChar(is)!='f') break;
      is >>PARSE("facet");
      is >>PARSE("normal") >>x >>y >>z;  rai::skip(is);
      is >>PARSE("outer") >>PARSE("loop");      rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   rai::skip(is);
      is >>PARSE("endloop");             rai::skip(is);
      is >>PARSE("endfacet");            rai::skip(is);
      if(!is.good()) {
        RAI_MSG("reading error - skipping facet " <<i <<" (line " <<i*7+2 <<")");
        is.clear();
        cout <<1 <<endl;
        rai::skipUntil(is, "endfacet");
        cout <<2 <<endl;
        k=k0;
      }
    }
    is >>PARSE("endsolid");
    if(!is.good()) RAI_MSG("couldn't read STL end tag (line" <<i*7+2);
    cout <<"... STL file read: #tris=" <<i <<" #lines=" <<i*7+2 <<endl;
    CHECK(!(k%9), "not mod 9..");
    V.resizeCopy(k/3, 3);
    T.resize(k/9, 3);
    for(i=0; i<T.N; i++) { T.elem(i)=i; }
  } else { //is binary
    is.clear();
    is.seekg(0, std::ios::beg);
    char header[80];
    is.read(header, 80);
    uint ntri;
    is.read((char*)&ntri, sizeof(ntri));
    T.resize(ntri, 3);
    floatA Vfloat(3*ntri, 3);
    float normal[3];
    uint16_t att;
    for(uint i=0; i<ntri; i++) {
      is.read((char*)&normal, 3*Vfloat.sizeT);
      is.read((char*)&Vfloat(3*i, 0), 9*Vfloat.sizeT);
      T(i, 0)=3*i+0;  T(i, 1)=3*i+1;  T(i, 2)=3*i+2;
      is.read((char*)&att, 2);
      CHECK_EQ(att, 0, "this stl file is broke");
    }
    copy(V, Vfloat);
  }
  return true;
}

/*void rai::Mesh::getOBJ(char* filename){
  if(!glm){
  glm = glmReadOBJ(filename);
  glmReverseWinding(glm);
  }

  ////glmUnitize(glm);
  glmFacetNormals(glm);
  glmVertexNormals(glm, 90.0);

  // creates a display list for the OBJ
  ////  g._pmodel_displaylist = glmList(glm, GLM_SMOOTH | GLM_MATERIAL);
  }*/

uint& Tni(uint, uint) { static uint dummy; return dummy; } //normal index

rai::String str;

char* strn(std::istream& is) {
  str.read(is, " \n\t\r", " \n\t\r", true); //we once had a character '\d' in there -- for Windows?
  CHECK(is.good(), "could not read line");
  return str.p;
}

/** initialises the ascii-obj file "filename"*/
void rai::Mesh::readObjFile(std::istream& is) {
  // make a first pass through the file to get a count of the number
  // of vertices, normals, texcoords & triangles
  uint nV, nN, nTex, nT;
  nV = nN = nTex = nT = 0;
  int v, n, t;

//  // we only want to parse the relevant subpart/submesh of the mesh therefore
//  // jump to the right position and stop parsing at the right positon.
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);

//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false; !ex;) {
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        rai::skipRestOfLine(is);
        strn(is);
        break;
      case 'v':
        switch(str.p[1]) {
          case '\0': nV++;    rai::skipRestOfLine(is); break;  // vertex
          case 'n':  nN++;    rai::skipRestOfLine(is); break;  // normal
          case 't':  nTex++;  rai::skipRestOfLine(is); break;  // texcoord
          default: HALT("firstPass(): Unknown token '" <<str.p <<"'");  break;
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
        if(strstr(str.p, "//")) {
          // v//n
          CHECK(sscanf(str.p, "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/t
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) nT++;
        } else {
          // v
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) nT++;
        }
        break;

      default:  RAI_MSG("unsupported .obj file tag '" <<str <<"'");  rai::skipRestOfLine(is);  strn(is);  break;
    }
  }

  //allocate memory
  V.resize(nV, 3);
  Vn.resize(nN, 3);
  T.resize(nT, 3);
  Tn.resize(nT, 3);
  if(nTex) Tt.resize(nT, 3);
  //if(nVN) N.resize(nVN, 3);
  if(nTex) tex.resize(nTex, 2);

  // rewind to beginning of file and read in the data this pass
  is.seekg(0);
  is.clear();
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);

  /* on the second pass through the file, read all the data into the
     allocated arrays */
  nV = nN = nTex = nT = 0;
  ////_material = 0;

//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false; !ex;) {
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        rai::skipRestOfLine(is);
        strn(is);
        break;  //comment
      case 'v':               // v, vn, vt
        switch(str.p[1]) {
          case '\0': is >>V(nV, 0) >>V(nV, 1) >> V(nV, 2);  nV++;  break;  //vertex
          case 'n':  is >>Vn(nN, 0) >>Vn(nN, 1) >>Vn(nN, 2);  nN++;  break;  //normal
          case 't':  is >>tex(nTex, 0) >>tex(nTex, 1);   nTex++;  break;  //texcoord
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        if(strstr(str.p, "//")) {
          // v//vn
          sscanf(str.p, "%d//%d", &v, &n);

          T(nT, 0) = v < 0 ? v + nV : v;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->nT++] = nT;
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/vt/vn
          T(nT, 0) = v < 0 ? v + nV : v;
          Tt(nT, 0) = t < 0 ? t + nTex : t;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tt(nT, 1) = t < 0 ? t + nTex : t;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tt(nT, 2) = t < 0 ? t + nTex : t;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);  Tt(nT, 0) = Tt(nT-1, 0);  Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);  Tt(nT, 1) = Tt(nT-1, 2);  Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tt(nT, 2) = t < 0 ? t + nTex : t;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/vt
          T(nT, 0) = v < 0 ? v + nV : v;
          Tt(nT, 0) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tt(nT, 1) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tt(nT, 2) = t < 0 ? t + nTex : t;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) {
            T(nT, 0) = T(nT-1, 0);  Tt(nT, 0) = Tt(nT-1, 0);  T(nT, 1) = T(nT-1, 2);
            Tt(nT, 1) = Tt(nT-1, 2);  T(nT, 2) = v < 0 ? v + nV : v;  Tt(nT, 2) = t < 0 ? t + nTex : t;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else {
          // v
          sscanf(str.p, "%d", &v);
          T(nT, 0) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          //// group->triangles[group->numtriangles++] = nT;
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) {
            T(nT, 0) = T(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        }
        break;

      default:  rai::skipRestOfLine(is);  strn(is);  break;
    }
  }

  //CONVENTION!: start counting vertex indices from 0!!
  T -= 1u;
  CHECK(T.max() < nV, "");
  if(nTex) {
    Tt -= 1u;
    CHECK(Tt.max() < nTex, "");
  }
}
