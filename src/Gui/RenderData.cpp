#include "RenderData.h"

#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

uint bufW=2000, bufH=2000;
GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);

RenderObject& RenderScene::add(){
  std::shared_ptr<RenderObject> obj = make_shared<RenderObject>();
  objs.append(obj);
  return *obj;
}

void RenderScene::addLight(const arr& pos, const arr& focus, double heightAbs){
  std::shared_ptr<rai::Camera> light = make_shared<rai::Camera>();
  light->setHeightAbs(heightAbs);
//  light->setHeightAngle(45.);
  light->setZRange(1., 10.);
  light->X.pos.set(pos); //setPosition(4., -2., 4.);
  light->focus(focus(0), focus(1), focus(2)); //0.,0.,1.);
  light->upright(); //{0.,1.,0.});
  lights.append(light);
}

void RenderScene::addAxes(double scale, const rai::Transformation& _X){
  rai::Mesh M;
  rai::Transformation X;
  rai::Mesh tip;
  tip.setCone(.08, .16);
  tip.scale(scale);

  X.setZero().addRelativeTranslation(0., 0., .84*scale);
  tip.C = replicate({0., 0., 1.}, tip.V.d0);
  M.addMesh(tip, X);

  X.setZero().addRelativeTranslation(.84*scale, 0., 0.).addRelativeRotationDeg(90, 0., 1., 0);
  tip.C = replicate({1., 0., 0.}, tip.V.d0);
  M.addMesh(tip, X);

  X.setZero().addRelativeTranslation(0., .84*scale, 0.).addRelativeRotationDeg(90, -1., 0., 0);
  tip.C = replicate({0., 1., 0.}, tip.V.d0);
  M.addMesh(tip, X);

  add().mesh(M, _X, .9, _marker);

  add().lines({0.,0.,0., scale,0.,0.,
               0.,scale,0., 0.,0.,0.,
               0.,0.,0., 0.,0.,scale},
              {1.,0.,0., 1.,0.,0.,
               0.,1.,0., 0.,1.,0.,
               0.,0.,1., 0.,0.,1.},
              _X
              );
}

void RenderScene::glInitialize(OpenGL &gl){
  ContextIDs& id = contextIDs[&gl];
  CHECK(!id.initialized, "");

  glewExperimental = true; // Needed for core profile
  if(glewInit() != GLEW_OK) HALT("Failed to initialize GLEW\n");
  glClearColor(0.0f, 0.0f, 0.4f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_CULL_FACE);

  id.prog_ID = LoadShaders( rai::raiPath("src/Gui/shader.vs"), rai::raiPath("src/Gui/shader.fs") );
  id.prog_Projection_W = glGetUniformLocation(id.prog_ID, "Projection_W");
  id.prog_ViewT_CW = glGetUniformLocation(id.prog_ID, "ViewT_CW");
  id.prog_ModelT_WM = glGetUniformLocation(id.prog_ID, "ModelT_WM");
  id.prog_ShadowProjection_W = glGetUniformLocation(id.prog_ID, "ShadowProjection_W");
  id.prog_shadowMap = glGetUniformLocation(id.prog_ID, "shadowMap");
  id.prog_numLights = glGetUniformLocation(id.prog_ID, "numLights");
  id.prog_lightDirection_C = glGetUniformLocation(id.prog_ID, "lightDirection_C");

  id.progMarker = LoadShaders( rai::raiPath("src/Gui/shaderMarker.vs"), rai::raiPath("src/Gui/shaderMarker.fs") );
  id.progMarker_Projection_W = glGetUniformLocation(id.progMarker, "Projection_W");
  id.progMarker_ModelT_WM = glGetUniformLocation(id.progMarker, "ModelT_WM");

  { //if(stopRender>_shadows){
    id.progShadow_ID = LoadShaders( rai::raiPath("src/Gui/shaderShadow.vs"), rai::raiPath("src/Gui/shaderShadow.fs") );
    id.progShadow_ShadowProjection_W = glGetUniformLocation(id.progShadow_ID, "ShadowProjection_W");
    id.progShadow_ModelT_WM = glGetUniformLocation(id.progShadow_ID, "ModelT_WM");

    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    id.shadowFramebuffer = 0;
    glGenFramebuffers(1, &id.shadowFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, id.shadowFramebuffer);

    // Depth texture. Slower than a depth buffer, but you can sample it later in your shader
    glGenTextures(1, &id.shadowTexture);
    glBindTexture(GL_TEXTURE_2D, id.shadowTexture);
    glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT16, bufW, bufH, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, id.shadowTexture, 0);

    // No color output in the bound framebuffer, only depth.
    glDrawBuffer(GL_NONE);

    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) HALT("failed");
  }
  id.initialized=true;

//  for(auto& obj:objs){
//    obj->glInitialize();
//  }
}

void RenderObject::glRender(){
  CHECK(initialized, "");

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);

  glBindVertexArray(vbo);
  glDrawArrays(mode, 0, vertices.d0);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void RenderScene::renderObjects(GLuint idT_WM, const uintA& sorting, RenderType type){
  arr T_WM = eye(4);

  CHECK_EQ(sorting.N, objs.N, "");

  for(uint i=0;i<objs.N;i++){
    uint j;
    if(type!=_transparent) j = sorting.elem(i); //solid: near to far
    else j = sorting.elem(objs.N-1-i);    //transparent: far to near

    std::shared_ptr<RenderObject>& obj = objs.elem(j);
    if(obj->type!=type) continue;

    if(idT_WM){
      T_WM = obj->X.getAffineMatrix();
      glUniformMatrix4fv(idT_WM, 1, GL_TRUE, rai::convert<float>(T_WM).p);
    }

    obj->glRender();
  }

  if(type==_marker){
    distMarkers.pos.reshape(-1,2,3);
    CHECK_EQ(distMarkers.pos.d0, distMarkers.slices.N, "");
    for(uint i=0;i<distMarkers.pos.d0;i++){
      int s = distMarkers.slices(i);
      if(slice>=0 && s>=0 && s!=slice) continue;
      arr a=distMarkers.pos(i,0,{});
      arr b=distMarkers.pos(i,1,{});
      arr d = b-a;
      double l = length(d);
      if(l<1e-10) continue;
      d /= length(d);
      rai::Transformation X=0;
      X.pos.set(b);
      X.rot.setDiff(Vector_z, d);
      if(idT_WM){
        T_WM = X.getAffineMatrix();
        glUniformMatrix4fv(idT_WM, 1, GL_TRUE, rai::convert<float>(T_WM).p);
      }
      objs(distMarkers.markerObj)->glRender();
      X.pos.set(a);
      X.rot.addX(RAI_PI);
      T_WM = X.getAffineMatrix();
      glUniformMatrix4fv(idT_WM, 1, GL_TRUE, rai::convert<float>(T_WM).p);
      objs(distMarkers.markerObj)->glRender();
    }
  }
}

void RenderScene::glDraw(OpenGL& gl){
  ContextIDs& id = contextIDs[&gl];
  CHECK(id.initialized, "");

  for(std::shared_ptr<RenderObject>& obj:objs){
    if(!obj->initialized) obj->glInitialize();
  }

  camera = gl.camera;

  //sort objects
  for(std::shared_ptr<RenderObject>& obj:objs){
    obj->cameraDist = (obj->X.pos - camera.X.pos).length();
  }
  uintA sorting;
  sorting.setStraightPerm(objs.N);
  std::sort(sorting.p, sorting.p+sorting.N, [&](uint i,uint j){ return objs.elem(i)->cameraDist < objs.elem(j)->cameraDist; });

  if(dontRender>_shadow) for(uint k=0;k<(renderCount?1:2);k++){ //why do I have to render twice on first pass??
    // Render to shadowFramebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, id.shadowFramebuffer);
    glViewport(0, 0, bufW, bufH);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK); //FRONT would also work here, for shadows!
    glDisable(GL_BLEND);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(id.progShadow_ID);

    //set shadow projection matrix
    arr flip = eye(4);
//    flip(1,1) = flip(2,2) = -1.;
    arr P_IC = lights(0)->getT_IC();
    arr T_CW = lights(0)->getT_CW();
    arr Pshadow_IW = flip * P_IC * flip * T_CW;
    glUniformMatrix4fv(id.progShadow_ShadowProjection_W, 1, GL_TRUE, rai::convert<float>(Pshadow_IW).p);

    renderObjects(id.progShadow_ModelT_WM, sorting, _solid);

    glUseProgram(id.prog_ID);
    arr tmp = arr{{4,4},{ 0.5, 0.0, 0.0, 0.5,
                          0.0, 0.5, 0.0, 0.5,
                          0.0, 0.0, 0.5, 0.5,
                          0.0, 0.0, 0.0, 1.0}};
    tmp = tmp * Pshadow_IW;
    glUniformMatrix4fv(id.prog_ShadowProjection_W, 1, GL_TRUE, rai::convert<float>(tmp).p);
  }
  //LOG(1) <<"rendering! " <<renderCount;

  // Render to the screen
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, gl.width, gl.height);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  if(dontRender>_transparent){
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

//  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  arr ViewT_CW = gl.camera.getT_CW();
  arr Projection_W = gl.camera.getT_IC() * ViewT_CW;
  {
    glUseProgram(id.prog_ID);

    //set camera parameters
    glUniformMatrix4fv(id.prog_ViewT_CW, 1, GL_TRUE, rai::convert<float>(ViewT_CW).p);
    glUniformMatrix4fv(id.prog_Projection_W, 1, GL_TRUE, rai::convert<float>(Projection_W).p);

    //set light parameters
    glUniform1i(id.prog_numLights, lights.N);

    arr lightDirs;
    for(uint i=0;i<lights.N;i++){
      arr l_C = ViewT_CW * lights(i)->X.getAffineMatrix();
      lightDirs.append({l_C(0,2), l_C(1,2), l_C(2,2)});
    }
    lightDirs.reshape(lights.N, 3);
    glUniform3fv(id.prog_lightDirection_C, lights.N, rai::convert<float>(-lightDirs).p);

    if(dontRender>_shadow) {
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, id.shadowTexture);
      glUniform1i(id.prog_shadowMap, 1);
    }

    renderObjects(id.prog_ModelT_WM, sorting, _solid);
//    renderObjects(id.prog_ModelT_WM, sorting, _marker);
  }

  if(dontRender>_transparent) for(uint k=0;k<(renderCount?1:2);k++){
    glUseProgram(id.progMarker);
    glUniformMatrix4fv(id.progMarker_Projection_W, 1, GL_TRUE, rai::convert<float>(Projection_W).p);
    renderObjects(id.progMarker_ModelT_WM, sorting, _marker);
  }

  if(dontRender>_marker) for(uint k=0;k<(renderCount?1:2);k++){
    glUseProgram(id.prog_ID);
    renderObjects(id.prog_ModelT_WM, sorting, _transparent);
  }

  renderCount++;
}

void RenderScene::glDeinitialize(OpenGL& gl){
  ContextIDs& id = contextIDs[&gl];
  if(id.initialized){
    glDeleteProgram(id.prog_ID);
    glDeleteProgram(id.progShadow_ID);
    glDeleteFramebuffers(1, &id.shadowFramebuffer);
    glDeleteTextures(1, &id.shadowTexture);
  }

  objs.clear();
}

RenderObject::~RenderObject(){
  if(initialized){
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    glDeleteBuffers(1, &normalbuffer);
    glDeleteVertexArrays(1, &vbo);
  }
  initialized=false;
}

void RenderObject::mesh(rai::Mesh& mesh, const rai::Transformation& _X, double avgNormalsThreshold, RenderType _type){
  X = _X;
  type = _type;
  if(type==_solid && (mesh.C.N==4 || mesh.C.N==2) && mesh.C(-1)<1.) type = _transparent;
  version=mesh.version;

  mesh.computeNormals();
  vertices.resize(mesh.T.d0*3, 3);
  colors.resize(vertices.d0, 4);
  normals.resizeAs(vertices);
  arr c;
  if(!mesh.C.N) c = arr{.8,.8,.8};
  if(mesh.C.nd==1) c = mesh.C;
  if(c.N==1){ double g=c.elem(); c = arr{g,g,g}; }
  for(uint i=0;i<mesh.T.d0;i++){
    for(uint j=0;j<3;j++){
      if(mesh.C.nd==2) c.referToDim(mesh.C, mesh.T(i,j));
      for(uint k=0;k<3;k++) vertices(3*i+j,k) = mesh.V(mesh.T(i,j), k);
      for(uint k=0;k<3;k++) colors(3*i+j,k) = c(k); //m.C(m.T(i,j), k);
      if((mesh.C.nd==1 && mesh.C.N==4) || (mesh.C.nd==2 && mesh.C.d1==4)) colors(3*i+j, 3)=c(3); else colors(3*i+j, 3)=1.;
      for(uint k=0;k<3;k++) normals(3*i+j, k) = mesh.Tn(i,k);
    }
  }

  //-- normal smoothing
  if(avgNormalsThreshold<1.){
    //go through all vertices and check whether to average normals for smoothing
    uintAA sameVertices(mesh.V.d0);
    for(uint i=0;i<mesh.T.d0;i++){
      for(uint j=0;j<3;j++) sameVertices(mesh.T(i,j)).append(3*i+j);
    }

    for(uint i=0;i<sameVertices.N;i++){
      uintA& verts = sameVertices(i);
      uint n = verts.N;
      if(n<2) continue;
      //copy to arr
      arr ns(n, 3);
      for(uint j=0;j<n;j++) ns[j] = rai::convert<double>(normals[verts(j)]);
      //check if we can average all of them (all pairwise scalar products > threshold)
      bool avg = true;
      for(uint j=0;j<n;j++) for(uint k=j+1;k<n;k++){
        if(scalarProduct(ns[j], ns[k])<avgNormalsThreshold){ avg=false; break; }
      }
      if(avg){
        arr na = sum(ns, 0);
        na /= length(na);
        floatA naf = rai::convert<float>(na);
        for(uint j=0;j<n;j++) normals[verts(j)] = naf;
      }else{
        //average them pair-wise
        for(uint j=0;j<n;j++) for(uint k=0;k<n;k++){
          arr nj=ns[j], nk=ns[k];
          if(scalarProduct(nj, nk)>avgNormalsThreshold){
            arr na = nj+nk;
            na /= length(na);
            nj = na;
            nk = na;
          }
        }
        for(uint j=0;j<n;j++) normals[verts(j)] = rai::convert<float>(ns[j]);
      }
    }
  }
}

void RenderObject::lines(const arr& lines, const arr& color, const rai::Transformation& _X, RenderType _type){
  X = _X;
  vertices = rai::convert<float>(lines).reshape(-1, 3);
  if(color.N>3){
    colors = rai::convert<float>(color).reshape(-1, 3);
  }else{
    colors = rai::convert<float>(replicate(color, vertices.d0));
  }
  if(colors.d1==3){ colors.insColumns(3); for(uint i=0;i<colors.d0;i++) colors(i,3)=1.; }
  normals.clear();
  type = _type;
  mode = GL_LINES;
}

void RenderObject::glInitialize(){
  glGenVertexArrays(1, &vbo);
  glBindVertexArray(vbo);

  glGenBuffers(1, &vertexbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
  glBufferData(GL_ARRAY_BUFFER, vertices.N*vertices.sizeT, vertices.p, GL_STATIC_DRAW);

  glGenBuffers(1, &colorbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glBufferData(GL_ARRAY_BUFFER, colors.N*colors.sizeT, colors.p, GL_STATIC_DRAW);

  glGenBuffers(1, &normalbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
  glBufferData(GL_ARRAY_BUFFER, normals.N*normals.sizeT, normals.p, GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);

  glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

  glBindVertexArray(0);

  initialized=true;
}

//===========================================================================

GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path){

  // Create the shaders
  GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

#if 1
  // Read the Vertex Shader code from the file
  std::string VertexShaderCode;
  std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
  if(VertexShaderStream.is_open()){
    std::stringstream sstr;
    sstr << VertexShaderStream.rdbuf();
    VertexShaderCode = sstr.str();
    VertexShaderStream.close();
  }else{
    printf("Impossible to open %s. Are you in the right directory ? Don't forget to read the FAQ !\n", vertex_file_path);
    getchar();
    return 0;
  }

  // Read the Fragment Shader code from the file
  std::string FragmentShaderCode;
  std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
  if(FragmentShaderStream.is_open()){
    std::stringstream sstr;
    sstr << FragmentShaderStream.rdbuf();
    FragmentShaderCode = sstr.str();
    FragmentShaderStream.close();
  }
  char const * VertexSourcePointer = VertexShaderCode.c_str();
  char const * FragmentSourcePointer = FragmentShaderCode.c_str();
#else
  str vertexShaderCode(FILE(vertex_file_path).getIs());
  str fragmentShaderCode(FILE(fragment_file_path).getIs());
  char const * VertexSourcePointer = vertexShaderCode.p; //VertexShaderCode.c_str();
  char const * FragmentSourcePointer = fragmentShaderCode.p; //FragmentShaderCode.c_str();
  //cout <<vertexShaderCode <<fragmentShaderCode <<endl;
#endif
  GLint Result = GL_FALSE;
  int InfoLogLength;

  // Compile Vertex Shader
//  printf("Compiling shader : %s\n", vertex_file_path);
  glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
  glCompileShader(VertexShaderID);

  // Check Vertex Shader
  glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
    glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
    printf("%s\n", &VertexShaderErrorMessage[0]);
  }

  // Compile Fragment Shader
//  printf("Compiling shader : %s\n", fragment_file_path);
  glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
  glCompileShader(FragmentShaderID);

  // Check Fragment Shader
  glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
  glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
    glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
    printf("%s\n", &FragmentShaderErrorMessage[0]);
  }

  // Link the program
//  printf("Linking program\n");
  GLuint ProgramID = glCreateProgram();
  glAttachShader(ProgramID, VertexShaderID);
  glAttachShader(ProgramID, FragmentShaderID);
  glLinkProgram(ProgramID);

  // Check the program
  glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
  glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
  if ( InfoLogLength > 0 ){
    std::vector<char> ProgramErrorMessage(InfoLogLength+1);
    glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
    printf("%s\n", &ProgramErrorMessage[0]);
  }

  glDetachShader(ProgramID, VertexShaderID);
  glDetachShader(ProgramID, FragmentShaderID);

  glDeleteShader(VertexShaderID);
  glDeleteShader(FragmentShaderID);

  return ProgramID;
}

void RenderScene::addDistMarker(const arr& a, const arr& b, int s){
  if(distMarkers.markerObj==-1){
    distMarkers.markerObj=objs.N;
    rai::Mesh m;
    m.setCone(.01, .01);
    m.translate(0.,0.,-.01);
    m.C={1.,0.,1.};
    add().mesh(m, 0, .9, _marker);
  }
  distMarkers.pos.append(a);
  distMarkers.pos.append(b);
  distMarkers.slices.append(s);
}

void RenderScene::clearObjs(){
  objs.clear();
  distMarkers.markerObj=-1;
  distMarkers.pos.clear();
  distMarkers.slices.clear();
  renderCount=0;
}
