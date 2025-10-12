#include "shaders.cxx"
#include "RenderData.h"

#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

namespace rai {

struct OpenGL2Context {
  std::map<OpenGL*, RenderData::ContextIDs> contextIDs;
};

Singleton<OpenGL2Context> contextIDs;

uint bufW=2048, bufH=2048;
double shadowHeight = 5.;

GLuint LoadShadersFile(const char * vertex_file_path,const char * fragment_file_path);
GLuint LoadShaders(const std::string& VertexShaderCode, const std::string& FragmentShaderCode);

RenderData::RenderData(){
  rai::Mesh M;
  M.setCylinder(.0025, 1., 1);
  M.translate(0.,0.,.5);
  M.C={1.,0.,1.};
  cylin.asset = make_shared<RenderAsset>();
  cylin.asset->mesh(M);
}

RenderAsset& RenderData::add(const Transformation& _X, RenderType _type){
  std::shared_ptr<RenderItem> obj = make_shared<RenderItem>(_X, _type);
  items.append(obj);
  obj->asset = make_shared<RenderAsset>();
  return *(obj->asset);
}

RenderAsset& RenderData::addShared(std::shared_ptr<RenderItem> _item, const rai::Transformation& _X, RenderType _type){
  std::shared_ptr<RenderItem> item = make_shared<RenderItem>(_X, _type);
  items.append(item);
  item->asset = _item->asset;
  return *(item->asset);
}

void RenderData::addLight(const arr& pos, const arr& focus, double heightAbs){
  std::shared_ptr<rai::Camera> light = make_shared<rai::Camera>();
  light->setHeightAbs(heightAbs);
//  light->setHeightAngle(45.);
  light->setZRange(1., 10.);
  light->X.pos.set(pos); //setPosition(4., -2., 4.);
  light->focus(focus); //0.,0.,1.);
  light->upright(); //{0.,1.,0.});
  lights.append(light);
}

void RenderData::addAxes(double scale, const rai::Transformation& _X){
  rai::Mesh M;
  rai::Transformation X;
  rai::Mesh tip, ax;
  tip.setCone(.08, .16);
  tip.scale(scale);
  ax.setCylinder(.02, .9, 1);
  ax.scale(scale);

  X.setZero().appendRelativeTranslation(0., 0., .84*scale);
  tip.C = replicate({0., 0., 1.}, tip.V.d0);
  M.addMesh(tip, X);
  X.setZero().appendRelativeTranslation(0., 0., .45*scale);
  ax.C = replicate({0., 0., 1.}, ax.V.d0);
  M.addMesh(ax, X);

  X.setZero().appendRelativeTranslation(.84*scale, 0., 0.).appendRelativeRotationDeg(90, 0., 1., 0);
  tip.C = replicate({1., 0., 0.}, tip.V.d0);
  M.addMesh(tip, X);
  X.setZero().appendRelativeTranslation(.45*scale, 0., 0.).appendRelativeRotationDeg(90, 0., 1., 0);
  ax.C = replicate({1., 0., 0.}, ax.V.d0);
  M.addMesh(ax, X);

  X.setZero().appendRelativeTranslation(0., .84*scale, 0.).appendRelativeRotationDeg(90, -1., 0., 0);
  tip.C = replicate({0., 1., 0.}, tip.V.d0);
  M.addMesh(tip, X);
  X.setZero().appendRelativeTranslation(0., .45*scale, 0.).appendRelativeRotationDeg(90, -1., 0., 0);
  ax.C = replicate({0., 1., 0.}, ax.V.d0);
  M.addMesh(ax, X);

  add(_X, _marker).mesh(M, .9);

//  add().lines({0.,0.,0., scale,0.,0.,
//               0.,scale,0., 0.,0.,0.,
//               0.,0.,0., 0.,0.,scale},
//              {1.,0.,0., 1.,0.,0.,
//               0.,1.,0., 0.,1.,0.,
//               0.,0.,1., 0.,0.,1.},
//              _X
//              );
}

void RenderFont::glInitialize(){
  FT_Library ft;
  int r = FT_Init_FreeType(&ft);
  if(r){
    LOG(-1) <<"FreeType Error: Could not initialize FreeType Library. error code: " <<r <<" -> text rendering disabled";
    return;
  }

  StringA fonts = { "tlwg/Sawasdee.ttf", "freefont/FreeSerif.ttf", "ubuntu/Ubuntu-L.ttf", "dejavu/DejaVuSans.ttf", "teluguvijayam/mallanna.ttf", "none"};
  uint fontID = 2;
  FT_Face face;
  r = FT_New_Face(ft, "/usr/share/fonts/truetype/" + fonts(fontID), 0, &face);
  if(r){
    LOG(-1) <<"FreeType Error: Failed to load font '" <<fonts(fontID) <<"' error code: " <<r <<" -> text rendering disabled";
    FT_Done_FreeType(ft);
    return;
  }

  FT_Set_Pixel_Sizes(face, 0, 16);

  // disable byte-alignment restriction
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // load first 128 characters of ASCII set
  characters.resize(128);
  for (unsigned char c = 0; c < 128; c++)
  {
    // Load character glyph
    if (FT_Load_Char(face, c, FT_LOAD_RENDER))
    {
      std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
      continue;
    }
    // generate texture
    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_RED,
          face->glyph->bitmap.width,
          face->glyph->bitmap.rows,
          0,
          GL_RED,
          GL_UNSIGNED_BYTE,
          face->glyph->bitmap.buffer
          );
    // set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // now store character for later use
    characters(c) = { texture,
                      (int)face->glyph->bitmap.width, (int)face->glyph->bitmap.rows,
                      face->glyph->bitmap_left, face->glyph->bitmap_top,
                      (int)face->glyph->advance.x };
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // destroy FreeType once we're finished
  FT_Done_Face(face);
  FT_Done_FreeType(ft);
}

void RenderData::ensureInitialized(OpenGL &gl){
  auto lock = dataLock(RAI_HERE);

  ContextIDs& id = contextIDs()->contextIDs[&gl];
  if(id.initialized) return;

  glewExperimental = true; // Needed for core profile
  if(glewInit() != GLEW_OK) HALT("Failed to initialize GLEW\n");

  if(opt.userShaderFiles){
    id.prog_ID = LoadShadersFile("shaderObj.vs", "shaderObj.fs");
  }else{
    id.prog_ID = LoadShaders( objVS, objFS );
    // id.prog_ID = LoadShadersFile(rai::raiPath("src/Gui/shaderObj.vs"), rai::raiPath("src/Gui/shaderObj.fs") );
  }
  id.prog_Projection_W = glGetUniformLocation(id.prog_ID, "Projection_W");
  id.prog_ModelT_WM = glGetUniformLocation(id.prog_ID, "ModelT_WM");
  id.prog_eyePosition_W = glGetUniformLocation(id.prog_ID, "eyePosition_W");
  id.prog_ShadowProjection_W = glGetUniformLocation(id.prog_ID, "ShadowProjection_W");
  id.prog_useShadow = glGetUniformLocation(id.prog_ID, "useShadow");
  id.prog_shadowMap = glGetUniformLocation(id.prog_ID, "shadowMap");
  id.prog_numLights = glGetUniformLocation(id.prog_ID, "numLights");
  id.prog_lightDirection_W = glGetUniformLocation(id.prog_ID, "lightDirection_W");
  id.prog_FlatColor = glGetUniformLocation(id.prog_ID, "flatColor");
  id.prog_textureDim = glGetUniformLocation(id.prog_ID, "textureDim");
  id.prog_textureImage = glGetUniformLocation(id.prog_ID, "textureImage");

  if(opt.userShaderFiles){
    id.progTensor = LoadShadersFile("shaderTensor.vs", "shaderTensor.fs");
  }else{
    id.progTensor = LoadShaders( tensorVS, tensorFS );
    // id.progTensor = LoadShadersFile(rai::raiPath("../rai/src/Gui/shaderTensor.vs"), rai::raiPath("../rai/src/Gui/shaderTensor.fs") );
  }
  id.progTensor_Projection_W = glGetUniformLocation(id.progTensor, "Projection_W");
  id.progTensor_ModelT_WM = glGetUniformLocation(id.progTensor, "ModelT_WM");
  id.progTensor_ModelScale = glGetUniformLocation(id.progTensor, "ModelScale");
  id.progTensor_eyePosition_W = glGetUniformLocation(id.progTensor, "eyePosition_W");
  id.progTensor_FlatColor = glGetUniformLocation(id.progTensor, "flatColor");
  id.progTensor_tensorTexture = glGetUniformLocation(id.progTensor, "tensorTexture");

  {
    id.progMarker = LoadShaders( markerVS, markerFS );
    // id.progMarker = LoadShadersFile(rai::raiPath("src/Gui/shaderMarker.vs"), rai::raiPath("src/Gui/shaderMarker.fs") );
  }
  id.progMarker_Projection_W = glGetUniformLocation(id.progMarker, "Projection_W");
  id.progMarker_ModelT_WM = glGetUniformLocation(id.progMarker, "ModelT_WM");
  id.progMarker_FlatColor = glGetUniformLocation(id.progMarker, "flatColor");

  { //if(stopRender>_shadows){
    id.progShadow = LoadShaders( shadowVS, shadowFS );
    //id.progShadow = LoadShadersFile( rai::raiPath("src/Gui/shaderShadow.vs"), rai::raiPath("src/Gui/shaderShadow.fs") );
    id.progShadow_ShadowProjection_W = glGetUniformLocation(id.progShadow, "ShadowProjection_W");
    id.progShadow_ModelT_WM = glGetUniformLocation(id.progShadow, "ModelT_WM");

    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    id.shadowFramebuffer = 0;
    glGenFramebuffers(1, &id.shadowFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, id.shadowFramebuffer);

    // Depth texture. Slower than a depth buffer, but you can sample it later in your shader
    glGenTextures(1, &id.shadowTexture);
    glBindTexture(GL_TEXTURE_2D, id.shadowTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, bufW, bufH, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    static GLfloat borderColor [4] = {1., 1., 1., 1.};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, id.shadowTexture, 0);

    // No color output in the bound framebuffer, only depth.
    glDrawBuffer(GL_NONE);

    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      HALT("shadow framebuffer generation failed");

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  {
    id.progText = LoadShaders( textVS, textFS ); //rai::raiPath("src/Gui/shaderText.vs"), rai::raiPath("src/Gui/shaderText.fs") );
    id.progText_color = glGetUniformLocation( id.progText, "textColor" );
    id.progText_useTexColor = glGetUniformLocation( id.progText, "useTexColor" );
  }

  id.font.glInitialize();

  id.initialized=true;
}

void RenderData::renderObjects(GLuint prog_ModelT_WM, const uintA& sortedObjIDs, RenderType type, GLint idFlatColor, GLint idScale, GLint idTextureDim){
  arr ModelT_WM = eye(4);

  CHECK_EQ(sortedObjIDs.N, items.N, "");

  for(uint i=0;i<items.N;i++){

    uint objID;
    if(type!=_transparent) objID = sortedObjIDs.elem(i); //solid: near to far
    else objID = sortedObjIDs.elem(items.N-1-i);    //transparent: far to near

    if((int)objID==distMarkers.markerObj) continue;

    std::shared_ptr<RenderItem>& obj = items.elem(objID);
    if(obj->type!=type) continue;

    ModelT_WM = obj->X.getMatrix();
    glUniformMatrix4fv(prog_ModelT_WM, 1, GL_TRUE, rai::convert<float>(ModelT_WM).p);

    if(idScale>=0 && obj->scale.N==3){
      glUniform3f(idScale, obj->scale(0), obj->scale(1), obj->scale(2));
    }

    if((renderFlatColors || type==_tensor) && idFlatColor>=0 && obj->flatColor.N){
      CHECK_EQ(obj->flatColor.N, 3, "");
      byte *rgb = obj->flatColor.p;
      glUniform4f(idFlatColor, rgb[0]/255.f, rgb[1]/255.f, rgb[2]/255.f, 1.);
    }else{
      glUniform4f(idFlatColor, 0.f, 0.f, 0.f, 0.f);
      if(idTextureDim>=0){
        glUniform1i(idTextureDim, obj->asset->textureDim);
      }
    }

    obj->asset->glRender();

    if(opt.polygonLines){
      glUniform4f(idFlatColor, 0.f, 0.f, 0.f, 1.f);
      glPolygonMode(GL_FRONT, GL_LINE);
      obj->asset->glRender();
      glPolygonMode(GL_FRONT, GL_FILL);
    }
  }

  if(type==_marker){
    distMarkers.pos.reshape(-1,2,3);
    distMarkers.color.reshape(-1,3);
    CHECK_EQ(distMarkers.pos.d0, distMarkers.slices.N, "");
    CHECK_EQ(distMarkers.color.d0, distMarkers.slices.N, "");
    for(uint i=0;i<distMarkers.pos.d0;i++){
      int s = distMarkers.slices(i);
      if(slice>=0 && s>=0 && s!=slice) continue;
      arr a=distMarkers.pos(i,0,{});
      arr b=distMarkers.pos(i,1,{});
      arr d = b-a;
      double l = length(d);
      if(l<1e-10) d = {0., 0., 1.};
      else d /= length(d);
      rai::Transformation X=0;
//      X.pos.set(b);
//      X.rot.setDiff(Vector_z, d);
//      if(idT_WM){
//        T_WM = X.getMatrix();
//        glUniformMatrix4fv(idT_WM, 1, GL_TRUE, rai::convert<float>(T_WM).p);
//      }
//      items(distMarkers.markerObj)->glRender();
//      X.pos.set(a);
//      X.rot.addX(RAI_PI);
//      T_WM = X.getMatrix();
//      glUniformMatrix4fv(idT_WM, 1, GL_TRUE, rai::convert<float>(T_WM).p);
//      items(distMarkers.markerObj)->glRender();

      X.pos.set(a);
      X.rot.setDiff(Vector_z, d);
      ModelT_WM = X.getMatrix();
      for(uint k=0;k<4;k++) ModelT_WM(k,2) *= l; //scale length
      glUniformMatrix4fv(prog_ModelT_WM, 1, GL_TRUE, rai::convert<float>(ModelT_WM).p);
      glUniform4f(idFlatColor, distMarkers.color(i,0), distMarkers.color(i,1), distMarkers.color(i,2), 1.);
      cylin.asset->glRender();
    }
  }
}

void RenderData::glDraw(OpenGL& gl){
  auto lock = dataLock(RAI_HERE);

  ContextIDs& id = contextIDs()->contextIDs[&gl];
  CHECK(id.initialized, "");

  for(std::shared_ptr<RenderItem>& obj:items) if(!obj->asset->initialized) obj->asset->glInitialize();
  for(std::shared_ptr<RenderText>& txt:texts) if(!txt->initialized) txt->glInitialize();
  for(std::shared_ptr<RenderQuad>& quad:quads) if(!quad->initialized) quad->glInitialize();
  if(!cylin.asset->initialized) cylin.asset->glInitialize();

  if(!gl.activeView){
    camera = gl.camera;
  }else{
    camera = gl.activeView->camera;
  }

  //sort objects
  for(std::shared_ptr<RenderItem>& obj:items) obj->cameraDist = (obj->X.pos - camera.X.pos).length();
  uintA sorting;
  sorting.setStraightPerm(items.N);
  std::sort(sorting.p, sorting.p+sorting.N, [&](uint i,uint j){ return items.elem(i)->cameraDist < items.elem(j)->cameraDist; });

  //mark transparent
  for(std::shared_ptr<RenderItem>& obj:items) if(obj->type==_solid && obj->asset->isTransparent) obj->type=_transparent;

  //-- SHADOW BEGIN
  if(renderUntil>=_shadow && opt.useShadow){
    // Render to shadowFramebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, id.shadowFramebuffer);
    glViewport(0, 0, bufW, bufH);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK); //FRONT would also work here, for shadows!
    glDisable(GL_BLEND);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(id.progShadow);

    //set shadow projection matrix
    arr flip = eye(4), P_IC=eye(4), T_CW=eye(4);
    flip(1,1) = flip(2,2) = -1.;
    if(lights.N){
      P_IC = lights(0)->getT_IC();
      T_CW = lights(0)->getT_CW();
    }
    arr Pshadow_IW = flip * P_IC * flip * T_CW;
    glUniformMatrix4fv(id.progShadow_ShadowProjection_W, 1, GL_TRUE, rai::convert<float>(Pshadow_IW).p);

    renderObjects(id.progShadow_ModelT_WM, sorting, _solid);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glUseProgram(id.prog_ID);
    arr tmp = arr{{4,4},{ 0.5, 0.0, 0.0, 0.5,
                          0.0, 0.5, 0.0, 0.5,
                          0.0, 0.0, 0.5, 0.5,
                          0.0, 0.0, 0.0, 1.0}};
    tmp = tmp * Pshadow_IW;
    glUniformMatrix4fv(id.prog_ShadowProjection_W, 1, GL_TRUE, rai::convert<float>(tmp).p);
  }
  //-- SHADOW END

  // Render to the screen
  if(gl.offscreen){
    glBindFramebuffer(GL_FRAMEBUFFER, gl.offscreenFramebuffer);
  }else{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  if(!gl.activeView){
    glViewport(0, 0, gl.width, gl.height);
  }else{
    glViewport(gl.activeView->le*gl.width, gl.activeView->bo*gl.height, (gl.activeView->ri-gl.activeView->le)*gl.width+1, (gl.activeView->to-gl.activeView->bo)*gl.height+1);
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glCullFace(GL_BACK);
//  if(renderUntil>=_transparent){
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  }

  arr ViewT_CW = camera.getT_CW();
  arr Projection_W = camera.getT_IC() * ViewT_CW;
  {
    glUseProgram(id.prog_ID);

    //set camera parameters
    glUniformMatrix4fv(id.prog_Projection_W, 1, GL_TRUE, rai::convert<float>(Projection_W).p);
    glUniform3f(id.prog_eyePosition_W, camera.X.pos.x, camera.X.pos.y, camera.X.pos.z);

    //set light parameters
    glUniform1i(id.prog_numLights, lights.N);

    arr lightDirs;
    for(uint i=0;i<lights.N;i++){
      arr l_C = /*ViewT_CW * */lights(i)->X.getMatrix();
      lightDirs.append({l_C(0,2), l_C(1,2), l_C(2,2)});
    }
    lightDirs.reshape(lights.N, 3);
    glUniform3fv(id.prog_lightDirection_W, lights.N, rai::convert<float>(-lightDirs).p);

    if(renderUntil>=_shadow && opt.useShadow) {
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, id.shadowTexture);
      glUniform1i(id.prog_useShadow, 1);
      glUniform1i(id.prog_shadowMap, 1);
    }else{
      glUniform1i(id.prog_useShadow, 0);
    }

    glUniform1i(id.prog_textureImage, 0);

    renderObjects(id.prog_ModelT_WM, sorting, _solid, id.prog_FlatColor, -1, id.prog_textureDim);
//    renderObjects(id.prog_ModelT_WM, sorting, _marker);

    if(renderUntil>=_shadow && opt.useShadow) glBindTexture(GL_TEXTURE_2D, 0);
  }

  // glDisable(GL_DEPTH_TEST);
  if(renderUntil>=_tensor) {
    glUseProgram(id.progTensor);
    glUniformMatrix4fv(id.progTensor_Projection_W, 1, GL_TRUE, rai::convert<float>(Projection_W).p);
    glUniform3f(id.progTensor_eyePosition_W, camera.X.pos.x, camera.X.pos.y, camera.X.pos.z);
    glUniform1i(id.progTensor_tensorTexture, 0);
    renderObjects(id.progTensor_ModelT_WM, sorting, _tensor, id.progTensor_FlatColor, id.progTensor_ModelScale);
  }
  // glEnable(GL_DEPTH_TEST);

  if(renderUntil>=_marker) {
    glUseProgram(id.progMarker);
    glUniformMatrix4fv(id.progMarker_Projection_W, 1, GL_TRUE, rai::convert<float>(Projection_W).p);
    renderObjects(id.progMarker_ModelT_WM, sorting, _marker, id.progMarker_FlatColor);
  }

  if(renderUntil>=_transparent) {
    glUseProgram(id.prog_ID);
    renderObjects(id.prog_ModelT_WM, sorting, _transparent, id.prog_FlatColor, -1, id.prog_textureDim);
  }

  if(renderUntil>=_text) {

    glUseProgram(id.progText);
    glUniform1i(id.progText_useTexColor, 0);
    glm::mat4 projection = glm::ortho(0.0f, float(gl.width), 0.0f, float(gl.height));
    glUniformMatrix4fv(glGetUniformLocation(id.progText, "projection"), 1, GL_FALSE, &projection[0][0]);

    for(auto &txt:texts) txt->glRender(id.progText_color, id.font, gl.height);

    glUniform1i(id.progText_useTexColor, 1);
    for(auto &quad:quads) quad->glRender();
  }

  renderCount++;
}

void RenderData::glDeinitialize(OpenGL& gl){
  auto lock = dataLock(RAI_HERE);

  ContextIDs& id = contextIDs()->contextIDs[&gl];
  if(id.initialized){
    glDeleteProgram(id.prog_ID);
    glDeleteProgram(id.progShadow);
    glDeleteProgram(id.progTensor);
    glDeleteProgram(id.progMarker);
    glDeleteProgram(id.progText);
    glDeleteFramebuffers(1, &id.shadowFramebuffer);
    glDeleteTextures(1, &id.shadowTexture);
    id.initialized=false;
  }
  contextIDs()->contextIDs.erase(&gl);

  items.clear();
}

void RenderAsset::mesh(rai::Mesh& mesh, double avgNormalsThreshold){
  isTransparent = (mesh.C.N==4 || mesh.C.N==2 || mesh.C.d1==4) && mesh.C.elem(-1)<1.;
  version=mesh.version;

  if(!mesh.isArrayFormatted) mesh.makeArrayFormatted(avgNormalsThreshold);

  CHECK_EQ(mesh.V.d0, mesh.T.d0*3, "");
  CHECK_EQ(mesh.V.d0, mesh.Vn.d0, "");
  vertices = rai::convert<float>(mesh.V);
  normals = rai::convert<float>(mesh.Vn);
  if(!mesh._texImg || !mesh._texImg->img.N || !mesh.texCoords.N){
    if(mesh.C.N){
      if(mesh.C.d0!=mesh.V.d0){
        colors = rai::convert<float>(reshapeColor(mesh.C, mesh.V.d0));
      }else{
        CHECK_EQ(mesh.V.d0, mesh.C.d0, "");
        CHECK_EQ(mesh.C.d1, 4, "");
        colors = rai::convert<float>(mesh.C);
      }
    }else{
      colors.resize(mesh.V.d0, 4);
      colors=1.;
    }
  }else{
    // CHECK_EQ(mesh.C.N, 0,"no vertex colors allowed for textured meshes");
    CHECK_EQ(mesh.V.d0, mesh.texCoords.d0,"");
    CHECK_EQ(mesh.texCoords.d1, 2, "");
    CHECK_EQ(mesh.texImg().img.d2, 3, "");
    colors = rai::convert<float>(mesh.texCoords);
    colors.insColumns(-1, 2);
    for(uint i=0;i<colors.d0;i++) colors(i,2) = colors(i,3) = 1.;
    _texture = mesh._texImg; //().rgb;
    textureDim = 2;
  }
  mode = GL_TRIANGLES;
}

void RenderAsset::pointCloud(const arr& points, const arr& color){
  mode = GL_POINTS;
  isTransparent = (color.N==4 || color.N==2) && color(-1)<1.;

  vertices = rai::convert<float>(points);
  arr c = reshapeColor(color);

  if(!color.N){
    colors.resize(vertices.d0, 4).setZero();
    for(uint i=0;i<vertices.d0;i++) colors(i,-1) = 1.;
  }else if(color.nd==1){
    colors.resize(vertices.d0, 4);
    for(uint i=0;i<vertices.d0;i++) for(uint k=0;k<4;k++) colors(i,k) = c(k);
  }else{
    if(color.N){
      CHECK_EQ(color.d0, vertices.d0, "");
      colors = rai::convert<float>(c);
    }
  }
}

void RenderAsset::lines(const arr& lines, const arr& color){
  vertices = rai::convert<float>(lines).reshape(-1, 3);
  colors = rai::convert<float>(reshapeColor(color, vertices.d0));
  normals.clear();
  mode = GL_LINES;
}

void RenderAsset::tensor(const floatA& vol, const arr& size){
  CHECK_EQ(size.N, 3, "");
  texture().img.resize(vol.d0, vol.d1, vol.d2);
  for(uint i=0;i<vol.N;i++){
    float& f = vol.p[i];
    texture().img.p[i] = (f<0.f? byte(0) : (f>1.f ? byte(255) : byte(f*255.f) ));
  }
  rai::Mesh mesh;
  mesh.setBox();
  mesh.scale(size);
  mesh.makeArrayFormatted();
  vertices = rai::convert<float>(mesh.V);
  colors.clear();
  normals = rai::convert<float>(mesh.Vn);
  textureDim = 3;
}


void RenderAsset::glRender(){
  CHECK(initialized, "");

  if(textureDim) CHECK(_texture, "");
  if(textureDim) glActiveTexture(GL_TEXTURE0);
  if(textureDim==2) glBindTexture(GL_TEXTURE_2D, _texture->glBufferID);
  if(textureDim==3) glBindTexture(GL_TEXTURE_3D, _texture->glBufferID);

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);

  glBindVertexArray(vao);
  glDrawArrays(mode, 0, vertices.d0);
  glBindVertexArray(0);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);

  if(textureDim==2) glBindTexture(GL_TEXTURE_2D, 0);
  if(textureDim==3) glBindTexture(GL_TEXTURE_3D, 0);
}

void RenderAsset::glInitialize(){
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, vertices.N*vertices.sizeT, vertices.p, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glGenBuffers(1, &colorBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
  glBufferData(GL_ARRAY_BUFFER, colors.N*colors.sizeT, colors.p, GL_STATIC_DRAW);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glGenBuffers(1, &normalBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
  glBufferData(GL_ARRAY_BUFFER, normals.N*normals.sizeT, normals.p, GL_STATIC_DRAW);
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindVertexArray(0);

  //generate texture
  if(textureDim){
    CHECK(_texture,"");
    if(_texture->glBufferID==UINT32_MAX){
      byteA& img = _texture->img;
      CHECK_EQ(img.nd, 3, "");
      glGenTextures(1, &_texture->glBufferID);
      glActiveTexture(GL_TEXTURE0);
      if(textureDim==2){ //this is texture image and colors are texture coordinates
        glBindTexture(GL_TEXTURE_2D, _texture->glBufferID);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); //CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); //CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        if(img.d2==1) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
        else if(img.d2==2) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
        else if(img.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
        else if(img.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
        else NIY;
        glBindTexture(GL_TEXTURE_2D, 0);
      }
      else if(textureDim==3){
        glEnable(GL_TEXTURE_3D);
        glBindTexture(GL_TEXTURE_3D, _texture->glBufferID);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        // glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_R8, img.d2, img.d1, img.d0, 0, GL_RED, GL_UNSIGNED_BYTE, img.p);
        glBindTexture(GL_TEXTURE_3D, 0);
      }
      else NIY;
    }
  }else{
    CHECK(!_texture,"");
  }

  initialized=true;

//   GLint mem=0;
//   glGetIntegerv(GL_VBO_FREE_MEMORY_ATI, &mem);
//   if(mem) LOG(0) <<" -- warning, little vbo memory left: " <<mem;
}

RenderAsset::~RenderAsset(){
  if(initialized){
    glDeleteBuffers(1, &vertexBuffer);
    glDeleteBuffers(1, &colorBuffer);
    glDeleteBuffers(1, &normalBuffer);
    glDeleteVertexArrays(1, &vao);
  }
  initialized=false;
}

//===========================================================================

GLuint LoadShadersFile(const char * vertex_file_path,const char * fragment_file_path){

  // Read the Vertex Shader code from the file
  std::string VertexShaderCode;
  std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
  if(VertexShaderStream.is_open()){
    std::stringstream sstr;
    sstr << VertexShaderStream.rdbuf();
    VertexShaderCode = sstr.str();
    VertexShaderStream.close();
  }else{
    HALT("can't open vertex shader file " <<vertex_file_path);
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
  }else{
    HALT("can't open fragment shader file " <<fragment_file_path);
    return 0;
  }

  return LoadShaders(VertexShaderCode, FragmentShaderCode);
}

GLuint LoadShaders(const std::string& VertexShaderCode, const std::string& FragmentShaderCode){

  char const * VertexSourcePointer = VertexShaderCode.c_str();
  char const * FragmentSourcePointer = FragmentShaderCode.c_str();
  GLint Result = GL_FALSE;
  int InfoLogLength;

  // Create the shaders
  GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

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

void RenderData::addDistMarker(const arr& a, const arr& b, int s, double size, const arr& color){
  if(distMarkers.markerObj==-1){
    distMarkers.markerObj=items.N;
    rai::Mesh m;
    m.setCone(size, size);
    m.translate(0.,0.,-size);
    m.C={1.,0.,1.};
    add(0, _marker).mesh(m, .9);
  }
  CHECK_EQ(color.N, 3, "");
  distMarkers.pos.append(a);
  distMarkers.pos.append(b);
  distMarkers.color.append(color);
  distMarkers.slices.append(s);
}

void RenderData::addText(const char* text, float x, float y, float size){
  std::shared_ptr<RenderText> txt = make_shared<RenderText>();
  texts.append(txt);
//  read_png(txt->texImg, pngTexture, false);
  txt->x = x;
  txt->y = y;
  txt->scale = size;
  txt->text = text;
}

void RenderData::setText(const char* text){
  if(!texts.N) addText(text, 10., 20., 1.);
  else{
    std::shared_ptr<RenderText>& txt = texts(0);
    txt->text = text;
  }
}

void RenderData::addQuad(const byteA& img, float x, float y, float w, float h){
  std::shared_ptr<RenderQuad> quad = make_shared<RenderQuad>();
  quads.append(quad);
  quad->img = img;

  if(w<0.) w=h/img.d0*img.d1;
  if(h<0.) h=w/img.d1*img.d0;
  quad->vertices = { x,     y + h,   0.0f, 0.0f ,
                     x,     y,       0.0f, 1.0f ,
                     x + w, y,       1.0f, 1.0f ,
                     x,     y + h,   0.0f, 0.0f ,
                     x + w, y,       1.0f, 1.0f ,
                     x + w, y + h,   1.0f, 0.0f };

}

RenderData& RenderData::addStandardScene(bool addFloor){
  arr floorColor = opt.floorColor;
  if(!floorColor.N) floorColor = arr{.4, .45, .5};
  if(!lights.N){
    arr lights = opt.lights;
    if(!lights.N) lights = {-3.,2.,3., 3.,0.,4.};
    lights.reshape(-1,3);
    for(uint i=0; i<lights.d0; i++){
      addLight(lights[i], {0.,0.,1.}, shadowHeight);
//      addLight({3.,0.,4.}, {0.,0.,1.});
    }
  }
  if(addFloor && floorColor.N==3){ // floor
    rai::Mesh m;
    m.setQuad();
    m.scale(10., 10., 0.);
    m.C = floorColor;
    add().mesh(m);
  }
  return *this;
}

RenderData& RenderData::clear(){
  items.clear();
  texts.clear();
  quads.clear();
  distMarkers.markerObj=-1;
  distMarkers.pos.clear();
  distMarkers.color.clear();
  distMarkers.slices.clear();
  renderCount=0;
  slice=-1;
  return *this;
}

void RenderText::glRender(GLuint progText_color, const RenderFont& font, float height){
  if(!font.characters.N) return;

  CHECK(initialized, "");
  glUniform3f(progText_color, 0., 0., 0.); //color.x, color.y, color.z);
  glActiveTexture(GL_TEXTURE0);

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);

  glBindVertexArray(vao);

  float _x = x, _y = height-y;

  // iterate through all characters
  for(uint i=0;i<text.N;i++){
    char c = text(i);
    if(c=='\n'){
      _x = x;
      _y -= font.characters('A').size_y + 8;
      continue;
    }
    RenderFont::Character ch = font.characters(c);

    float xpos = _x + ch.offset_x * scale;
    float ypos = _y - (ch.size_y - ch.offset_y) * scale;

    float w = ch.size_x * scale;
    float h = ch.size_y * scale;
    // update VBO for each character
    float vertices[6][4] = {
      { xpos,     ypos + h,   0.0f, 0.0f },
      { xpos,     ypos,       0.0f, 1.0f },
      { xpos + w, ypos,       1.0f, 1.0f },

      { xpos,     ypos + h,   0.0f, 0.0f },
      { xpos + w, ypos,       1.0f, 1.0f },
      { xpos + w, ypos + h,   1.0f, 0.0f }
    };
    // update content of VBO memory
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); // be sure to use glBufferSubData and not glBufferData
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // render quad
    glBindTexture(GL_TEXTURE_2D, ch.texture);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindTexture(GL_TEXTURE_2D, 0);

    // now advance cursors for next glyph (note that advance is number of 1/64 pixels)
    _x += (ch.advance_x >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
  }

  glBindVertexArray(0);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void RenderText::glInitialize(){
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindVertexArray(0);
  initialized = true;
}

RenderText::~RenderText(){
  if(initialized){
    glDeleteBuffers(1, &vertexBuffer);
    glDeleteVertexArrays(1, &vao);
  }
  initialized=false;
}

void RenderQuad::glRender(){
  CHECK(initialized, "");
  glActiveTexture(GL_TEXTURE0);

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);

  glBindTexture(GL_TEXTURE_2D, textureBuffer);
  //only necessary because texture could change
  if(img.nd==2 || img.d2==1) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==2) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);

  glBindVertexArray(vao);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
}

void RenderQuad::glInitialize(){
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, vertices.p, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindVertexArray(0);

  //generate texture
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &textureBuffer);
  glBindTexture(GL_TEXTURE_2D, textureBuffer);
  if(img.nd==2 || img.d2==1) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==2) glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
  else if(img.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
  else NIY;
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  initialized=true;
}

RenderQuad::~RenderQuad(){
  if(initialized){
    glDeleteBuffers(1, &vertexBuffer);
    glDeleteTextures(1, &textureBuffer);
    glDeleteVertexArrays(1, &vao);
  }
  initialized=false;
}

void RenderData::report(std::ostream& os){
  os <<"RenderData: #obj: " <<items.N
    <<" #lights: " <<lights.N
   <<" #texts: " <<texts.N
  <<" #quads: " <<quads.N
  <<" #distMarkers: " <<distMarkers.pos.d0
  <<endl;
}

}//namespace
