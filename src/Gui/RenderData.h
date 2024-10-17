#pragma once

#include "opengl.h" //needed to include glew-etc before gl.h

#include <GL/gl.h>

//#include <Geo/geo.h>
#include "../Core/util.h"
#include "../Geo/mesh.h"

#include <map>

namespace rai {

struct Render_Options {
  RAI_PARAM("Render/", bool, userShaderFiles, false)
  RAI_PARAM("Render/", bool, flatColors, false)
  RAI_PARAM("Render/", bool, useShadow, true)
  RAI_PARAM("Render/", arr, floorColor, {})
  RAI_PARAM("Render/", arr, lights, {})
};

enum RenderType { _solid, _shadow, _marker, _transparent, _text, _all };

struct RenderObject{
  rai::Transformation X=0;
  floatA vertices, colors, normals;
  GLuint vao, vertexBuffer, colorBuffer, normalBuffer;
  double cameraDist=-1.;
  RenderType type=_solid;
  GLenum mode=GL_TRIANGLES;
  int version=-1;
  int selection=-1;
  byteA flatColor;
  bool initialized=false;

  ~RenderObject();
  void mesh(rai::Mesh &mesh, const rai::Transformation& _X=0, double avgNormalsThreshold=.9, RenderType _type=_solid);
  void lines(const arr& lines, const arr& color, const rai::Transformation& _X=0, RenderType _type=_marker);
  void pointCloud(const arr& points, const arr& color, const rai::Transformation& _X=0, RenderType _type=_solid);

  //engine specific -> should be refactored
  void glRender();
  void glInitialize();
};

struct RenderFont {
  struct Character {  GLuint texture;  int size_x, size_y, offset_x, offset_y, advance_x;  };
  rai::Array<Character> characters;

  //engine specific -> should be refactored
  void glInitialize();
};

struct RenderText{
  GLuint vao, vertexBuffer;
  rai::String text;
  float x, y, scale;
  bool initialized=false;

  ~RenderText();
  //engine specific -> should be refactored
  void glRender(GLuint progText_color, const RenderFont& font, float height);
  void glInitialize();
};

struct RenderQuad {
  byteA img;
  floatA vertices;
  GLuint vao, vertexBuffer, texture;
  bool initialized=false;

  ~RenderQuad();
  //engine specific -> should be refactored
  void glRender();
  void glInitialize();
};

struct DistMarkers {
  int markerObj=-1;
  arr pos;
  intA slices;
  void clear(){ pos.clear(); slices.clear(); }
};

struct RenderData {
  Mutex dataLock;

  Render_Options opt;

  rai::Camera camera;
  rai::Array<std::shared_ptr<RenderObject>> objs;
  rai::Array<std::shared_ptr<rai::Camera>> lights;
  rai::Array<std::shared_ptr<RenderText>> texts;
  rai::Array<std::shared_ptr<RenderQuad>> quads;
  DistMarkers distMarkers;  
  RenderObject cylin; //predefined objects

  RenderType renderUntil=_all;
  bool renderFlatColors=false;
  int slice=-1;
  uint renderCount=0;

  struct ContextIDs{
    bool initialized=false;
    GLuint prog_ID, prog_Projection_W, prog_ViewT_CW, prog_ModelT_WM, prog_ShadowProjection_W, prog_useShadow, prog_shadowMap, prog_numLights, prog_lightDirection_C, prog_FlatColor;
    GLuint progShadow_ID, progShadow_ShadowProjection_W, progShadow_ModelT_WM;
    GLuint shadowFramebuffer, shadowTexture;
    GLuint progMarker, progMarker_Projection_W, progMarker_ModelT_WM;
    GLuint progText, progText_color, progText_useTexColor;
    RenderFont font;
  };

  RenderData();

  RenderObject& add();

  void addLight(const arr& pos, const arr& focus, double heightAbs=5.);
  void addAxes(double scale, const rai::Transformation& _X);
  void addDistMarker(const arr& a, const arr& b, int s=-1, double size=.1);
  void addText(const char* text, float x, float y, float size);
  void setText(const char* text);
  void addQuad(const byteA& img, float x, float y, float w, float h);

  RenderData& addStandardScene();
  RenderData& clear();

  //engine specific -> should be refactored
  void ensureInitialized(OpenGL &gl);
  virtual void glDraw(OpenGL &gl);
  void glDeinitialize(OpenGL &gl);

//private:
  void renderObjects(GLuint idT_WM, const uintA& sortedObjIDs, RenderType type, GLuint idFlatColor);
};

}//namespace
