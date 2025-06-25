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
  RAI_PARAM("Render/", arr, backgroundColor, {})
  RAI_PARAM("Render/", arr, floorColor, {})
  RAI_PARAM("Render/", arr, lights, {})
};

enum RenderType { _solid, _shadow, _tensor, _text, _marker, _transparent, _all };

struct RenderAsset{
  floatA vertices, colors, normals;  //for 2D textures: colors are vertex-wise texture coordinates
  GLuint vao, vertexBuffer, colorBuffer, normalBuffer;
  shared_ptr<SharedTextureImage> _texture;
  SharedTextureImage& texture(){ if(!_texture) _texture=make_shared<SharedTextureImage>(); return *_texture; }
  GLenum mode=GL_TRIANGLES;
  uint textureDim=0;
  bool initialized=false;
  bool isTransparent=false;
  int version=-1;

  ~RenderAsset();
  void mesh(rai::Mesh &mesh, double avgNormalsThreshold=.9);
  void lines(const arr& lines, const arr& color);
  void pointCloud(const arr& points, const arr& color);
  void tensor(const floatA& vol, const arr& size={1.,1.,1.});

  //engine specific -> should be refactored
  void glInitialize();
  void glRender();
};

//an asset can be drawn multiple times in a scene, with different poses -> each time it is drawn is a RenderItem
struct RenderItem{
  std::shared_ptr<RenderAsset> asset;

  rai::Transformation X=0;
  arr scale;
  double cameraDist=-1.;
  RenderType type=_solid;
  byteA flatColor;

  RenderItem(const rai::Transformation& _X=0, RenderType _type=_solid) : X(_X), type(_type) {}
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
  GLuint vao, vertexBuffer, textureBuffer;
  bool initialized=false;

  ~RenderQuad();
  //engine specific -> should be refactored
  void glInitialize();
  void glRender();
};

struct DistMarkers {
  int markerObj=-1;
  arr pos;
  arr color;
  intA slices;
  void clear(){ pos.clear(); slices.clear(); }
};

struct RenderData {
  rai::Mutex dataLock;

  rai::Render_Options opt;

  rai::Camera camera;
  rai::Array<std::shared_ptr<RenderItem>> items;
  rai::Array<std::shared_ptr<rai::Camera>> lights;
  rai::Array<std::shared_ptr<RenderText>> texts;
  rai::Array<std::shared_ptr<RenderQuad>> quads;
  DistMarkers distMarkers;  
  RenderItem cylin; //predefined objects

  RenderType renderUntil=_all;
  bool renderFlatColors=false;
  int slice=-1;
  uint renderCount=0;

  struct ContextIDs{
    bool initialized=false;
    GLuint shadowFramebuffer, shadowTexture;
    GLuint prog_ID, prog_Projection_W, prog_ModelT_WM, prog_eyePosition_W, prog_ShadowProjection_W, prog_useShadow, prog_shadowMap, prog_numLights, prog_lightDirection_W, prog_FlatColor, prog_textureDim, prog_textureImage;
    GLuint progShadow, progShadow_ShadowProjection_W, progShadow_ModelT_WM;
    GLuint progTensor, progTensor_Projection_W, progTensor_ModelT_WM, progTensor_ModelScale, progTensor_eyePosition_W, progTensor_FlatColor, progTensor_tensorTexture;
    GLuint progMarker, progMarker_Projection_W, progMarker_ModelT_WM, progMarker_FlatColor;
    GLuint progText, progText_color, progText_useTexColor;
    RenderFont font;
  };

  RenderData();

  RenderAsset& add(const rai::Transformation& _X=0, RenderType _type=_solid);
  RenderAsset& addShared(std::shared_ptr<RenderItem> _item, const rai::Transformation& _X=0, RenderType _type=_solid);

  void addLight(const arr& pos, const arr& focus, double heightAbs=5.);
  void addAxes(double scale, const rai::Transformation& _X);
  void addDistMarker(const arr& a, const arr& b, int s=-1, double size=.1, const arr& color=arr{1.,0.,1.});
  void addText(const char* text, float x, float y, float size);
  void setText(const char* text);
  void addQuad(const byteA& img, float x, float y, float w, float h);

  RenderData& addStandardScene(bool addFloor=true);
  RenderData& clear();

  //engine specific -> should be refactored
  void ensureInitialized(OpenGL &gl);
  virtual void glDraw(OpenGL &gl);
  void glDeinitialize(OpenGL &gl);
  void visualsOnly(bool _visualsOnly=true){ if(_visualsOnly) renderUntil=_tensor; else renderUntil=_all; }

//private:
  void renderObjects(GLuint idT_WM, const uintA& sortedObjIDs, RenderType type, GLint idFlatColor=-1, GLint idScale=-1, GLint idTextureDim=-1);

  void report(std::ostream& os);
};

}//namespace
