#include "opengl.h"
#include <GL/gl.h>

//#include <Geo/geo.h>
#include "../Geo/mesh.h"

enum RenderType { _solid, _shadow, _transparent, _marker, _text, _all };

struct RenderObject{
  rai::Transformation X=0;
  floatA vertices, colors, normals;
  GLuint vao, vertexBuffer, colorBuffer, normalBuffer;
  double cameraDist=-1.;
  RenderType type=_solid;
  GLenum mode=GL_TRIANGLES;
  int version=-1;
  int selection=-1;
  bool initialized=false;

  ~RenderObject();
  void mesh(rai::Mesh &mesh, const rai::Transformation& _X, double avgNormalsThreshold=.9, RenderType _type=_solid);
  void lines(const arr& lines, const arr& color, const rai::Transformation& _X, RenderType _type=_marker);
//private:
  void glRender();
  void glInitialize();
};

struct RenderFont {
  struct Character {  GLuint texture;  int size_x, size_y, offset_x, offset_y, advance_x;  };
  rai::Array<Character> characters;

  void glInitialize();
};

struct RenderText{
  GLuint vao, vertexBuffer;
  str text;
  float x, y, scale;
  bool initialized=false;

  void glRender(GLuint progText_color, const RenderFont& font, float height);
  void glInitialize();
};

struct DistMarkers {
  int markerObj=-1;
  arr pos;
  intA slices;
};

struct RenderScene : GLDrawer{
  rai::Camera camera;
  rai::Array<std::shared_ptr<RenderObject>> objs;
  rai::Array<std::shared_ptr<rai::Camera>> lights;
  rai::Array<std::shared_ptr<RenderText>> texts;
  DistMarkers distMarkers;

  RenderType dontRender=_all;
  int slice=-1;
  uint renderCount=0;

  struct ContextIDs{
    bool initialized=false;
    GLuint prog_ID, prog_Projection_W, prog_ViewT_CW, prog_ModelT_WM, prog_ShadowProjection_W, prog_shadowMap, prog_numLights, prog_lightDirection_C;
    GLuint progShadow_ID, progShadow_ShadowProjection_W, progShadow_ModelT_WM;
    GLuint shadowFramebuffer, shadowTexture;
    GLuint progMarker, progMarker_Projection_W, progMarker_ModelT_WM;
    GLuint progText, progText_color;
    RenderFont font;
  };

  std::map<OpenGL*, ContextIDs> contextIDs;

  RenderObject& add();
  void addLight(const arr& pos, const arr& focus, double heightAbs=5.);

  void addAxes(double scale, const rai::Transformation& _X);
  void addDistMarker(const arr& a, const arr& b, int s);
  void addText(const char* text, float x, float y, float size);
  void setText(const char* text);
  void clearObjs();

  void glInitialize(OpenGL &gl);
  void glDraw(OpenGL &gl);
  void glDeinitialize(OpenGL &gl);

//private:
  void renderObjects(GLuint idT_WM, const uintA& sorting, RenderType type);
};
