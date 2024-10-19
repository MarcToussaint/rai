#include <Gui/opengl.h>

byteA img;

void Draw(void*){
  glStandardScene(nullptr);

  glDisable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  glTexImage2D(GL_TEXTURE_2D, 0, 3, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);

  glBegin(GL_POLYGON);
  glTexCoord2f(0., 1.);  glVertex3f(0., 0., 1.);
  glTexCoord2f(1., 1.);  glVertex3f(1., 0., 1.);
  glTexCoord2f(1., 0.);  glVertex3f(1., 1., 1.);
  glTexCoord2f(0., 0.);  glVertex3f(0., 1., 1.);
  glEnd();

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);
}

int main(int argc, char **argv){
  read_ppm(img, "box.ppm", false);
  OpenGL gl;
  gl.add(Draw, nullptr);
  gl.background = img;
  gl.watch();
  return 0;
}

