#include <stdlib.h>
#include <unistd.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <Core/util.h>
#include <Gui/opengl.h>

void draw1(void*,OpenGL& gl){
  glStandardLight(nullptr, gl);
  glColor(1,0,0);
  glFrontFace(GL_CW);
//  glutSolidTeapot(1.);
  glDrawAxes(1.);
  glFrontFace(GL_CCW);
}

int main(int argc, char ** argv)
{
  // 1. Initialize EGL
  EGLDisplay eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);

  EGLint major, minor;
  eglInitialize(eglDisplay, &major, &minor);

  // 2. Select an appropriate configuration
  EGLConfig eglConfig;
  EGLint num_config;
  EGLint attribute_list[] = {
    EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
    EGL_CONFORMANT, EGL_OPENGL_BIT,
    EGL_COLOR_BUFFER_TYPE, EGL_RGB_BUFFER,
    EGL_LUMINANCE_SIZE, 0,
    EGL_RED_SIZE, 8,
    EGL_GREEN_SIZE, 8,
    EGL_BLUE_SIZE, 8,
    EGL_ALPHA_SIZE, 8,
    EGL_DEPTH_SIZE, 8,
    EGL_LEVEL, 0,
    EGL_BUFFER_SIZE, 24,
    EGL_NONE
  };
  //  EGLint attribute_list[] = {
//    EGL_RED_SIZE, 1,
//    EGL_GREEN_SIZE, 1,
//    EGL_BLUE_SIZE, 1,
//    EGL_NONE
//  };
  eglChooseConfig(eglDisplay, attribute_list, &eglConfig, 1, &num_config);

  // 3. Create a surface
  EGLint pbufferAttribs[] = {
    EGL_WIDTH, 400,
    EGL_HEIGHT, 400,
    EGL_NONE,
  };
  EGLSurface eglSurface = eglCreatePbufferSurface(eglDisplay, eglConfig, pbufferAttribs);
  //  eglSurface = eglCreateWindowSurface(eglDisplay, eglConfig, native_window, nullptr);
  CHECK_EQ(eglGetError(), EGL_SUCCESS, "");

  // 4. Bind the API
  eglBindAPI(EGL_OPENGL_API);

  // 5. Create a context and make it current
  EGLContext eglContext = eglCreateContext(eglDisplay, eglConfig, EGL_NO_CONTEXT, 
					   nullptr);
  CHECK_EQ(eglGetError(), EGL_SUCCESS, "");

  eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext);
  CHECK_EQ(eglGetError(), EGL_SUCCESS, "");

  int w=400, h=400;
  

  byteA img(h,w,3);
#if 1
  OpenGL gl("bla", 400, 400, true);
  gl.add(draw1,0);
//  gl.Draw(400,400, nullptr, true);
  gl.renderInBack(w,h);
  img = gl.captureImage;
#else
  glClearColor(1.0,0.0,0.0,1.0);
  glClear(GL_COLOR_BUFFER_BIT);
#endif

//  eglSwapBuffers( eglDisplay, eglSurface);
  glReadPixels(0,0,400,400,GL_RGB, GL_UNSIGNED_BYTE, img.p);

  write_ppm(img, "z.ppm", true);
//  write_ppm(convert<byte>(255.f*gl.captureDepth), "z.ppm", true);

  std::cout <<"DONE" <<std::endl;
  return EXIT_SUCCESS;
}
