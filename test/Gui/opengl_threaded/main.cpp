#include <Gui/opengl.h>
#include <Core/thread.h>

void draw1(void*){
  glStandardLight(NULL);
  glColor3f(1,0,0);
  //glDrawBox(1.,1.,1.);
  glutSolidTeapot(1.);
}

struct Proc:public Thread{
  OpenGL *gl;
  Proc(const char* name):Thread(name, 0.01){}
  void open(){
    gl = new OpenGL(name);
    gl->add(draw1);
    gl->update();
  }
  void close(){
    delete gl;
  }
  void step(){
    gl->processEvents();
  }
};

void TEST(ThreadedOpenGL) {
  Proc gl1("gl1"),gl2("gl2"),gl3("gl3");
  gl1.threadLoop();
  gl2.threadLoop();
  mlr::wait(2.);
  gl1.threadClose();
  gl2.threadClose();

  Proc *gli;
  mlr::Array<mlr::String> names;
  ThreadL procs;
  for (int i=0; i<20; ++i){
    names.append(STRING("many_"<<i));
    gli = new Proc(names(i));
    gli->threadLoop();
    procs.append(gli);
  }
  mlr::wait(5.);
  close(procs);
}

int MAIN(int argc, char **argv){
  mlr::initCmdLine(argc,argv);

  testThreadedOpenGL();

  return 0;
}
