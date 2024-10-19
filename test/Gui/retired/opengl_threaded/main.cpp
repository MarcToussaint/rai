#include <Gui/opengl.h>
#include <Core/thread.h>

void draw1(void*, OpenGL& gl){
  glStandardLight(nullptr, gl);
  glColor3f(1,0,0);
  //glDrawBox(1.,1.,1.);
  glutSolidTeapot(1.);
}

struct Proc:public Thread{
  OpenGL *gl=0;
  Proc(const char* name):Thread(name, 0.05){
    threadLoop();
  }
  ~Proc(){
    threadClose();
  }

  void open(){
    gl = new OpenGL(name);
    gl->add(draw1);
    gl->update();
  }
  void close(){
    delete gl;
  }
  void step(){
  }
};

void TEST(ThreadedOpenGL) {
  Proc gl1("gl1"),gl2("gl2"),gl3("gl3");
  rai::wait(2.);
  rai::wait();
  gl1.threadClose();
  gl2.threadClose();

  Proc *gli;
  rai::Array<rai::String> names;
  ThreadL procs;
  for (int i=0; i<20; ++i){
    names.append(STRING("many_"<<i));
    gli = new Proc(names(i));
    procs.append(gli);
  }
  rai::wait(5.);
  for(Thread* th : procs) th->threadClose();
}

int MAIN(int argc, char **argv){
  rai::initCmdLine(argc,argv);

  testThreadedOpenGL();

  return 0;
}
