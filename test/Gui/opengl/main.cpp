#include <Plot/plot.h>
#include <Gui/opengl.h>
#include <Geo/mesh.h>
#ifdef RAI_QT
#  include <QtGui/QApplication>
#endif

#include <GL/gl.h>
#include <GL/glut.h>
#include <Core/thread.h>
using namespace std;

/************ first test ************/

void draw1(void*,OpenGL& gl){
  glStandardLight(nullptr, gl);
  glColor(1,0,0);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
//  glDrawBox(1,.7,.5);
  glFrontFace(GL_CCW);
}

void TEST(Teapot) {
  OpenGL gl;
  gl.reportEvents = true;
  gl.add(draw1,0);
  gl.watch();
  cout <<"returned from watch - watch again" <<endl;
  gl.watch();
  cout <<"returned from 2nd watch - quit" <<endl;
}

/************ multiple views ************/

void TEST(MultipleViews) {
  byteA img;
  read_ppm(img,"box.ppm",false);
  OpenGL gl;
  gl.reportEvents=true;
  gl.reportSelects=true;
  gl.text <<"multiple views";
  gl.add(draw1,0);
  gl.addSubView(0,draw1,0);
  gl.addSubView(1,draw1,0);
  gl.setSubViewPort(1,.1,.4,.1,.4);
  gl.setSubViewPort(0,.6,.9,.6,.9);
  gl.views(0).img=&img;
  gl.views(0).text="little image";
  //gl.views(1).text="2nd teapot";
  gl.watch();
}

/************ grab test ************/

void TEST(Grab) {
  OpenGL gl("title",300,300);
  gl.add(draw1,0);
  cout <<"normal view - written to z.ppm " <<endl;
  gl.update("title", true);
  write_ppm(gl.captureImage,"z.1.ppm");
  write_ppm(convert<byte>(255.f*gl.captureDepth),"z.2.ppm");

  gl.watch();

  gl.camera.setPosition(0,0,10);
  gl.camera.setZRange(9,10);
  gl.camera.setHeightAbs(2);
  cout <<"orthogonal top view" <<endl;
  gl.watch();

  //grap the depth image from current view:
  gl.update(nullptr, true);
  cout <<"max " <<(int)gl.captureDepth.max() <<" min " <<(int)gl.captureDepth.min() <<endl;
  gl.watchImage(gl.captureDepth,true,1);
}

/************ second test ************/

static void draw2(void*, OpenGL& gl){
  glStandardLight(nullptr, gl);
  glDrawAxes(1.);
  glColor(1.,1.,1.);
  glDisable(GL_CULL_FACE);
}

void TEST(Mesh) {
  uint i,j,N=10;
  rai::Vector v;
  rai::Mesh mesh;

  //add points to the mesh
  mesh.V.resize(N*N,3);
  for(i=0;i<N;i++) for(j=0;j<N;j++){
    v.x=(double)j-N/2; v.y=(double)i-N/2;
    v.z=-.15*(v.x*v.x+v.y*v.y); //paraboloid
    //v.z=((i+j)&1); //up-down surface
    //v.z=2.*rnd.uni(); //random surface
    mesh.V(i*N+j,0)=v.x; mesh.V(i*N+j,1)=v.y; mesh.V(i*N+j,2)=v.z; //insert v in the list mesh.V
  }

  //connect them to a grid (i.e., define the triangle between the points)
  mesh.setGrid(N,N);
  //mesh.trinormals=true; //leads to uniform triangle colors
  //mesh.gridToStrips(grid); //alternative to triangle list -- but deletion is disabled

  //calculate normals
  mesh.computeNormals();

  //test deletion of some random triangles
  for(i=0;i<mesh.T.d0;i++) if(rnd.uni()<.1){ mesh.T(i,0)=mesh.T(i,1)=mesh.T(i,2)=0; }
  mesh.deleteUnusedVertices();

  //draw
  OpenGL gl;
  gl.text="testing Mesh";
  gl.add(draw2,0);
  gl.add(mesh);
  gl.watch();
}

/************ third test ************/

void TEST(Obj) {
  rai::Mesh mesh,mesh2;

  //mesh.readObjFile(FILE("../../external/3dmodel/obj/gipshand2-273k.obj"));
  mesh.readFile("base-male-nude.obj");
  //mesh.readObjFile(FILE("../../../3dmodel/obj/gipshand2-273k.obj"));
  //mesh.readObjFile(FILE("../../../3dmodel/obj/base-male-nude.obj"));
  mesh.scale(.1,.1,.1);
  mesh.computeNormals();
//  mesh2.readObjFile(FILE("magnolia.obj"));
//  mesh2.scale(.01,.01,.01);
//  mesh2.computeNormals();
  OpenGL gl;
  gl.text="testing Mesh";
  gl.add(draw2,0);
  gl.add(mesh);
  gl.watch();
  gl.clear();
  gl.add(draw2,0);
  gl.add(mesh2);
  gl.watch();
}

/************ fourth test ************/

/* menu test */

void menuCallback1(int i){
  cout <<"menu 1 callback: " <<i <<endl;
}
void menuCallback2(int i){
  cout <<"menu 2 callback: " <<i <<endl;
}
void menuCallback3(int i){
  cout <<"menu 3 callback: " <<i <<endl;
}

void TEST(Menu){
#if 0
  OpenGL gl;
  gl.text.clear() <<"press the right moust";
  gl.add(draw1,0);

  int submenu1, submenu2;

  submenu1 = glutCreateMenu(menuCallback1);
  glutAddMenuEntry((char*)"abc", 1);
  glutAddMenuEntry((char*)"ABC", 2);
  submenu2 = glutCreateMenu(menuCallback2);
  glutAddMenuEntry((char*)"Green", 1);
  glutAddMenuEntry((char*)"Red", 2);
  glutAddMenuEntry((char*)"White", 3);
  glutCreateMenu(menuCallback3);
  glutAddMenuEntry((char*)"9 by 15", 0);
  glutAddMenuEntry((char*)"Times Roman 10", 1);
  glutAddMenuEntry((char*)"Times Roman 24", 2);
  glutAddSubMenu((char*)"Messages", submenu1);
  glutAddSubMenu((char*)"Color", submenu2);
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  gl.watch();
#endif
}

/************ 5th test ************/

byteA texImg;
//static GLuint texName;
void draw5(void*, OpenGL& gl){
  glStandardScene(nullptr, gl);

#if 1
  glDisable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  if(texImg.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
  if(texImg.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.d1, texImg.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImg.p);

  glBegin(GL_POLYGON);
  glTexCoord2f(0., 1.);  glVertex3f(0., 0., 1.);
  glTexCoord2f(1., 1.);  glVertex3f(1., 0., 1.);
  glTexCoord2f(1., 0.);  glVertex3f(1., 1., 1.);
  glTexCoord2f(0., 0.);  glVertex3f(0., 1., 1.);
  glEnd();

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);
#endif

  glDrawTexQuad(texImg,
                0.0, 0.0, 1.5,
                1.0, 0.0, 2.5,
                1.0, 1.0, 2.5,
                0.0, 1.0, 1.5,
                3.,2.);

}

#ifdef RAI_PNG

#include <png.h>

void read_png(byteA &img, const char *file_name, bool swap_rows) {
  FILE *fp = fopen(file_name, "rb");

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  CHECK(png, "");

  png_infop info = png_create_info_struct(png);
  CHECK(info, "");

  if(setjmp(png_jmpbuf(png))) abort();

  png_init_io(png, fp);
  png_read_info(png, info);

  uint width      = png_get_image_width(png, info);
  uint height     = png_get_image_height(png, info);
  png_byte color_type = png_get_color_type(png, info);
  png_byte bit_depth  = png_get_bit_depth(png, info);

  // Read any color_type into 8bit depth, RGBA format.
  // See http://www.libpng.org/pub/png/libpng-manual.txt

  if(bit_depth == 16)
    png_set_strip_16(png);

  if(color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png);

  // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand_gray_1_2_4_to_8(png);

  if(png_get_valid(png, info, PNG_INFO_tRNS))
    png_set_tRNS_to_alpha(png);

  // These color_type don't have an alpha channel then fill it with 0xff.
  if(color_type == PNG_COLOR_TYPE_RGB ||
     color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

  if(color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_gray_to_rgb(png);

  png_read_update_info(png, info);

  img.resize(height, png_get_rowbytes(png,info));
  rai::Array<byte*> cpointers = img.getCarray();
  //    row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  //    for(int y = 0; y < height; y++) {
  //      row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
  //    }

  png_read_image(png, cpointers.p);

  img.resize(height, width, img.N/(height*width));

  fclose(fp);

  if(swap_rows) flip_image(img);
}

#endif

void TEST(Texture) {
  OpenGL gl;
  read_ppm(texImg, "box.ppm", false);
  //  read_png(texImg, "box.png", false);
//  remove_alpha_channel(texImg);
//  write_ppm(texImg, "z.ppm");
//  texName=glImageTexture(texImg);
  gl.background = texImg;
  gl.add(draw5,0);
  gl.watch();
}

//===========================================================================

void TEST(Texture2) {
  OpenGL gl;
  rai::Mesh m;
  m.readFile("owl.obj");
  read_png(m.texImg, "owl.png", true);
  gl.add(glStandardLight);
  gl.add(m);
  gl.watch();
}

//===========================================================================

void TEST(OfflineRendering){
  OpenGL gl("view", 40, 40, true);
  gl.add(draw1,0);
  gl.update(nullptr, true);
//  gl.renderInBack(200, 200);
  write_ppm(gl.captureImage,"z.ppm");
//  OpenGL gl2("captured", gl.captureImage.d1, gl.captureImage.d0);
//  gl2.watchImage(gl.captureImage, true, 1);
  cout <<"returned from watch - watch again" <<endl;
}

/************ test clicking on and identifying objects in the scene ************/

void draw3(void*, OpenGL& gl){
  glPushName(0xa0);
  glStandardLight(nullptr, gl);
  glColor(1,0,0);
  glutSolidTeapot(1.);
  glPopName();
}

void TEST(Select) {
  OpenGL gl;
  gl.add(draw3,0);
  gl.text <<"hover over objects and read cout...";
  //gl.selectOnHover=true;
  gl.reportSelects=true;
  gl.reportEvents=false;
  gl.watch();
}

/************ test clicking on and identifying objects in the scene ************/

void TEST(UI){
  OpenGL gl;
  glUI ui;
  gl.reportEvents=true;
  gl.add(draw1,0);
  gl.add(glDrawUI,&ui);
  gl.addHoverCall(&ui);
  gl.addClickCall(&ui);
  ui.addButton(100,100,"OK, this is it!");
  gl.watch();
}

void TEST(Image) {
  OpenGL gl;
  byteA img;
  read_ppm(img,"box.ppm",false);
  gl.watchImage(img,true,2);

  img=gl.captureImage;
  write_ppm(img,"z.ppm",false); //this turns out flipped!!! -> the capturing flips
}

//extern void qtCheckInitialized();

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  testTeapot();
  testOfflineRendering();
  testGrab();
  testMultipleViews();
  testUI();
  testSelect();
  testObj();
  testMesh();
  testTexture();
  testTexture2();
//  testMenu();
  testImage();

  return 0;
}

