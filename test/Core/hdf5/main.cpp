#include <Core/h5.h>

int read(const char* filename)
{
  double time = -rai::clockTime();
  H5_Reader L(filename);
  time += rai::clockTime();
  cout <<time <<"sec" <<endl;
  cout <<L.G <<endl;
  return 0; // successfully terminated
}

void write(const char* filename){

  H5_Writer H(filename);

  {
    intA x(3,2);
    x.setRandomPerm();
    std::cout <<x <<std::endl;
    H.add("test", x);
  }

  {
    arr x = rand({2,3,2});
    std::cout <<x <<std::endl;
    H.add("testarr", x);
  }
}

int main(int argn, char** argv){
  write("bla.h5");
  read("bla.h5");
  return 0;
}
