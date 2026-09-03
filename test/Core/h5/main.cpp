#include <Core/h5.h>

void write(const char* filename){
  rai::H5_Writer h5(filename);

  {
    intA x(3,2);
    x.setRandomPerm();
    std::cout <<x <<std::endl;
    h5.write("test", x);
  }

  {
    arr x = rand({2,3,2});
    std::cout <<x <<std::endl;
    h5.write("testarr", x);
  }

  {
    arrA x;
    x.append(arr{1.,2.,3.});
    x.append(arr{4.,5.});
    std::cout <<x <<std::endl;
    h5.writeA("arr-array", x);
  }

  {
    rai::Graph dict = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", rai::String("BLA")} };
    dict["a"] = 5.; //overwriting with same type
    dict["d"] = arr{{2,1},{1., 2.5}}; //new element
    dict["e"] = StringA{"alpha", "beta"};
    cout <<dict <<endl;
    h5.writeDict("info", dict);
  }
}

int read(const char* filename){
  double time = -rai::clockTime();
  rai::H5_Reader h5(filename);
  auto a = h5.read<int>("test");
  auto b = h5.read<double>("testarr");
  auto c = h5.readDict("info");
  cout <<a <<'\n' <<b <<'\n' <<c <<endl;
  time += rai::clockTime();
  cout <<time <<"sec" <<endl;
  return 0; // successfully terminated
}


int main(int argn, char** argv){
  write("bla.h5");
  read("bla.h5");
  return 0;
}
