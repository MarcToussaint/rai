
#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_GE
#undef CHECK_LE
#include <torch/script.h>
#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_GE
#undef CHECK_LE

#include <Core/array.h>
#include <Core/util.h>


arr convert_Tensor2arrRef(const torch::Tensor& tensor) {
  torch::Tensor tmp = tensor.cpu().to(torch::kDouble);
  return arr(tmp.data_ptr<double>(), sizeof(torch::kDouble)*tmp.numel(), false);
}

torch::Tensor convert_arr2Tensor(const arr& array) {
  return torch::from_blob(array.p, {array.d0}, torch::kDouble); //.to(torch::kFloat); //.to(ImS::defaultTorchDevice);
}


void test() {
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
  
  arr a = arr{1.0, 2.0};
  torch::Tensor a_t = convert_arr2Tensor(a);
  std::cout << a_t << std::endl;
  a(0) = 3.0;
  std::cout << a_t << std::endl;
}


int MAIN(int argc, char **argv){
  rai::initCmdLine(argc, argv);
  
  test();
  
  return 0;
}
