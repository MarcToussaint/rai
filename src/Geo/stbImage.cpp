#include "stbImage.h"

#ifndef RAI_NO_STB_IMPL
#  define STB_IMAGE_IMPLEMENTATION
#endif
#include "stb_image.h"

namespace rai{

byteA loadImage(const char* imgFile){
  int width, height, nrComponents;
  unsigned char* data = stbi_load(imgFile, &width, &height, &nrComponents, 0);
  byteA img;
  if(data) {
    img.resize(height, width, nrComponents);
    memmove(img.p, data, img.N);
    if(nrComponents==1){
      make_RGB(img);
    }else if(nrComponents==4){
      img.reshape(height*width, 4);
      img.delColumns(-1);
      img.reshape(height, width, 3);
    }
  } else {
    LOG(-1) <<"failed to load image file: " <<imgFile;
  }
  stbi_image_free(data);

  return img;
}

}
