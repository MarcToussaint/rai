
#include "image.h"
#include "misc.h"

namespace felzenszwalb {
template <class T>
image<T>::image(const int width, const int height, const bool init) {
  w = width;
  h = height;
  data = new T[w * h];  // allocate space for image data
  access = new T*[h];   // allocate space for row pointers
  
  // initialize row pointers
  for (int i = 0; i < h; i++)
    access[i] = data + (i * w);  
  
  if (init)
    memset(data, 0, w * h * sizeof(T));
}

// nils: this constructor does not allocate ANY image data
template <class T>
image<T>::image(const int width, const int height, const int dummy) {
  w = width;
  h = height;
  data = NULL;
  access = NULL;
}


template <class T>
image<T>::~image() {
   if (data != NULL);
      delete [] data; 
   if (access != NULL)
      delete [] access;
}

template <class T>
void image<T>::init(const T &val) {
  T *ptr = imPtr(this, 0, 0);
  T *end = imPtr(this, w-1, h-1);
  while (ptr <= end)
    *ptr++ = val;
}


template <class T>
image<T> *image<T>::copy() const {
  image<T> *im = new image<T>(w, h, false);
  memcpy(im->data, data, w * h * sizeof(T));
  return im;
}

// explicit instantiation
template class image<float>;
template class image<int>;
template class image<unsigned int>;
template class image<long>;
template class image<short>;
template class image<rgb>;
template class image<uchar>;

}  // namespace felzenszwalb


