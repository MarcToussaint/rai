/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/*
 * image_pub.hpp
 *
 *  Created on: Jul 11, 2014
 *      Author: ingo
 */

#ifndef IMAGE_PUB_H_
#define IMAGE_PUB_H_

#include <string>
#include "../Perception/pixel_format.h"

namespace rai {
template<class T> struct Array;
}

namespace rai {
struct sImagePublisher;

class ImagePublisher {
 private:
  sImagePublisher* s;
 public:
  /** Create an image publisher for the give camera name and expecting that images have the specified pixel format. */
  ImagePublisher(const std::string& topic, const std::string& camera_name, const PixelFormat pix_fmt);
  ~ImagePublisher();

  /** Publishes an image with the given capture timestamp. After this method returns, the content of "image" is
   * no longer needed, even though publishing may occur in the background.
   */
  void publish(const rai::Array<unsigned char>& image, double timestamp);
};

void init_image_publishers(int argc, char* argv[], const char* name, bool install_sigint_handler);
/** Calls init_image_publishers(4) with true for the install_signal_handler argument */
void init_image_publishers(int argc, char* argv[], const char* name);
bool process_image_callbacks();
void ros_shutdown();
}

#endif /* IMAGE_PUB_H_ */
