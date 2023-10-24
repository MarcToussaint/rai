/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "module_VideoWriter.h"
#include "video.h"

REGISTER_MODULE(VideoEncoderModule)

VideoEncoderModule::VideoEncoderModule() : Thread("FloatA_Recorder"), fps(30), video(nullptr) {
}

void VideoEncoderModule::open() {
  rai::String nowStr;
  rai::getNowString();
  video = new VideoEncoder_libav_simple(STRING("z.video." << nowStr << ".avi"), fps);
}

void VideoEncoderModule::close() {
  timeTagFile.close();
  delete video;
  video = nullptr;
}

void VideoEncoderModule::step() {
  //-- grab from shared memory (necessary?)
  uint rev = img.readAccess();
  double time = img.data->revisionTime();
  byteA image = img();
  img.deAccess();

  //save image
  flip_image(image);
  video->addFrame(image);

  //save time tag
  rai::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);
  timeTagFile <<tag <<endl;
}
