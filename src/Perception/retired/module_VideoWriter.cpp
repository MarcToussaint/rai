#include "module_VideoWriter.h"
#include "video.h"

REGISTER_MODULE(VideoEncoderModule)

VideoEncoderModule::VideoEncoderModule():Module("FloatA_Recorder"), fps(30), video(NULL){
}

void VideoEncoderModule::open(){
  MT::String nowStr;
  MT::getNowString();
  video = new VideoEncoder_libav_simple(STRING("z.video." << nowStr << ".avi"), fps);
}


void VideoEncoderModule::close(){
  timeTagFile.close();
  delete video;
  video = NULL;
}

void VideoEncoderModule::step(){
  //-- grab from shared memory (necessary?)
  uint rev = img.readAccess();
  double time = img.var->revisionTime();
  byteA image = img();
  img.deAccess();

  //save image
  flip_image(image);
  video->addFrame(image);

  //save time tag
  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);
  timeTagFile <<tag <<endl;
}
