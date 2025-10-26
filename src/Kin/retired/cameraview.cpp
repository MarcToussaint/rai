//===========================================================================


Sim_CameraView::Sim_CameraView(Var<Configuration>& _kin,
			       Var<byteA> _color,
			       Var<floatA> _depth,
			       double beatIntervalSec, const char* _cameraFrameName, bool _idColors, const byteA& _frameIDmap)
    : Thread("Sim_CameraView", beatIntervalSec),
    model(this, _kin, (beatIntervalSec<0.)),
    color(this, _color),
    depth(this, _depth),
    V(model.get()()) {
  if(_cameraFrameName) {
    Frame* f = model.get()->getFrame(_cameraFrameName);
    V.setCamera(f);
    V.selectSensor(f);
  }
  if(_idColors) {
    V.renderMode = V.seg;
    if(_frameIDmap.N)
      V.frameIDmap = _frameIDmap;
    else {
      NIY; //V.C.clear();;
      V.updateConfiguration(model.get());
    }
  } else {
    V.renderMode = V.visuals;
  }
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

Sim_CameraView::~Sim_CameraView() {
  threadClose();
}

void Sim_CameraView::step() {
  byteA img;
  floatA dep;
  V.updateConfiguration(model.get());
  V.computeImageAndDepth(img, dep);
  color.set() = img;
  depth.set() = dep;
}

arr Sim_CameraView::getFxycxy() {
  auto& c = V.currentCamera;
  return arr{c->cam.focalLength* c->cam.height, c->cam.focalLength* c->cam.height, .5*(c->cam.width-1.), .5*(c->cam.height-1.)};
}
