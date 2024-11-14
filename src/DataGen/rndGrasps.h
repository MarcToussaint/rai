#include <Core/util.h>
#include <Kin/kin.h>

struct RndGrasps_Options {
      RAI_PARAM("rndGrasps/", int, verbose, 2)
      RAI_PARAM("rndGrasps/", int, dataPerShape, 20)
      RAI_PARAM("rndGrasps/", rai::String, filesPrefix, "shapenet/models/")
      RAI_PARAM("rndGrasps/", int, numShapes, -1)
      RAI_PARAM("rndGrasps/", int, startShape, 0)
      RAI_PARAM("rndGrasps/", rai::String, evaluationsFile, "z.dat")
      RAI_PARAM("rndGrasps/", int, simVerbose, 2)
      RAI_PARAM("rndGrasps/", double, simTau, .01)
      RAI_PARAM("rndGrasps/", double, gripperCloseSpeed, .001)
      RAI_PARAM("rndGrasps/", double, moveSpeed, .005)
      RAI_PARAM("rndGrasps/", double, pregraspNormalSdv, .2)
      RAI_PARAM("rndGrasps/", rai::String, trainingFile, "train.dat")
};

struct RndGrasps{
  RndGrasps_Options opt;

  RndGrasps();
  void run();

  void initialize(){}
  void getSamples(arr& X, arr& Contexts, arr& Scores, uint N);
  arr evaluateSample(const arr& x, const arr& context);
  void displaySamples(const arr& X, const arr& Contexts, const arr& Scores={});

private:
  bool addObject(rai::Mesh& pts, rai::Mesh& mesh);
  bool generateRndCandidate();
  arr evaluateCandidate();
public:
  StringA files;
  rai::Configuration C;
  void clearScene();
  bool createScene(const char* file, int idx, bool rndPose=true);
  void setRelGripperPose(const arr& pose, const char* objPts="objPts0");
};

void generateTrainingData();
void testLoadTrainingData();
bool addShapenetObj(rai::Configuration& C, const char* prefix, const char* file);
