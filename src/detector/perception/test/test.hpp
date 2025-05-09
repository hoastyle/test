#include <atomic>
#include <deque>
#include <shared_mutex>
#include <thread>  // NOLINT
#include <unordered_map>
#include <vector>

#include "core/GlobalCommonConfig.hpp"
#include "core/MMMessage.hpp"
#include "detector/BaseDetector.hpp"
#include "runtime/System.hpp"
#include "util/Util.hpp"
#include "pcl/LidarImageDetection.hpp"
#include "ceres/ceres.h"
#include "basic/CircularBuffer.hpp"

namespace mm {

class CraneStatus;
class PLCProcess;
class Transformer;

typedef enum {
  AIV_LEFT_LONG = 0,
  AIV_LEFT_SHORT,
  AIV_RIGHT_SHORT,
  AIV_RIGHT_LONG,
  AIV_MAX
} AIV_SOURCE_TYPE;

struct AIV_CNTR_LOCK {
  bool done;
  int id;
  std::string str;
  PointCloudXYZIPtr raw;
  std::vector<PointCloudXYZIPtr> vLock;
  std::vector<Eigen::Vector3f> vLockCenter;
  Eigen::Vector3f lockLockDis;

  AIV_CNTR_LOCK() {
    done        = false;
    id          = AIV_LEFT_LONG;
    str         = "";
    raw         = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    lockLockDis = Eigen::Vector3f::Zero();
  }

  void Reset() {
    done = false;
    id   = AIV_LEFT_LONG;
    str  = "";
    raw->clear();
    vLock.clear();
    vLockCenter.clear();
    lockLockDis = Eigen::Vector3f::Zero();
  }

  void Calculate();
};

class test : public BaseDetector {
 public:
  test();
  ~test();

  // Node
  int Init(const YAML::Node& config, int index) override;
  int Start() override;
  int Close() override;
  int Reset() override;
  int Stop() override;
  int updateConfig(
      const YAML::Node& config, const std::vector<std::string>& keys) override;
  void onLidarXYZIFrameReady(const MM_DEVICE& dev, PointCloudXYZIPtr cloud,
      const std::string& property = "null") override;

  bool checkFBSafety(PointCloudXYZIPtr pOri, bool isLeft);
  bool checkLoadSafety(PointCloudXYZIPtr pOri, bool isLeft, int index);

 private:
  void detectorThread();
  void readConfig(
      const YAML::Node& config, YoloParam& yoloParam1, YoloParam& yoloParam2);
  void processTrans();
  bool prepareData();
  bool checkTruckType();
  bool checkOpsMode(uint8_t& stage);
  void safetyHandler(std::string scene, bool isDanger, int source);

  // functions
  void detCntrLock(AIV_CNTR_LOCK& lock);
  void sortLockHole(AIV_CNTR_LOCK& lock);
  void preCollisionCheck();
  void landCollisionCheck();

  void onDetectionReady(const MMMessage&);

 private:
  std::shared_ptr<Transformer> mpTransformer;
  std::shared_ptr<CraneStatus> mpCraneStatus;
  std::shared_ptr<PLCProcess> mpPLCProcess;
  std::unique_ptr<AivTruckDet> mpAivTruckDet;
  std::unique_ptr<CntrLockHoleDet> mpCntrLockHoleDet;

  int mDebugLevel;
  std::string mDebugDir;

  std::vector<Eigen::Matrix4d> mvLvx2Uniform;
  std::vector<std::shared_ptr<mm::Lock>> mvMutexLvxData;
  std::vector<std::shared_ptr<CircularBuffer<PointCloudXYZIPtr>>>
      mvLidarBuffer;
  std::vector<PointCloudXYZIPtr> mvLidarData;
  std::vector<uint32_t> mvFrameIndex;
  std::vector<uint32_t> mvLastIndex;

  int mMergedFrames;
  uint8_t mStage;

  // Lock
  float mfLockRes;
  std::vector<float> mvLockDepthRoi;
  std::vector<float> mvLockHeightRoi;
  std::vector<float> mvLockWidthRoi;

  // Aiv FB precheck
  bool mbIsAivFBSaved;
  bool mbIsAivFBUnSafetySaved;
  bool mbIsAivFBLockSaved;
  int mCheckAivCount;
  float mfCheckAivStartHeight;
  float mfAivPressBlockGap;
  float mfCheckAivDepthStep;
  float mfCheckAivDepthRange;
  float mfCheckAivRes;
  float mfBlockWidth;
  int mCheckAivPointThres;
  std::vector<float> mvCheckAivFBRoiMin;
  std::vector<float> mvCheckAivFBRoiMax;
  std::vector<float> mvCheckAivSafeDis;
  std::vector<float> mvCheckAivLockRoiMin;
  std::vector<float> mvCheckAivLockRoiMax;

  float mfAivMoveThres;
  std::atomic_bool mbAivFBReady;
  CircularBuffer<float> mAivGuideBuffer;
  MMMHandle mDetectionReadyHandle;

  // Aiv load check
  bool mbIsAivLandSaved;
  bool mbIsAivLandUnSafetySaved;
  std::vector<float> mvLoadAivLockRoiMin;
  std::vector<float> mvLoadAivLockRoiMax;
};

}  // namespace mm
