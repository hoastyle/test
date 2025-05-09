#include "test.hpp"

#include "policy/MMAivCheckStatus.hpp"
#include "detector/CDetectorRegister.hpp"
#include "transform/Transformer.hpp"
#include "audio/TextToSpeech.hpp"
#include "core/CraneStatus.hpp"
#include "core/GlobalCommonConfig.hpp"
#include "core/PLCProcess.hpp"
#include "log/Log.hpp"
#include "plc/MMStatus.hpp"
#include "runtime/System.hpp"

DETECTOR_REGISTER(mm::test)

namespace mm {
test::test() {}

test::~test() {}

int test::Init(const YAML::Node& config, int index) {
  if (!mbInitialized) {
    setNodeName(FOLDERNAME);
    setNodeIndex(index);

    YoloParam yoloParam1, yoloParam2;
    readConfig(config, yoloParam1, yoloParam2);

    mpCraneStatus = CraneStatus::getSingleton();
    mpPLCProcess  = PLCProcess::getSingleton();
    mpTransformer = Transformer::getSingleton();

    for (int i = 0; i < AIV_MAX; ++i) {
      std::shared_ptr<CircularBuffer<PointCloudXYZIPtr>> pBuffer =
          std::make_shared<CircularBuffer<PointCloudXYZIPtr>>(mMergedFrames);
      mvLidarBuffer.push_back(pBuffer);
      std::shared_ptr<mm::Lock> pLvxMutex = std::make_shared<mm::Lock>();
      mvMutexLvxData.emplace_back(pLvxMutex);
      mvFrameIndex.push_back(0);
      mvLastIndex.push_back(0);
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      mvLvx2Uniform.push_back(trans);
      PointCloudXYZIPtr pLvxData(new PointCloudXYZI);
      mvLidarData.push_back(pLvxData);
    }

    DebugParam debugParam;
    DepthImgParam depthImgParam;
    AivParam aivImgParam(mvCheckAivFBRoiMin, mvCheckAivFBRoiMax, mfCheckAivRes,
        mfCheckAivDepthStep, mfCheckAivDepthRange, mCheckAivPointThres);
    mpAivTruckDet =
        std::make_unique<AivTruckDet>(debugParam, aivImgParam, yoloParam1);
    mpCntrLockHoleDet = std::make_unique<CntrLockHoleDet>(
        debugParam, depthImgParam, yoloParam2);

    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << getNodeIndex();
    std::string key = getNodeName() + "-" + ss.str();

    auto gcc  = GlobalCommonConfig::getSingleton();
    mDebugDir = gcc->getDebugPath() + key + "/";
    if (!std::filesystem::exists(mDebugDir)) {
      std::filesystem::create_directory(mDebugDir);
      MM_DEBUG("%s created", mDebugDir.c_str());
    }

    processTrans();

    mAivGuideBuffer.rset_capacity(5);
    MessageCallback DetectionReady = std::bind(
        &test::onDetectionReady, this, std::placeholders::_1);
    mDetectionReadyHandle = Subscribe(
        "onDetectionReady", DetectionReady, getNodeName(), getNodeIndex());

    mbExitDetector = false;
    std::function<void(void)> func =
        std::bind(&test::detectorThread, this);
    mDetectorThread = std::thread(func);

    mbInitialized = true;
  }

  return MM_STATUS_OK;
}

int test::Start() {
  MM_INFO("Start");

  return BaseDetector::Start();
}

int test::Stop() {
  MM_INFO("Stop");
  mCheckAivCount = 0;
  int source     = AIV_FB_CHECK_DANGER::LIDAR_AIV_FB_DANGER;
  safetyHandler("AivFB", false, source);

  return BaseDetector::Stop();
}

int test::Close() {
  MM_INFO("Close");

  if (mDetectorThread.joinable()) {
    mDetectorThread.join();
  }

  mbInitialized = false;
  return MM_STATUS_OK;
}

int test::Reset() {
  MM_INFO("Reset");

  for (int i = 0; i < AIV_MAX; ++i) {
    mm::WriteLock lock(*mvMutexLvxData[i]);
    mvLidarBuffer[i]->clear();
    mvLidarData[i]->clear();
    mvLastIndex[i] = 0;
  }

  mbIsAivFBUnSafetySaved = false;
  mbIsAivFBLockSaved     = false;
  mbIsAivFBSaved         = false;
  mbAivFBReady           = false;

  mbIsAivLandSaved         = false;
  mbIsAivLandUnSafetySaved = false;

  mCheckAivCount = 0;
  return MM_STATUS_OK;
}

int test::updateConfig(
    const YAML::Node& config, const std::vector<std::string>& keys) {
  return MM_STATUS_OK;
}

void test::readConfig(
    const YAML::Node& config, YoloParam& yoloParam1, YoloParam& yoloParam2) {
  mDebugLevel   = config["debug_level"].as<int>();
  mMergedFrames = config["merge_frames"].as<int>();

  mfCheckAivStartHeight = config["pre_check"]["start_height"].as<float>();
  mfAivPressBlockGap    = config["pre_check"]["press_block_gap"].as<float>();
  mvCheckAivFBRoiMin =
      config["pre_check"]["aiv_FB_roi_min"].as<std::vector<float>>();
  mvCheckAivFBRoiMax =
      config["pre_check"]["aiv_FB_roi_max"].as<std::vector<float>>();
  mvCheckAivSafeDis =
      config["pre_check"]["aiv_safe_y_dis"].as<std::vector<float>>();
  mvCheckAivLockRoiMin =
      config["pre_check"]["cntr_lock_roi_min"].as<std::vector<float>>();
  mvCheckAivLockRoiMax =
      config["pre_check"]["cntr_lock_roi_max"].as<std::vector<float>>();
  mfCheckAivRes        = config["pre_check"]["res"].as<float>();
  mfCheckAivDepthStep  = config["pre_check"]["depth_step"].as<float>();
  mfCheckAivDepthRange = config["pre_check"]["depth_range"].as<float>();
  mCheckAivPointThres  = config["pre_check"]["point_thres"].as<int>();
  mfBlockWidth         = config["pre_check"]["block_width"].as<float>();

  mfAivMoveThres = config["pre_check"]["move_thres"].as<float>();

  auto gcc              = GlobalCommonConfig::getSingleton();
  std::string modelPath = gcc->getModelPath();
  YAML::Node node1      = config["pre_check"]["model"];
  yoloParam1.configPath =
      modelPath + "yolo/" + node1["config_path"].as<std::string>();
  yoloParam1.weightsPath =
      modelPath + "yolo/" + node1["weight_path"].as<std::string>();
  yoloParam1.modelNetType     = node1["modelNetType"].as<int>();
  yoloParam1.modelPresionType = node1["modelPresionType"].as<int>();
  yoloParam1.demoThresh       = node1["model_thres"].as<float>();

  mvLoadAivLockRoiMin =
      config["load_check"]["cntr_lock_roi_min"].as<std::vector<float>>();
  mvLoadAivLockRoiMax =
      config["load_check"]["cntr_lock_roi_max"].as<std::vector<float>>();

  mfLockRes       = config["lock"]["step_res"].as<float>();
  mvLockDepthRoi  = config["lock"]["depth_roi"].as<std::vector<float>>();
  mvLockHeightRoi = config["lock"]["height_roi"].as<std::vector<float>>();
  mvLockWidthRoi  = config["lock"]["width_roi"].as<std::vector<float>>();

  YAML::Node node2 = config["lock"]["model"];
  yoloParam2.configPath =
      modelPath + "yolo/" + node2["config_path"].as<std::string>();
  yoloParam2.weightsPath =
      modelPath + "yolo/" + node2["weight_path"].as<std::string>();
  yoloParam2.modelNetType     = node2["modelNetType"].as<int>();
  yoloParam2.modelPresionType = node2["modelPresionType"].as<int>();
  yoloParam2.demoThresh       = node2["model_thres"].as<float>();
}

void test::processTrans() {
  EuclidTransform e;
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  for (int i = AIV_LEFT_LONG; i < AIV_MAX; ++i) {
    mpTransformer->getPointTrans(
        COORDINATE_LIVOX_1 + i, COORDINATE_UNIFORM3, e);
    e.getTransform(trans);
    trans.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
    mvLvx2Uniform[i]        = trans;
  }
}

void test::onLidarXYZIFrameReady(const MM_DEVICE& dev,
    PointCloudXYZIPtr cloud, const std::string& property) {
  if (!mbStarted) return;

  uint32_t frameIndex = cloud->header.seq;

  if (property.find("gantry1") != property.npos) {
    mm::WriteLock lock(*mvMutexLvxData[AIV_LEFT_LONG]);
    mvLidarBuffer[AIV_LEFT_LONG]->push_back(cloud);
  } else if (property.find("gantry2") != property.npos) {
    mm::WriteLock lock(*mvMutexLvxData[AIV_LEFT_SHORT]);
    mvLidarBuffer[AIV_LEFT_SHORT]->push_back(cloud);
  } else if (property.find("gantry3") != property.npos) {
    mm::WriteLock lock(*mvMutexLvxData[AIV_RIGHT_SHORT]);
    mvLidarBuffer[AIV_RIGHT_SHORT]->push_back(cloud);
  } else if (property.find("gantry4") != property.npos) {
    mm::WriteLock lock(*mvMutexLvxData[AIV_RIGHT_LONG]);
    mvLidarBuffer[AIV_RIGHT_LONG]->push_back(cloud);
  } else {
    MM_ERROR("Unexpected lidar %s-%u", property.c_str(), frameIndex);
    return;
  }

  mcvData.notify_all();
}

void test::detectorThread() {
  std::string threadName =
      std::to_string(getNodeIndex()) + "-" + Demangle(typeid(*this).name());
  setThreadName(threadName.c_str());

  while (!mbExitDetector) {
    WAIT_IF_NOT_RUNNING(mbStarted);
    WAIT_IF_NO_DATA(true);

    MilliTimer threadTimer;
    uint8_t stage = STAGE0;

    if (!checkOpsMode(stage)) continue;
    if (!checkTruckType()) continue;

    switch (stage) {
      case STAGE2:
        prepareData();
        if (mbAivFBReady) {
          preCollisionCheck();
        }
        break;
      case STAGE4:
        safetyHandler("AivFB", false, AIV_FB_CHECK_DANGER::LIDAR_AIV_FB_DANGER);
        prepareData();
        landCollisionCheck();
        break;
      default: break;
    }

    float fps = 0.f;
    calcFps(threadTimer.toc(), fps);
  }
}

bool test::prepareData() {
  std::vector<bool> vUpdated(AIV_MAX, false);
  for (int i = AIV_LEFT_LONG; i < AIV_MAX; ++i) {
    mm::ReadLock lock(*mvMutexLvxData[i]);

    if (static_cast<int>(mvLidarBuffer[i]->size()) != mMergedFrames) {
      MM_INFO("Wait for lidar data");
      return false;
    }

    if ((mvLidarBuffer[i]->back()->header.seq != mvLastIndex[i]) ||
        mvLidarData[i]->empty()) {
      vUpdated[i] = true;

      mvLidarData[i]->clear();
      for (int j = static_cast<int>(mvLidarBuffer[i]->size() - 1);
          j >= static_cast<int>(mvLidarBuffer[i]->size() - mMergedFrames);
          j--) {
        PointCloudXYZIPtr pCloud(new PointCloudXYZI);
        pcl::copyPointCloud(*mvLidarBuffer[i]->at(j), *pCloud);
        *mvLidarData[i] += *pCloud;
      }
    }

    mvFrameIndex[i] = mvLidarBuffer[i]->back()->header.seq;
  }

  for (int i = AIV_LEFT_LONG; i < AIV_MAX; ++i) {
    if (vUpdated[i]) {
      pcl::transformPointCloud(
          *mvLidarData[i], *mvLidarData[i], mvLvx2Uniform[i].cast<float>());
    }

    mvLastIndex[i] = mvFrameIndex[i];
  }

  return true;
}

bool test::checkTruckType() {
  MM_TRUCK_TYPE truckType = mpCraneStatus->getTruckType();

  if (truckType != TRUCK_AIV) {
    return false;
  }

  return true;
}

bool test::checkOpsMode(uint8_t& stage) {
  TOS_OPS_MODE mode = mpCraneStatus->getMode();
  if (mode.m.sceneType != ST_IT || mode.m.opsType != OT_LOAD ||
      mode.m.opsStep != OS_LAND) {
    MM_WARN("mode sceneType %d opsType %d opsStep %d", mode.m.sceneType,
        mode.m.opsType, mode.m.opsStep);

    return false;
  }

  stage = mode.m.opsStage;
  return true;
}

void test::detCntrLock(AIV_CNTR_LOCK& lock) {
  DebugParam debugParam;
  debugParam.dir   = mDebugDir;
  debugParam.str   = lock.str;
  debugParam.debug = mDebugLevel >= 2;
  mpCntrLockHoleDet->setDebugParam(debugParam);
  mpCntrLockHoleDet->Reset();
  if (mpCntrLockHoleDet->process(lock.raw)) {
    lock.done  = true;
    lock.vLock = mpCntrLockHoleDet->getLockHole();
    lock.Calculate();

    sortLockHole(lock);
  }
}

void test::sortLockHole(AIV_CNTR_LOCK& lock) {
  if (lock.vLock.size() != 2) {
    return;
  }

  if (boost::algorithm::starts_with(lock.str, "left") &&
      lock.vLockCenter[0][1] < lock.vLockCenter[1][1]) {
    std::swap(lock.vLockCenter[0], lock.vLockCenter[1]);
  }
  if (boost::algorithm::starts_with(lock.str, "right") &&
      lock.vLockCenter[0][1] > lock.vLockCenter[1][1]) {
    std::swap(lock.vLockCenter[0], lock.vLockCenter[1]);
  }
}

void test::preCollisionCheck() {
  bool is40FTSpreader(true);
  mpPLCProcess->isSpreader40FT(is40FTSpreader);

  int leftBlock  = is40FTSpreader ? AIV_LEFT_LONG : AIV_LEFT_SHORT;
  int rightBlock = is40FTSpreader ? AIV_RIGHT_LONG : AIV_RIGHT_SHORT;
  PointCloudXYZIPtr pLeft(new PointCloudXYZI);
  PointCloudXYZIPtr pRight(new PointCloudXYZI);
  pcl::copyPointCloud(*mvLidarData[leftBlock], *pLeft);
  pcl::copyPointCloud(*mvLidarData[rightBlock], *pRight);

  bool isLeftSafe = false, isRightSafe = false;
  isLeftSafe  = checkFBSafety(pLeft, true);
  isRightSafe = checkFBSafety(pRight, false);

  bool isSafe = (isLeftSafe || isRightSafe);
  int index   = mvFrameIndex[leftBlock];
  if (!mbIsAivFBSaved || (!isSafe && !mbIsAivFBUnSafetySaved)) {
    std::string env = (isSafe ? "_s" : "_d");
    pcl_utils::savePointCloud(mDebugDir,
        "AivFBOri_L_" + std::to_string(index) + env + ".pcd", *pLeft);
    pcl_utils::savePointCloud(mDebugDir,
        "AivFBOri_R_" + std::to_string(index) + env + ".pcd", *pRight);
    mbIsAivFBSaved = true;

    if (!isSafe) {
      mbIsAivFBUnSafetySaved = true;
    }
  }

  if (!isSafe) {
    MM_WARN("Aiv front/back block may be crashed, [L/R]: [%s / %s]",
        isLeftSafe ? "not crash" : "crash",
        isRightSafe ? "not crash" : "crash");
    mCheckAivCount++;
    if (mCheckAivCount > 2) {
      safetyHandler("AivFB", true, AIV_FB_CHECK_DANGER::LIDAR_AIV_FB_DANGER);
    }
  } else {
    MM_INFO("Aiv front/back block is safe");
    if (mCheckAivCount > 2) {
      safetyHandler("AivFB", false, AIV_FB_CHECK_DANGER::LIDAR_AIV_FB_DANGER);
    }
    mCheckAivCount = 0;
  }
}

bool test::checkFBSafety(PointCloudXYZIPtr pOri, bool isLeft) {
  if (!mpAivTruckDet->process(pOri)) {
    MM_WARN("Process Aiv origin pt failed");
    return false;
  }

  PointCloudXYZIPtr pPress(new PointCloudXYZI);
  PointCloudXYZIPtr pBlock(new PointCloudXYZI);
  bool bPress = mpAivTruckDet->getResult(AIV_DET::AIV_DET_PRESS, pPress);
  bool bBlock = mpAivTruckDet->getResult(AIV_DET::AIV_DET_BLOCK, pBlock);
  if (!bPress && !bBlock) {
    MM_WARN("Failed to get Aiv press or block pt");
    return false;
  }

  float curY = -100.f;
  if (bPress) {
    PointXYZI pressPt;
    pressPt.z = 100.f;
    for (auto& pt : pPress->points) {
      if (pressPt.z > pt.z) {
        pressPt = pt;
      }
    }
    MM_INFO("PressPt: [%.3f, %.3f, %.3f]", pressPt.x, pressPt.y, pressPt.z);
    curY = pressPt.y;
  } else if (bBlock) {
    PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*pBlock, minPt, maxPt);

    PointXYZI blockPt;
    blockPt.z = 100.f;
    for (auto& pt : pBlock->points) {
      if (blockPt.z > pt.z) {
        blockPt = pt;
      }
    }

    MM_INFO("BlockPt: [%.3f, %.3f, %.3f]", blockPt.x, blockPt.y, blockPt.z);
    curY =
        blockPt.y + (isLeft ? mfAivPressBlockGap : -1.f * mfAivPressBlockGap);
  }

  bool isSafe = false;
  isSafe =
      (isLeft ? (mvCheckAivSafeDis[0] < curY && mvCheckAivSafeDis[1] > curY)
              : (mvCheckAivSafeDis[2] < curY && mvCheckAivSafeDis[3] > curY));
  return isSafe;
}

void AIV_CNTR_LOCK::Calculate() {
  for (auto iter : vLock) {
    if (iter->empty()) {
      return;
    }
  }

  // Center X value: Use most corner four points' average value
  // Center Y value: Use minPt/maxPt's average value
  // Center Z value: Use the maxPt
  for (auto iter : vLock) {
    Eigen::Vector3f center;
    PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*iter, minPt, maxPt);
    center[1] = (minPt.y + maxPt.y) / 2.0;
    center[2] = maxPt.z;

    // find four cornerest points
    Eigen::Vector3f conerestPt(Eigen::Vector3f::Zero());
    PointCloudXYZIPtr cloud(new PointCloudXYZI);
    pcl::copyPointCloud(*iter, *cloud);
    for (auto pt : cloud->points) {
      pt.x = 0;
    }

    std::vector<PointXYZI> vCorner;
    PointXYZI corner;
    corner.x = 0;
    corner.y = minPt.y;
    corner.z = minPt.z;
    vCorner.push_back(corner);
    corner.y = minPt.y;
    corner.z = maxPt.z;
    vCorner.push_back(corner);
    corner.y = maxPt.x;
    corner.z = minPt.z;
    vCorner.push_back(corner);
    corner.y = maxPt.x;
    corner.z = maxPt.z;
    vCorner.push_back(corner);

    pcl::KdTreeFLANN<PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> indices;
    std::vector<float> distances;

    for (auto pt : vCorner) {
      kdtree.nearestKSearch(pt, 1, indices, distances);
      conerestPt = conerestPt + Eigen::Vector3f(iter->points.at(indices[0]).x,
                                    iter->points.at(indices[0]).y,
                                    iter->points.at(indices[0]).z);
      indices.clear();
      distances.clear();
    }

    conerestPt = conerestPt / 4.0;
    center[0]  = conerestPt[0];
    vLockCenter.push_back(center);
  }

  if (vLock.size() > 1) {
    lockLockDis    = vLockCenter[0] - vLockCenter[1];
    lockLockDis[0] = std::fabs(lockLockDis[0]);
    lockLockDis[1] = std::fabs(lockLockDis[1]);
    lockLockDis[2] = std::fabs(lockLockDis[2]);
  }
}

void test::landCollisionCheck() {
  bool is40FTSpreader(true);
  mpPLCProcess->isSpreader40FT(is40FTSpreader);

  int leftBlock   = is40FTSpreader ? AIV_LEFT_LONG : AIV_LEFT_SHORT;
  int rightBlock  = is40FTSpreader ? AIV_RIGHT_LONG : AIV_RIGHT_SHORT;
  bool isLeftSafe = false, isRightSafe = false;
  isLeftSafe  = checkLoadSafety(mvLidarData[leftBlock], true, leftBlock);
  isRightSafe = checkLoadSafety(mvLidarData[rightBlock], false, rightBlock);

  bool isSafe = (is40FTSpreader ? (isLeftSafe & isRightSafe)
                                : (isLeftSafe | isRightSafe));

  if (!isSafe) {
    MM_WARN("Aiv landing collision, [L/R]: [%s / %s]",
        isLeftSafe ? "not crash" : "crash",
        isRightSafe ? "not crash" : "crash");
  }
}

bool test::checkLoadSafety(
    PointCloudXYZIPtr pOri, bool isLeft, int index) {
  if (!mpAivTruckDet->process(pOri)) {
    MM_WARN("Process Aiv press failed");
    return false;
  }

  PointCloudXYZIPtr pTruck(new PointCloudXYZI);
  PointCloudXYZIPtr pCntr(new PointCloudXYZI);
  if (!mpAivTruckDet->getResult(AIV_DET::AIV_DET_TRUCK, pTruck)) {
    MM_WARN("Failed to get Aiv truck pt");
    return false;
  }
  if (!mpAivTruckDet->getResult(AIV_DET::AIV_DET_CNTR, pCntr)) {
    MM_WARN("Failed to get Aiv container pt");
    return false;
  }

  bool isSideDet = false;
  PointCloudXYZIPtr pSideBlock(new PointCloudXYZI);
  isSideDet = mpAivTruckDet->getResult(AIV_DET::AIV_DET_SIDEBLOCK, pSideBlock);
  // sideBlock width = 15cm

  AIV_CNTR_LOCK cntrLock;
  cntrLock.id = index;
  cntrLock.str =
      (isLeft ? "left_" : "right_") + std::to_string(mvFrameIndex[index]);
  pcl::copyPointCloud(*pCntr, *cntrLock.raw);

  Eigen::Vector4f lockRoiMin{mvLoadAivLockRoiMin[0], mvLoadAivLockRoiMin[1],
      mvLoadAivLockRoiMin[2], 1.f};
  Eigen::Vector4f lockRoiMax{mvLoadAivLockRoiMax[0], mvLoadAivLockRoiMax[1],
      mvLoadAivLockRoiMax[2], 1.f};
  std::pair<float, float> depthRange =
      std::make_pair(mvLockDepthRoi[0], mvLockDepthRoi[1]);
  std::pair<float, float> widthRange =
      std::make_pair(mvLockWidthRoi[0], mvLockWidthRoi[1]);
  std::pair<float, float> heightRange =
      std::make_pair(mvLockHeightRoi[0], mvLockHeightRoi[1]);
  DepthImgParam depthImgParam(
      lockRoiMin, lockRoiMax, depthRange, heightRange, widthRange, mfLockRes);
  mpCntrLockHoleDet->setDepthImgParam(depthImgParam);
  detCntrLock(cntrLock);

  bool isSafe = false;
  if (cntrLock.done) {
    Eigen::Vector3f lockPos = cntrLock.vLockCenter[0];
    if (isSideDet) {
      PointXYZI minPt, maxPt;
      pcl::getMinMax3D(*pSideBlock, minPt, maxPt);
      float sideBlockX = minPt.x;
      isSafe           = (lockPos[0] - sideBlockX) > 0.2f ? true : false;
    }
    // TODO: check container and truck relative position
    isSafe = true;
  }

  if (!isSafe && !mbIsAivLandUnSafetySaved) {
    mbIsAivLandUnSafetySaved = true;
    mbIsAivLandSaved         = false;
  }

  if (!mbIsAivLandSaved) {
    std::string pos = isLeft ? "_L_" : "_R_";
    pcl_utils::savePointCloud(mDebugDir,
        "AivLandOri" + pos + std::to_string(mvFrameIndex[index]) + ".pcd",
        *pOri);

    if (!isLeft) {
      mbIsAivLandSaved = true;
    }
  }

  return isSafe;
}

void test::safetyHandler(
    std::string scene, bool isDanger, int source) {
  MM_CRANE_SAFETY safetyAllow;
  safetyAllow.serName = getNodeName() + "_" + scene;
  if (isDanger) {
    safetyAllow.actionAllow &= ~AL_SPR_DOWN;
    mpPLCProcess->feedbackIPCErrCode(SEC_AIV_FB_LIDAR_STOP);
  } else {
    safetyAllow.actionAllow |= AL_SPR_DOWN;
    mpPLCProcess->feedbackBitErrCode(SEC_AIV_FB_LIDAR_STOP, false);
  }

  NOTIFY_CLIENTS(onCraneSafetyChanged, safetyAllow);
}

void test::onDetectionReady(const MMMessage& msg) {
  auto msgData = msg.data;
  std::string source;
  uint32_t index;
  MM_POSITION pos;
  MM_ORIENTATION ori;
  uint8_t valid;
  deserializeFromMessage(msgData, source, index, pos, ori, valid);

  mAivGuideBuffer.push_back(pos.y);
  if (std::fabs(mAivGuideBuffer.front() - mAivGuideBuffer.back()) < mfAivMoveThres) {
    mbAivFBReady = true;
  } else {
    mbAivFBReady = false;
  }
}

}  // namespace mm
