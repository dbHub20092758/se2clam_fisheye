/**
 * This file is part of se2clam.
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
 */
#include "Track.h"
#include <ros/ros.h>
#include "Map.h"
#include "LocalMapper.h"
#include "sugarCV.h"
#include "converter.h"
#include "optimizer.h"
#include "ORBmatcher.h"

#include <glog/logging.h>


namespace se2clam {
using namespace std;
using namespace cv;

typedef lock_guard<mutex> locker;

bool Track::mbUseOdometry = true;

Track::Track()
{
    mLocalMPs = vector<Point3f>(Config::MaxFtrNumber, Point3f(-1,-1,-1));
    nMinFrames = 8;
    nMaxFrames = Config::FPS;
    mnGoodPrl = 0;
    //mbTriangulated = false;

    mpORBextractor = new ORBextractor(Config::MaxFtrNumber,Config::ScaleFactor,Config::MaxLevel);
    mMatchIdx.clear();
    mvbGoodPrl.clear();

    mbFinished = false;
    mbFinishRequested = false;
}

Track::~Track(){}

void Track::setMap(Map *pMap){
    mpMap = pMap;
}

void Track::setLocalMapper(LocalMapper *pLocalMapper){
    mpLocalMapper = pLocalMapper;
}

void Track::setSensors(Sensors* pSensors) {
    mpSensors = pSensors;
}

void Track::setCamera(vslam::CameraPtr camera) {
    camera_ = camera;
}

void Track::run(){

    if(Config::LOCALIZATION_ONLY)
        return;

//    string fullOdoName = Config::DataPath + "/odo_raw.txt";
//    ifstream rec(fullOdoName);
//    string line;

//    int i = 0;
//    float x, y, theta;

    ros::Rate rate(Config::FPS*5);

    while(ros::ok()){

//        string fullImgName = Config::DataPath + "/image/" + to_string(i) + ".bmp";
//        Mat img = imread(fullImgName, cv::IMREAD_GRAYSCALE);
//        getline(rec, line);
//        istringstream iss(line);
//        iss >> x >> y >> theta;

//        Se2 odo(x,y,theta);

        cv::Mat img;
        Se2 odo;

        WorkTimer timer;
        timer.start();

        bool sensorUpdated = mpSensors->update();
        Point3f odo_3f;

        if(sensorUpdated) {
            mpSensors->readData(odo_3f, img);
            odo = Se2(odo_3f.x, odo_3f.y, odo_3f.z);
            {
                locker lock(mMutexForPub);
                bool noFrame = !(Frame::nextId);
                if(noFrame)
                    mCreateFrame(img, odo);
                else
                    mTrack(img, odo);
            }
            mpMap->setCurrentFramePose(mFrame.Tcw);
        }


        timer.stop();
        //cerr << "Tracking consuming time: " << timer.time << " ms" << endl;

        if(checkFinish())
            break;


        rate.sleep();
    }

    cerr << "Exiting tracking .." << endl;

    setFinish();
}

void Track::mCreateFrame(const Mat &img, const Se2& odo){


    mFrame = Frame(img, odo, mpORBextractor, Config::Kcam, Config::Dcam, camera_);

    mFrame.Tcw = Config::cTb.clone();

    if(mFrame.keyPoints.size() > 100){
        printf("-- INFO TR: Create first frame with %d features.\n", mFrame.N);
        mpKF = make_shared<KeyFrame>(mFrame);
        mpMap->insertKF(mpKF);
        resetLocalTrack();
    }else
        Frame::nextId = 0;
}



void Track::mTrack(const Mat &img, const Se2& odo){

    WorkTimer timer;
    timer.start();

    mFrame = Frame(img, odo, mpORBextractor, Config::Kcam, Config::Dcam, camera_);

    ///上一帧和当前帧进行匹配
    ORBmatcher matcher(0.9);
    int nMatched = matcher.MatchByWindow(mRefFrame, mFrame, mPrevMatched, 20, mMatchIdx);
//    LOG(INFO) << "MatchByWindow nMatched is :" << nMatched;

    //TODO(DUAN) : 坐标点转换到归一化平面
#ifndef  USING_NORM_PLANE
    nMatched = removeOutliers(mRefFrame.keyPointsUn, mFrame.keyPointsUn, mMatchIdx);
#else
    nMatched = removeOutliers(mRefFrame.keyPointsNormPlane, mFrame.keyPointsNormPlane, mMatchIdx);
#endif
//    LOG(INFO) << "removeOutliers nMatched is :" << nMatched;

    ///采用里程计位姿进行位姿更新
    updateFramePose();

    // Check parallax and do triangulation
    ///当前帧和上一关键帧进行三角化匹配
    int nTrackedOld = doTriangulate();

    // Need new KeyFrame decision
    if( needNewKF(nTrackedOld, nMatched) ) {

        // Insert KeyFrame
        ///增加关键帧时才产生地图点
        mpKF = make_shared<KeyFrame>(mFrame);
        mpLocalMapper->addNewKF(mpKF, mLocalMPs, mMatchIdx, mvbGoodPrl);

        resetLocalTrack();

        printf("-- INFO TR: Add new KF at frame %d\n", mFrame.id);
    }
    LOG(INFO) << "pose is:" << mFrame.Tcw;

    timer.stop();
}

void Track::updateFramePose(){
    Se2 dOdo = mpKF->mOdo - mFrame.mOdo;
    mFrame.Tcr = Config::cTb * Se2(dOdo.x, dOdo.y, dOdo.theta).toCvSE3() * Config::bTc;
    mFrame.Tcw = mFrame.Tcr * mpKF->Tcw;
}


void Track::resetLocalTrack(){
    mFrame.Tcr = cv::Mat::eye(4,4,CV_32FC1);
    KeyPoint::convert(mFrame.keyPoints, mPrevMatched);    ///上一帧关键帧特征点
    mRefFrame = mFrame;     ///参考帧为上一帧关键帧
    mLocalMPs = mpKF->mViewMPs;     ///局部地图点为关键帧可以看到的点
    mnGoodPrl = 0;
    mMatchIdx.clear();
}



int Track::copyForPub(vector<KeyPoint>& kp1, vector<KeyPoint>& kp2,
                      Mat& img1, Mat& img2,
                      vector<int>& vMatches12) {

    locker lock(mMutexForPub);
    mRefFrame.copyImgTo(img1);
    mFrame.copyImgTo(img2);

    kp1 = mRefFrame.keyPoints;
    kp2 = mFrame.keyPoints;
    vMatches12 = mMatchIdx;

    return !mMatchIdx.empty();
}


void Track::calcOdoConstraintCam(const Se2 &dOdo, Mat& cTc, g2o::Matrix6d &Info_se3){

    const Mat bTc = Config::bTc;
    const Mat cTb = Config::cTb;

    const Mat bTb = Se2(dOdo.x, dOdo.y, dOdo.theta).toCvSE3();

    cTc = cTb * bTb * bTc;

    float dx = dOdo.x * Config::ODO_X_UNCERTAIN + Config::ODO_X_NOISE;
    float dy = dOdo.y * Config::ODO_Y_UNCERTAIN + Config::ODO_Y_NOISE;
    float dtheta = dOdo.theta * Config::ODO_T_UNCERTAIN + Config::ODO_T_NOISE;

    g2o::Matrix6d Info_se3_bTb = g2o::Matrix6d::Zero();
    //    float data[6] = { 1.f/(dx*dx), 1.f/(dy*dy), 1, 1e4, 1e4, 1.f/(dtheta*dtheta) };
    float data[6] = { 1.f/(dx*dx), 1.f/(dy*dy), 1e-4, 1e-4, 1e-4, 1.f/(dtheta*dtheta) };
    for(int i = 0; i < 6; i++)
        Info_se3_bTb(i,i) = data[i];
    Info_se3 = Info_se3_bTb;


//    g2o::Matrix6d J_bTb_cTc = toSE3Quat(bTc).adj();
//    J_bTb_cTc.block(0,3,3,3) = J_bTb_cTc.block(3,0,3,3);
//    J_bTb_cTc.block(3,0,3,3) = g2o::Matrix3D::Zero();

//    Info_se3 = J_bTb_cTc.transpose() * Info_se3_bTb * J_bTb_cTc;

//    for(int i = 0; i < 6; i++)
//        for(int j = 0; j < i; j++)
//            Info_se3(i,j) = Info_se3(j,i);

    //assert(verifyInfo(Info_se3));

}

void Track::calcSE3toXYZInfo(Point3f xyz1, const Mat &Tcw1, const Mat &Tcw2, Eigen::Matrix3d &info1, Eigen::Matrix3d &info2){

    Point3f O1 = Point3f(scv::inv(Tcw1).rowRange(0,3).col(3));
    Point3f O2 = Point3f(scv::inv(Tcw2).rowRange(0,3).col(3));
    Point3f xyz = scv::se3map(scv::inv(Tcw1), xyz1);
    Point3f vO1 = xyz - O1;
    Point3f vO2 = xyz - O2;
    float sinParallax = cv::norm(vO1.cross(vO2)) / ( cv::norm(vO1) * cv::norm(vO2) );

    Point3f xyz2 = scv::se3map(Tcw2, xyz);
    float length1 = cv::norm(xyz1);
    float length2 = cv::norm(xyz2);
    float dxy1 = 2.f * length1 / Config::fxCam;
    float dxy2 = 2.f * length2 / Config::fxCam;
    float dz1 = dxy2 / sinParallax;
    float dz2 = dxy1 / sinParallax;

    Mat info_xyz1 = (Mat_<float>(3,3) <<
                     1.f/(dxy1*dxy1), 0,               0,
                     0,               1.f/(dxy1*dxy1), 0,
                     0,               0,               1.f/(dz1*dz1));

    Mat info_xyz2 = (Mat_<float>(3,3) <<
                     1.f/(dxy2*dxy2), 0,               0,
                     0,               1.f/(dxy2*dxy2), 0,
                     0,               0,               1.f/(dz2*dz2));

    Point3f z1 = Point3f(0, 0, length1);
    Point3f z2 = Point3f(0, 0, length2);
    Point3f k1 = xyz1.cross(z1);
    float normk1 = cv::norm(k1);
    float sin1 = normk1/( cv::norm(z1) * cv::norm(xyz1) );
    k1 = k1 * (std::asin(sin1) / normk1);
    Point3f k2 = xyz2.cross(z2);
    float normk2 = cv::norm(k2);
    float sin2 = normk2/( cv::norm(z2) * cv::norm(xyz2) );
    k2 = k2 * (std::asin(sin2) / normk2);

    Mat R1, R2;
    Mat k1mat = (Mat_<float>(3,1) << k1.x, k1.y, k1.z);
    Mat k2mat = (Mat_<float>(3,1) << k2.x, k2.y, k2.z);
    cv::Rodrigues(k1mat, R1);
    cv::Rodrigues(k2mat, R2);

    info1 = toMatrix3d( R1.t() * info_xyz1 * R1 );
    info2 = toMatrix3d( R2.t() * info_xyz2 * R2 );

}

int Track::removeOutliers(const vector<KeyPoint> &kp1, const vector<KeyPoint> &kp2, vector<int> &matches){
    vector<Point2f> pt1, pt2;
    vector<int> idx;
    pt1.reserve(kp1.size());
    pt2.reserve(kp2.size());
    idx.reserve(kp1.size());

    for(int i = 0, iend = kp1.size(); i < iend; i++){
        if(matches[i] < 0)
            continue;
        idx.push_back(i);
        pt1.push_back(kp1[i].pt);
        pt2.push_back(kp2[matches[i]].pt);
    }

    vector<unsigned char> mask;

    if(pt1.size() != 0)
        findFundamentalMat(pt1, pt2, mask);

    int nInlier = 0;
    for(int i = 0, iend = mask.size(); i < iend; i++){
        if(!mask[i])
            matches[idx[i]] = -1;
        else
            nInlier++;
    }

    // If too few match inlier, discard all matches. The enviroment might not be suitable for image tracking.
    if(nInlier < 10) {
        nInlier = 0;
        std::fill(mMatchIdx.begin(), mMatchIdx.end(), -1);
    }

    return nInlier;

}

int Track::removeOutliers(const std::vector<cv::Point2f> &kp1, const std::vector<cv::Point2f> &kp2,
                          std::vector<int> &matches) {

    vector<Point2f> pt1, pt2;
    vector<int> idx;
    pt1.reserve(kp1.size());
    pt2.reserve(kp2.size());
    idx.reserve(kp1.size());

    for(int i = 0, iend = kp1.size(); i < iend; i++){
        if(matches[i] < 0)
            continue;
        idx.push_back(i);
        pt1.push_back(kp1[i]);
        pt2.push_back(kp2[matches[i]]);
    }

    vector<unsigned char> mask;

    if(pt1.size() != 0)
        findFundamentalMat(pt1, pt2, mask);

    int nInlier = 0;
    for(int i = 0, iend = mask.size(); i < iend; i++){
        if(!mask[i])
            matches[idx[i]] = -1;
        else
            nInlier++;
    }

    // If too few match inlier, discard all matches. The enviroment might not be suitable for image tracking.
    if(nInlier < 10) {
        nInlier = 0;
        std::fill(mMatchIdx.begin(), mMatchIdx.end(), -1);
    }

    return nInlier;

}

bool Track::needNewKF(int nTrackedOldMP, int nMatched){
    int nOldKP = mpKF->getSizeObsMP();     ///关键帧观测到的地图点数
    bool c0 = mFrame.id - mpKF->id > nMinFrames;
    bool c1 = (float)nTrackedOldMP <= (float)nOldKP * 0.5f;
    bool c2 = mnGoodPrl > 40;              ///视差满足要求的点数大于40个
    bool c3 = mFrame.id - mpKF->id > nMaxFrames;
    bool c4 = nMatched < 0.1f * Config::MaxFtrNumber || nMatched < 20;
    bool bNeedNewKF = c0 && ( (c1 && c2) || c3 || c4 );

    ///移动位姿约束，判断关键帧；角度和位移
    bool bNeedKFByOdo = true;
    if(mbUseOdometry){
        Se2 dOdo = mFrame.mOdo - mpKF->mOdo;
        bool c5 = dOdo.theta >= 0.0349f; // Larger than 2 degree
        //cv::Mat cTc = Config::cTb * toT4x4(dOdo.x, dOdo.y, dOdo.theta) * Config::bTc;
        cv::Mat cTc = Config::cTb * Se2(dOdo.x, dOdo.y, dOdo.theta).toCvSE3() * Config::bTc;
        cv::Mat xy = cTc.rowRange(0,2).col(3);
        bool c6 = cv::norm(xy) >= ( 0.0523f * Config::UPPER_DEPTH * 0.1f ); // 3 degree = 0.0523 rad

        bNeedKFByOdo = c5 || c6;
    }
    bNeedNewKF = bNeedNewKF && bNeedKFByOdo;

    ///局部建图线程能接收关键帧？？？？
    if( mpLocalMapper->acceptNewKF() ) {
        return bNeedNewKF;
    }
    else if ( c0 && (c4 || c3) && bNeedKFByOdo) {
        mpLocalMapper->setAbortBA();
    }

    return false;
}

int Track::doTriangulate(){

    ///当前帧和关键之间的间隔太小，不进行三角化
    if(mFrame.id - mpKF->id < nMinFrames){
        return 0;
    }

    Mat TfromRefKF = scv::inv(mFrame.Tcr);
    Point3f Ocam = Point3f(TfromRefKF.rowRange(0,3).col(3));   ///相机原点坐标
    int nTrackedOld = 0;
    mvbGoodPrl = vector<bool>(mRefFrame.N, false);
    mnGoodPrl = 0;

    for(int i = 0; i < mRefFrame.N; i++){

        if(mMatchIdx[i] < 0)
            continue;

        if(mpKF->hasObservation(i)){
            mLocalMPs[i] = mpKF->mViewMPs[i];
            nTrackedOld++;
            continue;
        }

        //TODO(DUAN): 转换到归一化平面
#ifndef USING_NORM_PLANE
        Point2f pt_KF = mpKF->keyPointsUn[i].pt;
        Point2f pt = mFrame.keyPointsUn[mMatchIdx[i]].pt;
        cv::Mat P_KF = Config::PrjMtrxEye;
        cv::Mat P = Config::Kcam * mFrame.Tcr.rowRange(0,3);
#else
        Point2f pt_KF = mpKF->keyPointsNormPlane[i];
        Point2f pt = mFrame.keyPointsNormPlane[mMatchIdx[i]];
        cv::Mat P_KF = cv::Mat::eye(3,4,CV_32FC1);
        cv::Mat P = mFrame.Tcr.rowRange(0,3);
#endif

        Point3f pos = scv::triangulate(pt_KF, pt, P_KF, P);

        //TODO（DUAN）：z值检测
        if( Config::acceptDepth(pos.z) ) {
            mLocalMPs[i] = pos;
            if( scv::checkParallax(Point3f(0,0,0), Ocam, pos, 2) ) {
                mnGoodPrl++;
                mvbGoodPrl[i] = true;
            }
        }
        else {
            mMatchIdx[i] = -1;
        }
    }

    return nTrackedOld;
}

void Track::requestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Track::checkFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

bool Track::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Track::setFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

}// namespace se2clam
