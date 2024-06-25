/**
 * This file is part of se2clam.
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
 */
#ifndef ODOSLAM_CPP
#define ODOSLAM_CPP
#include "OdoSLAM.h"
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

#endif // ODOSLAM_CPP

namespace se2clam {
using namespace std;
using namespace cv;

std::mutex mMutexFinish;

OdoSLAM::~OdoSLAM(){
    delete mpMapPub;
    delete mpLocalizer;
    delete mpTrack;
    delete mpLocalMapper;
    delete mpGlobalMapper;
    delete mpMap;
    delete mpMapStorage;
    delete mpFramePub;
    delete mpSensors;
}

OdoSLAM::OdoSLAM(){

}

void OdoSLAM::setVocFileBin(const char *strVoc){
    cerr << "\n###\n"
         << "###  se2clam: SE(2)-Constrained Visual Odometric Localization and Mapping\n"
         << "###\n" << endl;

    //Init ORB BoW
    cerr << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    string strVocFile = strVoc;
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        return;
    }
    cerr << "Vocabulary loaded!" << endl << endl;
}

void OdoSLAM::setDataPath(const char *strDataPath){

#ifndef USING_FISHEYE
    Config::readConfig(strDataPath, false);

    float fx = Config::Kcam.at<float>(0,0);
    float fy = Config::Kcam.at<float>(1,1);
    float cx = Config::Kcam.at<float>(0,2);
    float cy = Config::Kcam.at<float>(1,2);
    LOG(INFO) << "intrinsic is:" << fx <<"," << fy << "," << cx <<"," << cy;

    float k1 = Config::Dcam.at<float>(0,0);
    float k2 = Config::Dcam.at<float>(0,1);
    float p1 = Config::Dcam.at<float>(0,2);
    float p2 = Config::Dcam.at<float>(0,3);
    float k3 = Config::Dcam.at<float>(0,4);

    camera_ = std::make_shared<vslam::Pinhole>(fx, fy, cx, cy,
                                               k1, k2, k3, p1, p2);
#else

    Config::readConfig(strDataPath, true);

    float fx = Config::Kcam.at<float>(0,0);
    float fy = Config::Kcam.at<float>(1,1);
    float cx = Config::Kcam.at<float>(0,2);
    float cy = Config::Kcam.at<float>(1,2);
    LOG(INFO) << "intrinsic is:" << fx <<"," << fy << "," << cx <<"," << cy;

    float k1 = Config::Dcam.at<float>(0,0);
    float k2 = Config::Dcam.at<float>(0,1);
    float p1 = Config::Dcam.at<float>(0,2);
    float p2 = Config::Dcam.at<float>(0,3);
    float k3 = Config::Dcam.at<float>(0,4);
    LOG(INFO) << "distort is:" << k1 <<"," << k2 << "," << k3 <<"," << p1 << "," << p2;

    float xi = Config::xi;
    camera_ = std::make_shared<vslam::CataCamera>(xi, fx, fy, cx, cy,
                                                  k1, k2, k3, p1, p2);
#endif

}

cv::Mat OdoSLAM::getCurrentVehiclePose()
{
    return scv::inv( mpMap->getCurrentFramePose() ) * Config::cTb;
}

cv::Mat OdoSLAM::getCurrentCameraPoseWC()
{
    return scv::inv( mpMap->getCurrentFramePose() );
}

cv::Mat OdoSLAM::getCurrentCameraPoseCW()
{
    return mpMap->getCurrentFramePose();
}

void OdoSLAM::start() {

    // Construct the system
    mpMap = new Map;
    mpSensors = new Sensors;
    mpTrack = new Track;
    mpLocalMapper = new LocalMapper;
    mpGlobalMapper = new GlobalMapper;
    mpFramePub = new FramePublish(mpTrack, mpGlobalMapper);
    mpMapStorage = new MapStorage();
    mpMapPub = new MapPublish(mpMap);
    mpLocalizer = new Localizer();

    mpTrack->setLocalMapper(mpLocalMapper);
    mpTrack->setMap(mpMap);
    mpTrack->setSensors(mpSensors);
    mpTrack->setCamera(camera_);

    mpLocalMapper->setMap(mpMap);
    mpLocalMapper->setGlobalMapper(mpGlobalMapper);

    mpGlobalMapper->setMap(mpMap);
    mpGlobalMapper->setLocalMapper(mpLocalMapper);
    mpGlobalMapper->setORBVoc(mpVocabulary);

    mpMapStorage->setMap(mpMap);

    mpMapPub->setFramePub(mpFramePub);

    mpLocalizer->setMap(mpMap);
    mpLocalizer->setORBVoc(mpVocabulary);
    mpLocalizer->setSensors(mpSensors);
    mpLocalizer->setCamera(camera_);

    mpFramePub->setLocalizer(mpLocalizer);
    mpMapPub->setLocalizer(mpLocalizer);


    if (Config::USE_PREV_MAP){
        mpMapStorage->setFilePath(Config::READ_MAP_FILE_PATH, Config::READ_MAP_FILE_NAME);
        mpMapStorage->loadMap();
    }

    mbFinishRequested = false;
    mbFinished = false;

    if (se2clam::Config::LOCALIZATION_ONLY) {

        thread threadLocalizer(&se2clam::Localizer::run, mpLocalizer);

        mpFramePub->mbIsLocalize = true;
        mpMapPub->mbIsLocalize = true;

        thread threadMapPub(&se2clam::MapPublish::run, mpMapPub);

        threadLocalizer.detach();
        threadMapPub.detach();

    }
    // SLAM case
    else {

        cout << "Running SLAM" << endl;

        mpMapPub->mbIsLocalize = false;
        mpFramePub->mbIsLocalize = false;


        thread threadTracker(&se2clam::Track::run, mpTrack);
        thread threadLocalMapper(&se2clam::LocalMapper::run, mpLocalMapper);
        thread threadGlobalMapper(&se2clam::GlobalMapper::run, mpGlobalMapper);
        thread threadMapPub(&se2clam::MapPublish::run, mpMapPub);

        threadTracker.detach();
        threadLocalMapper.detach();
        threadGlobalMapper.detach();
        threadMapPub.detach();

    }

    thread threadWait(&wait, this);
    threadWait.detach();

}

void OdoSLAM::wait(OdoSLAM* system){

    ros::Rate rate(Config::FPS * 10);
    cv::Mat empty(100, 640, CV_8U, cv::Scalar(0));

    //cv::namedWindow("Press q on this window to exit...");
    while (1) {
        if (system->checkFinish()) {

            system->sendRequestFinish();

            break;
        }
        //cv::imshow("Press q on this window to exit...", empty);
        if(cv::waitKey(5) == 'q'){
            system->requestFinish();
        }
        rate.sleep();
    }
    //cv::destroyAllWindows();

    system->saveMap();

    system->checkAllExit();

    system->clear();

    system->mbFinished = true;

    cerr << "System is cleared .." << endl;

}

void OdoSLAM::saveMap() {
    if (se2clam::Config::SAVE_NEW_MAP){
        mpMapStorage->setFilePath(se2clam::Config::WRITE_MAP_FILE_PATH, se2clam::Config::WRITE_MAP_FILE_NAME);
        printf("&& DBG MS: Begin save map.\n");
        mpMapStorage->saveMap();
    }

    // Save keyframe trajectory
    cerr << "\n# Finished. Saving keyframe trajectory ..." << endl;
    ofstream towrite(se2clam::Config::WRITE_MAP_FILE_PATH  + "/se2clam_kf_trajectory.txt");
    vector<se2clam::PtrKeyFrame> vct = mpMap->getAllKF();
    for (size_t i = 0; i<vct.size(); i++){
        if (!vct[i]->isNull()){
            Mat wTb = scv::inv(se2clam::Config::bTc * vct[i]->getPose());
            Mat wRb = wTb.rowRange(0, 3).colRange(0, 3);
            g2o::Vector3D euler = g2o::internal::toEuler(se2clam::toMatrix3d(wRb));
            towrite << vct[i]->id << " " <<
                       wTb.at<float>(0, 3) << " " <<
                       wTb.at<float>(1, 3) << " " <<
                       wTb.at<float>(2, 3) << " " <<
                       euler(2) << endl;
        }
    }
}

void OdoSLAM::requestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool OdoSLAM::checkFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    if(se2clam::Config::LOCALIZATION_ONLY){
        if(mpLocalizer->isFinished() || mpMapPub->isFinished()){
            mbFinishRequested = true;
            return true;
        }
    } else {
        if(mpTrack->isFinished() || mpLocalMapper->isFinished() ||
                mpGlobalMapper->isFinished() || mpMapPub->isFinished()) {
            mbFinishRequested = true;
            return true;
        }
    }

    return mbFinishRequested;
}

void OdoSLAM::sendRequestFinish(){
    if (Config::LOCALIZATION_ONLY) {
        mpLocalizer->requestFinish();
        mpMapPub->RequestFinish();
    }
    else {
        mpTrack->requestFinish();
        mpLocalMapper->requestFinish();
        mpGlobalMapper->requestFinish();
        mpMapPub->RequestFinish();
    }
}

void OdoSLAM::checkAllExit() {
    if (Config::LOCALIZATION_ONLY) {
        while (1) {
            if (mpLocalizer->isFinished() && mpMapPub->isFinished())
                break;
            else
                std::this_thread::sleep_for(std::chrono::microseconds(2));
        }
    }
    else {
        while (1) {
            if (mpTrack->isFinished() && mpLocalMapper->isFinished() &&
                    mpGlobalMapper->isFinished() && mpMapPub->isFinished()) {
                break;
            }
            else {
                std::this_thread::sleep_for(std::chrono::microseconds(2));
            }
        }
    }
}

void OdoSLAM::clear() {

}

void OdoSLAM::waitForFinish(){
    while (1) {
        if (mbFinished) {
            break;
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(2));
        }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(20));
    cerr << "wait for finish finished..." << endl;
}

bool OdoSLAM::ok(){
    unique_lock<mutex> lock(mMutexFinish);
    return !mbFinishRequested;
}

} // namespace se2clam
