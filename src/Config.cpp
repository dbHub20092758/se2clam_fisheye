/**
 * This file is part of se2clam.
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
 */
#include "Config.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>
#include <iomanip>

namespace se2clam{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace std;

    std::string Config::DataPath;
    int Config::ImgIndex;
    int Config::ImgIndexLocalSt;
    cv::Size Config::ImgSize;
    cv::Mat Config::bTc; // camera extrinsic
    cv::Mat Config::cTb; // inv of bTc
    cv::Mat Config::Kcam; // camera intrinsic
    float Config::fxCam;
    float Config::fyCam;
    cv::Mat Config::Dcam; // camera distortion
    float Config::xi;

    float Config::UPPER_DEPTH;
    float Config::LOWER_DEPTH;

    int Config::NUM_FILTER_LAST_SEVERAL_MU;
    int Config::FILTER_CONVERGE_CONTINUE_COUNT;
    float Config::DEPTH_FILTER_THRESHOLD;

    float Config::ScaleFactor; // scalefactor in detecting features
    int Config::MaxLevel; // level number of pyramid in detecting features
    int Config::MaxFtrNumber; // max feature number to detect
    float Config::FEATURE_SIGMA;

    float Config::ODO_X_UNCERTAIN, Config::ODO_Y_UNCERTAIN, Config::ODO_T_UNCERTAIN;
    float Config::ODO_X_NOISE, Config::ODO_Y_NOISE, Config::ODO_T_NOISE;

    float Config::PLANEMOTION_XROT_INFO = 1e6;
    float Config::PLANEMOTION_YROT_INFO = 1e6;
    float Config::PLANEMOTION_Z_INFO = 1;

    int Config::LOCAL_FRAMES_NUM;
    float Config::TH_HUBER;
    int Config::LOCAL_ITER;
    bool Config::LOCAL_VERBOSE = false;
    int Config::GLOBAL_ITER = 15;
    bool Config::GLOBAL_VERBOSE = false;
    bool Config::LOCAL_PRINT = false;
    bool Config::GLOBAL_PRINT = false;

    int Config::FPS;

    bool Config::USE_PREV_MAP = false;
    bool Config::LOCALIZATION_ONLY = false;
    bool Config::SAVE_NEW_MAP = false;
    std::string Config::READ_MAP_FILE_NAME;
    std::string Config::READ_MAP_FILE_PATH;
    std::string Config::WRITE_MAP_FILE_NAME = "se2clam.map";
    std::string Config::WRITE_MAP_FILE_PATH = "/home/se2clam/";

    std::string Config::WRITE_TRAJ_FILE_NAME;
    std::string Config::WRITE_TRAJ_FILE_PATH;

    cv::Mat Config::PrjMtrxEye;

//    int Config::MAPPUB_SCALE_RATIO = 300;
    int Config::MAPPUB_SCALE_RATIO = 1;

    int Config::GM_VCL_NUM_MIN_MATCH_MP = 15;
    int Config::GM_VCL_NUM_MIN_MATCH_KP = 30;
    double Config::GM_VCL_RATIO_MIN_MATCH_MP = 0.05;

    int Config::GM_DCL_MIN_KFID_OFFSET = 20;
    double Config::GM_DCL_MIN_SCORE_BEST = 0.005;

    void Config::readConfig(const std::string &path, bool is_fisheye){

        if(!is_fisheye){
            DataPath = path;
            std::string camParaPath = path + "/config/CamConfig.yml";
            cv::FileStorage camPara(camParaPath, cv::FileStorage::READ);
            assert(camPara.isOpened());
            cv::Mat _mK, _mD, _rvec, rvec, _T, T, R;
            float height, width;
            camPara["image_height"] >> height;
            camPara["image_width"] >> width;
            camPara["camera_matrix"] >> _mK;
            camPara["distortion_coefficients"] >> _mD;
            camPara["rvec_b_c"] >> _rvec;
            camPara["tvec_b_c"] >> _T;

            _mK.convertTo(Kcam,CV_32FC1);
            _mD.convertTo(Dcam,CV_32FC1);
            _rvec.convertTo(rvec,CV_32FC1);
            _T.convertTo(T,CV_32FC1);

//            T /= 1000.0;

            fxCam = Kcam.at<float>(0,0);
            fyCam = Kcam.at<float>(1,1);
            ImgSize.height = height;
            ImgSize.width = width;
            std::cerr << "# Load camera config ..." << std::endl;
            std::cerr << "- Camera matrix: " << std::endl << " " <<
                      Kcam << std::endl <<
                      "- Camera distortion: " << std::endl << " " <<
                      Dcam << std::endl <<
                      "- Img size: " << std::endl << " " <<
                      ImgSize << std::endl << std::endl;
            // bTc: camera extrinsic
            cv::Rodrigues(rvec,R);
            bTc = cv::Mat::eye(4,4,CV_32FC1);
            R.copyTo(bTc.rowRange(0,3).colRange(0,3));
            T.copyTo(bTc.rowRange(0,3).col(3));
            cv::Mat RT = R.t();
            cv::Mat t = -RT * T;
            cTb = cv::Mat::eye(4,4,CV_32FC1);
            RT.copyTo(cTb.rowRange(0,3).colRange(0,3));
            t.copyTo(cTb.rowRange(0,3).col(3));
            LOG(INFO) << "extrinsic Tbc is :" << bTc;
            LOG(INFO) << "extrinsic Tcb is :" << cTb;

            PrjMtrxEye = Kcam * cv::Mat::eye(3,4,CV_32FC1);
            camPara.release();

            std::string settingsPath = path + "/config/Settings.yml";
            cv::FileStorage settings(settingsPath, cv::FileStorage::READ);
            assert(settings.isOpened());

            ImgIndex = (int)settings["img_num"];
            ImgIndexLocalSt = (int)settings["img_id_local_st"];
            UPPER_DEPTH = (float)settings["upper_depth"];
            LOWER_DEPTH = (float)settings["lower_depth"];
            NUM_FILTER_LAST_SEVERAL_MU = (int)settings["depth_filter_avrg_count"];
            FILTER_CONVERGE_CONTINUE_COUNT = (int)settings["depth_filter_converge_count"];
            DEPTH_FILTER_THRESHOLD = (float)settings["depth_filter_thresh"];
            ScaleFactor = (float)settings["scale_facotr"];
            MaxLevel = (int)settings["max_level"];
            MaxFtrNumber = (int)settings["max_feature_num"];
            FEATURE_SIGMA = (float)settings["feature_sigma"];

            ODO_X_UNCERTAIN = (float)settings["odo_x_uncertain"];
            ODO_Y_UNCERTAIN = (float)settings["odo_y_uncertain"];
            ODO_T_UNCERTAIN = (float)settings["odo_theta_uncertain"];
            ODO_X_NOISE = (float)settings["odo_x_steady_noise"];
            ODO_Y_NOISE = (float)settings["odo_y_steady_noise"];
            ODO_T_NOISE = (float)settings["odo_theta_steady_noise"];
            if(!settings["plane_motion_xrot_info"].empty())
                PLANEMOTION_XROT_INFO = (float)settings["plane_motion_xrot_info"];
            if(!settings["plane_motion_yrot_info"].empty())
                PLANEMOTION_YROT_INFO = (float)settings["plane_motion_yrot_info"];
            if(!settings["plane_motion_z_info"].empty())
                PLANEMOTION_Z_INFO = (float)settings["plane_motion_z_info"];
            LOCAL_FRAMES_NUM = (int)settings["frame_num"];
            TH_HUBER = sqrt((float)settings["th_huber2"]);
            LOCAL_ITER = (int)settings["local_iter"];
            LOCAL_VERBOSE = (bool)(int)(settings["local_verbose"]);
            LOCAL_PRINT = (bool)(int)(settings["local_print"]);
            if((int)settings["global_iter"]){
                GLOBAL_ITER = (int)settings["global_iter"];
            }
            GLOBAL_VERBOSE = (bool)(int)(settings["global_verbose"]);
            GLOBAL_PRINT = (bool)(int)(settings["global_print"]);
            FPS = (int)settings["fps"];

            USE_PREV_MAP = (bool)(int)(settings["use_prev_map"]);
            SAVE_NEW_MAP = (bool)(int)(settings["save_new_map"]);
            LOCALIZATION_ONLY = (bool)(int)(settings["localization_only"]);
            settings["read_map_file_name"] >> READ_MAP_FILE_NAME;
            settings["write_map_file_name"] >> WRITE_MAP_FILE_NAME;
            settings["read_map_file_path"] >> READ_MAP_FILE_PATH;
            settings["write_map_file_path"] >> WRITE_MAP_FILE_PATH;
            settings["write_traj_file_name"] >> WRITE_TRAJ_FILE_NAME;
            settings["write_traj_file_path"] >> WRITE_TRAJ_FILE_PATH;

            MAPPUB_SCALE_RATIO = (int)(settings["mappub_scale_ratio"]);

            GM_VCL_NUM_MIN_MATCH_MP = (int)(settings["gm_vcl_num_min_match_mp"]);
            GM_VCL_NUM_MIN_MATCH_KP = (int)(settings["gm_vcl_num_min_match_kp"]);
            GM_VCL_RATIO_MIN_MATCH_MP = (double)(settings["gm_vcl_ratio_min_match_kp"]);

            GM_DCL_MIN_KFID_OFFSET = (int)(settings["gm_dcl_min_kfid_offset"]);
            GM_DCL_MIN_SCORE_BEST = (double)(settings["gm_dcl_min_score_best"]);

            settings.release();
        }else{

            //鱼眼相机
            DataPath = path;
            std::string camParaPath = path + "/config/CamConfig.yml";
            cv::FileStorage camPara(camParaPath, cv::FileStorage::READ);
            assert(camPara.isOpened());
            cv::Mat _mK, _mD, _rvec, rvec, _T, T;
            float height, width;
            camPara["image_height"] >> height;
            camPara["image_width"] >> width;
            camPara["camera_matrix"] >> _mK;
            camPara["distortion_coefficients"] >> _mD;
            camPara["rvec_b_c"] >> _rvec;    //欧拉角
            camPara["tvec_b_c"] >> _T;
            camPara["xi"] >> xi;
            _mK.convertTo(Kcam,CV_32FC1);
            _mD.convertTo(Dcam,CV_32FC1);
            _rvec.convertTo(rvec,CV_32FC1);
            _T.convertTo(T,CV_32FC1);

            LOG(INFO) << "xi is:" << xi;

            //内参缩放
            Kcam.at<float>(0,0) = Kcam.at<float>(0,0) * 832 / 1280;
            Kcam.at<float>(0,2) = Kcam.at<float>(0,2) * 832 / 1280;
            Kcam.at<float>(1,1) = Kcam.at<float>(1,1) * 640 / 960;
            Kcam.at<float>(1,2) = Kcam.at<float>(1,2) * 640 / 960;

            fxCam = Kcam.at<float>(0,0);
            fyCam = Kcam.at<float>(1,1);
            ImgSize.height = height;
            ImgSize.width = width;
            std::cerr << "# Load camera config ..." << std::endl;
            std::cerr << "- Camera matrix: " << std::endl << " " <<
                      Kcam << std::endl <<
                      "- Camera distortion: " << std::endl << " " <<
                      Dcam << std::endl <<
                      "- Img size: " << std::endl << " " <<
                      ImgSize << std::endl << std::endl;

            // bTc: camera extrinsic
            float roll = rvec.at<float>(0,0) * M_PI / 180.0;
            float pitch = rvec.at<float>(0,1) * M_PI / 180.0;
            float yaw = rvec.at<float>(0,2) * M_PI / 180.0;
            LOG(INFO) << "roll pitch yaw is:" << roll <<"," << pitch << "," << yaw;
            LOG(INFO) << "extrinsic tvec is:" << T;

            Eigen::Matrix3f rotation;           //Tbc0
            rotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

            //相机坐标系进行转换，车体定义坐标转换为常用相机坐标系
            Eigen::Matrix3f rotation1;          //Tc0c1
            rotation1 << 0, 0, 1, -1 ,0, 0, 0, -1, 0;
            LOG(INFO) << std::setfill(' ') << "rotation1 is :" << rotation1;

            Eigen::Matrix3f Tbc = rotation * rotation1;

            cv::Mat rotation_cv;
            cv::eigen2cv(Tbc, rotation_cv);

            bTc = cv::Mat::eye(4,4,CV_32FC1);
            rotation_cv.copyTo(bTc.rowRange(0,3).colRange(0,3));
            T.copyTo(bTc.rowRange(0,3).col(3));
            cv::Mat RT = rotation_cv.t();
            cv::Mat t = -RT * T;
            cTb = cv::Mat::eye(4,4,CV_32FC1);
            RT.copyTo(cTb.rowRange(0,3).colRange(0,3));
            t.copyTo(cTb.rowRange(0,3).col(3));
            LOG(INFO) << "extrinsic Tbc is :" << bTc;
            LOG(INFO) << "extrinsic Tcb is :" << cTb;

            PrjMtrxEye = Kcam * cv::Mat::eye(3,4,CV_32FC1);
            camPara.release();

            std::string settingsPath = path + "/config/Settings.yml";
            cv::FileStorage settings(settingsPath, cv::FileStorage::READ);
            assert(settings.isOpened());

            ImgIndex = (int)settings["img_num"];
            ImgIndexLocalSt = (int)settings["img_id_local_st"];
            UPPER_DEPTH = (float)settings["upper_depth"];
            LOWER_DEPTH = (float)settings["lower_depth"];
            NUM_FILTER_LAST_SEVERAL_MU = (int)settings["depth_filter_avrg_count"];
            FILTER_CONVERGE_CONTINUE_COUNT = (int)settings["depth_filter_converge_count"];
            DEPTH_FILTER_THRESHOLD = (float)settings["depth_filter_thresh"];
            ScaleFactor = (float)settings["scale_facotr"];
            MaxLevel = (int)settings["max_level"];
            MaxFtrNumber = (int)settings["max_feature_num"];
            FEATURE_SIGMA = (float)settings["feature_sigma"];

            ODO_X_UNCERTAIN = (float)settings["odo_x_uncertain"];
            ODO_Y_UNCERTAIN = (float)settings["odo_y_uncertain"];
            ODO_T_UNCERTAIN = (float)settings["odo_theta_uncertain"];
            ODO_X_NOISE = (float)settings["odo_x_steady_noise"];
            ODO_Y_NOISE = (float)settings["odo_y_steady_noise"];
            ODO_T_NOISE = (float)settings["odo_theta_steady_noise"];
            if(!settings["plane_motion_xrot_info"].empty())
                PLANEMOTION_XROT_INFO = (float)settings["plane_motion_xrot_info"];
            if(!settings["plane_motion_yrot_info"].empty())
                PLANEMOTION_YROT_INFO = (float)settings["plane_motion_yrot_info"];
            if(!settings["plane_motion_z_info"].empty())
                PLANEMOTION_Z_INFO = (float)settings["plane_motion_z_info"];
            LOCAL_FRAMES_NUM = (int)settings["frame_num"];
            TH_HUBER = sqrt((float)settings["th_huber2"]);
            LOCAL_ITER = (int)settings["local_iter"];
            LOCAL_VERBOSE = (bool)(int)(settings["local_verbose"]);
            LOCAL_PRINT = (bool)(int)(settings["local_print"]);
            if((int)settings["global_iter"]){
                GLOBAL_ITER = (int)settings["global_iter"];
            }
            GLOBAL_VERBOSE = (bool)(int)(settings["global_verbose"]);
            GLOBAL_PRINT = (bool)(int)(settings["global_print"]);
            FPS = (int)settings["fps"];

            USE_PREV_MAP = (bool)(int)(settings["use_prev_map"]);
            SAVE_NEW_MAP = (bool)(int)(settings["save_new_map"]);
            LOCALIZATION_ONLY = (bool)(int)(settings["localization_only"]);
            settings["read_map_file_name"] >> READ_MAP_FILE_NAME;
            settings["write_map_file_name"] >> WRITE_MAP_FILE_NAME;
            settings["read_map_file_path"] >> READ_MAP_FILE_PATH;
            settings["write_map_file_path"] >> WRITE_MAP_FILE_PATH;
            settings["write_traj_file_name"] >> WRITE_TRAJ_FILE_NAME;
            settings["write_traj_file_path"] >> WRITE_TRAJ_FILE_PATH;

            MAPPUB_SCALE_RATIO = (int)(settings["mappub_scale_ratio"]);

            GM_VCL_NUM_MIN_MATCH_MP = (int)(settings["gm_vcl_num_min_match_mp"]);
            GM_VCL_NUM_MIN_MATCH_KP = (int)(settings["gm_vcl_num_min_match_kp"]);
            GM_VCL_RATIO_MIN_MATCH_MP = (double)(settings["gm_vcl_ratio_min_match_kp"]);

            GM_DCL_MIN_KFID_OFFSET = (int)(settings["gm_dcl_min_kfid_offset"]);
            GM_DCL_MIN_SCORE_BEST = (double)(settings["gm_dcl_min_score_best"]);

            settings.release();
        }

    }

    bool Config::acceptDepth(float depth){
        return (depth >= LOWER_DEPTH && depth <= UPPER_DEPTH);
    }


    Se2::Se2(){}
    Se2::Se2(float _x, float _y ,float _theta):
            x(_x), y(_y), theta(_theta){}
    Se2::~Se2(){}

    Se2 Se2::operator +(const Se2& toadd){
        // Note: dx and dy, which is expressed in the previous,
        // should be transformed to be expressed in the world frame
        float cost = std::cos(theta);
        float sint = std::sin(theta);
        float _x = x + toadd.x*cost - toadd.y*sint;
        float _y = y + toadd.x*sint + toadd.y*cost;
        float _theta = theta + toadd.theta;
        return Se2(_x, _y, _theta);
    }

    Se2 Se2::operator -(const Se2& tominus){
        double PI = M_PI;

        float dx = x - tominus.x;
        float dy = y - tominus.y;
        float dtheta = theta - tominus.theta;

        dtheta = dtheta - floor(dtheta/(2*PI))*2*PI;

        if (dtheta > PI) {
            dtheta -= 2*PI;
        }

        float cost = std::cos(tominus.theta);
        float sint = std::sin(tominus.theta);
        // Note: dx and dy, which is expressed in world frame,
        // should be transformed to be expressed in the previous frame
        return Se2(cost*dx+sint*dy, -sint*dx+cost*dy, dtheta);
    }

    cv::Mat Se2::toCvSE3()
    {
        float c = cos(theta);
        float s = sin(theta);

        return (cv::Mat_<float>(4,4) <<
                                     c,-s, 0, x,
                s, c, 0, y,
                0, 0, 1, 0,
                0, 0, 0, 1);
    }


}
