
// /**
// This file is part of se2clam.
// Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
///

#include "OdoSLAM.h"
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace Eigen;

float getTimestamp(const std::string &image_name)
{

    int pos1 = image_name.find_last_of('/');
    int pos2 = image_name.find_last_of('.');

    std::string substr = image_name.substr(pos1 + 1, pos2 - pos1 - 1);
    //    LOG(INFO) << "substr is :" << substr;

    return atof(substr.c_str());
}

int main(int argc, char **argv)
{
    //! Initialize
    ros::init(argc, argv, "test_vn");
    ros::start();

    if (argc != 3)
    {
        cerr << "Usage: rosrun se2clam test_vn dataPath PATH_TO_ORBvoc.bin" << endl;
        ros::shutdown();
        return 1;
    }

    se2clam::OdoSLAM system;

    system.setVocFileBin(argv[2]);
    system.setDataPath(argv[1]);
    system.start();

    string fullOdoName = se2clam::Config::DataPath + "odo/test.txt";
    ifstream rec(fullOdoName);
    float x, y, theta;
    float timestamp;
    string line;

    ros::Rate rate(se2clam::Config::FPS);

    int n = se2clam::Config::ImgIndex;
    int i = 0;

    std::vector<cv::String> image_names;
    cv::glob(se2clam::Config::DataPath + "front", image_names, false);
    LOG(INFO) << "image size is:" << image_names.size();
    LOG(INFO) << "image size is:" << image_names[0];

    n = image_names.size();

    // 图像数据和odo数据已经同步
    for (; i < n && system.ok(); i++)
    {

        string fullImgName = image_names[i];
        Mat img = imread(fullImgName, cv::IMREAD_GRAYSCALE);

        float timestamp_img = getTimestamp(fullImgName);
        while (fabs(timestamp - timestamp_img) > 1e-6)
        {
            std::getline(rec, line);
            //            LOG(INFO) << "line is:" << line;
            std::stringstream iss(line);

            std::vector<double> datas;
            std::string data_str;
            while (std::getline(iss, data_str, ','))
            {
                double data = std::atof(data_str.c_str());
                datas.emplace_back(data);
            }

            timestamp = datas[0];
            x = datas[1];
            y = datas[2];
            theta = datas[3];

            //            LOG(INFO) << "odo pose is:" << timestamp << "," <<  x << "," << y << "," << theta;
        }
        //        LOG(INFO) << "odo pose is:" << x << "," << y << "," << theta;
        //        LOG(INFO) << "timestamp is:" << timestamp_img << "," << fullImgName;

        //        LOG(INFO) << "image id is: " << i;
        system.receiveOdoData(x, y, theta);
        system.receiveImgData(img);

        rate.sleep();
    }
    cerr << "Finish test..." << endl;

    system.requestFinish();
    system.waitForFinish();

    ros::shutdown();

    cerr << "Rec close..." << endl;
    rec.close();
    cerr << "Exit test..." << endl;
    return 0;
}
