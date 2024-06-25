// /**
// This file is part of se2clam.
// Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
///

#include "OdoSLAM.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char **argv)
{
    //! Initialize
    ros::init(argc, argv, "test_vn");
    ros::start();

    if(argc != 3){
        cerr << "Usage: rosrun se2clam test_vn dataPath PATH_TO_ORBvoc.bin" << endl;
        ros::shutdown();
        return 1;
    }

    se2clam::OdoSLAM system;

    system.setVocFileBin(argv[2]);
    system.setDataPath(argv[1]);
    system.start();

    string fullOdoName = se2clam::Config::DataPath + "/odo_raw_accu.txt";
    ifstream rec(fullOdoName);
    float x,y,theta;
    string line;

    ros::Rate rate(se2clam::Config::FPS);

    int n = se2clam::Config::ImgIndex;
    int i = 0;

    //图像数据和odo数据已经同步
    for(; i < n && system.ok(); i++) {

        string fullImgName = se2clam::Config::DataPath + "/image/" + to_string(i) + ".bmp";
        Mat img = imread(fullImgName, cv::IMREAD_GRAYSCALE);
        std::getline(rec, line);
        istringstream iss(line);
        iss >> x >> y >> theta;

//        x /= 1000.0;
//        y /= 1000.0;

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

