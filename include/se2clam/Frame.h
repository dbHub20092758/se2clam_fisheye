/**
 * This file is part of se2clam.
 * Copyright (C) Fan ZHENG (github.com/izhengfan), Hengbo TANG (github.com/hbtang)
 */
#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <mutex>
#include <memory>
#include "Config.h"
#include "ORBextractor.h"
#include "camera.h"

namespace se2clam {

const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;

class KeyFrame;
class Frame
{
public:
    Frame();
    Frame(const cv::Mat &im, const Se2& odo, ORBextractor* extractor, const cv::Mat &K, const cv::Mat &distCoef,
          vslam::CameraPtr camera);

    Frame(const Frame& f);
    Frame& operator=(const Frame& f);
    ~Frame();

    float getTime() const;
    void setTime(float time);

    void undistortKeyPoints(const cv::Mat &K, const cv::Mat &D);
    void computeBoundUn(const cv::Mat &K, const cv::Mat &D);

    // Image Info
    cv::Mat img;
    static float minXUn;
    static float minYUn;
    static float maxXUn;
    static float maxYUn;
    bool inImgBound(cv::Point2f pt);
    ORBextractor* mpORBExtractor;
    std::vector<cv::KeyPoint> keyPoints;

#ifndef USING_NORM_PLANE
    std::vector<cv::KeyPoint> keyPointsUn;
#else
    std::vector<cv::Point2f> keyPointsNormPlane;
#endif

    cv::Mat descriptors;
    int N;
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Scale Pyramid Info
    int mnScaleLevels;
    float mfScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    static bool mbInitialComputations;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(cv::KeyPoint &kp, int &posX, int &posY);


    // pose info: pose to ref KF, pose to World, odometry raw.
    cv::Mat Tcr;
    cv::Mat Tcw;
    Se2 mOdo;

    int id;
    static int nextId;

    //! Multi-thread processing
    std::mutex mMutexImg;
    void copyImgTo(cv::Mat & imgRet);

    std::mutex mMutexDes;
    void copyDesTo(cv::Mat & desRet);

    vslam::CameraPtr camera_;

protected:
    float mTime;
};

typedef std::shared_ptr<Frame> PtrFrame;

} // namespace se2clam
#endif // FRAME_H
