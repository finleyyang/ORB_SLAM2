/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
    //网格行数、列数

class MapPoint;
class KeyFrame;


class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    //获取半径为r的圆域内的特征点编号列表
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;



    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    //对于双目相机，知道特征点的左目坐标、右目坐标，不知道特征点深度
    //RGBD相机，知道特征点左目坐标、特征点深度，不知道右目坐标

    //双目视差公式为z/(z-f) = b/(b-d) 其中d = xl-xr
    //z为特征点深度，f为焦距，b为基线长度，d为左右视差

    //双目相机分别提取到左右目特征点之后对特征点进行双目匹配，并通过双目视差估计特征点深度，双目特征点匹配步骤
    //1.粗匹配：根据特征点距离和特征金字塔层级判断匹配，粗匹配关系是按行寻找的，对于左目图像中每个特征点，在右目图像对应行上寻找匹配特征点。
    //2.精匹配：根据特征点周围窗口内容相似度判断。
    //3.亚像素插值，将特征点相似度与匹配坐标之间拟合成二次曲线，寻找最佳匹配位置(得到的是一个小数)。
    //4.记录右目匹配mvuRight和深度mvDepth信息。
    //5.离群点筛选，以平均相似度的2.1倍为标准，筛选离群点。
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //双目相机不需要畸变矫正
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;               //左右目的特征点提取器

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    //相机的内参矩阵都是固定的，而且所有帧公用
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;    //相机畸变矫正参数

    // Stereo baseline multiplied by fx.
    //基线距离乘以内参
    float mbf;

    // Stereo baseline in meters.
    //基线距离
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;        //判断单目特征点和双目特征点的阀值，深度低于该值的被认为是单目特征点或者是双目特征点

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant(多余的) as images must be rectified(改正的，整流的).
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;          //畸变校正前的左右目特征点
    std::vector<cv::KeyPoint> mvKeysUn;                     //畸变校正后的左目特征点

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;               //左目特征点在右目中匹配特征点的横坐标（左右目匹配特征点的纵坐标相同）
    std::vector<float> mvDepth;                //特征点深度

    //对于第i个图像特征点来说
    //其畸变矫正前的左目特征点是mvKeys[i]
    //其畸变矫正前的左目特征点是mvKeysUn[i]
    //其在右目图片中对应的特征点的横坐标为mvuRight[i],纵坐标与mvKeys[i]的纵坐标相同
    //特征点的深度是mvDepth[i]

    //对于单目特征点(单目相机输入的特征点或没有找到右目匹配的左目图像特征点），其mvuRight和mvDepth均为-1

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;                   //左右目的特征点描述子

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // 每个网格的宽度和高度
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;

    //每个网格内特征点编号列表
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; //相机坐标

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    //畸变矫正后的图像边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    //是否为第一帧的标志，如果是第一帧，则需要初始化相机内参
    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    // 对所有特征点进行畸变矫正
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    //计算畸变矫正后的图像边界
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    // 将特征点分配到48行64列的网格中以加速匹配
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center

    // ....cw,即为世界到相机坐标，即为当前目标的世界坐标
    cv::Mat mRcw;  //旋转，从世界坐标到相机坐标  //即为当前相机的pose
    cv::Mat mtcw;  //平移，从世界坐标到相机坐标
    cv::Mat mRwc;  //旋转，从相机坐标到世界坐标
    cv::Mat mOw; //==mtwc 平移，从世界坐标到相机坐标
};

}// namespace ORB_SLAM

#endif // FRAME_H
