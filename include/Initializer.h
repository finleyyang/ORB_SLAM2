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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// *****THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.********
//该class是从ORB_SLAM1中留下来的只能用于单目初始化，单目初始化需要两帧图片
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:

    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;               //参考帧的特征点，在参考帧的时候创建Initializer对象

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;               //当前帧的特征点，在当前帧的时候调用Initializer对象

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;                //从参考帧到当前帧的匹配特征点对，vector元素是Match，是整数点对
    vector<bool> mvbMatched1;                  //参考帧特征点是否在当前帧存在匹配特征点

    // Calibration
    cv::Mat mK;                           //相机内参

    // Standard Deviation and Variance
    float mSigma, mSigma2;            //重投影误差阀值及其平方

    // Ransac max iterations
    int mMaxIterations;                    //RANSAC迭代次数

    // Ransac sets
    vector<vector<size_t> > mvSets;      //二维容器N*8，每一层 保存RANSAC计算H和F矩阵说需要的八对点

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
//RANSAC算法
//少数外电会极大影响计算结果的准确度，随着采样数量的增加，外点数量也会增加，外点数量也会增加，这是一种系统误差，无法通过增加采样点来解决。
//RANSAC(随机采样一致性)耍法的思路是少量多次重复试验，每次实验仅使用尽可能少的点来计算，并统计本次计算中的内点数，只要尝试足够多的话，钟会找到一个包含所有内点的解。
//RANSAC的核心是减少每次迭代所需的采样点的数量。从原理上来说，计算F矩阵最少只需要7对匹配点，计算H矩阵最少只需要4对匹配点，ORB—SLAM2为了编程方便，每次迭代使用8对匹配点计算F和H。

