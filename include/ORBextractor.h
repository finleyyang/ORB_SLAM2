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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;  //图像金字塔每层的图像

protected:

    void ComputePyramid(cv::Mat image);
    //计算金字塔模型
    //1.先进行图片的缩放，缩放到mvInvscaleFactor对应的尺寸
    //2.在图像外补一圈厚度为19的padding(提取FAST特征点需要特征点周围半径为3的圆域，计算ORB描述子需要特征点周围半径为16的圆域)


    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    //响应值描述的是该特征点的区分度大小
    //响应值越大的点越应该被留用作特征点
    //响应值类似于分数，分数越高的学生越好，越应该被录取

    //描述子是特征点的哈希运算
    //其大小无意义，仅用来在数据库中快熟找回某个特征点
    //描述子相当于学生的学号，系统随机运算出的一串数，用于找到该学生

    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    //分块搜索特征点，若某块区域内特征点普遍比较小的话就降低分数再搜索一遍，这里特征点的响应值最大为20，最小为7
    //对得到的所有特征点进行八叉树筛选，若某区域特征点数目过于密集，则只取其中响应值最大的那一个


    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    // 从配置文件中提取
    int nfeatures;  //所有层级提取的特征点之和和金字塔层数   TUM1.yaml中的值 1000
    double scaleFactor; //图像金字塔相邻层级间的缩放系数    TUM1.yaml中的值 1.2
    int nlevels; //金字塔层级数  TUM1.yaml中的值 8
    int iniThFAST; //提取特征点的描述子门槛（高）  TUM1.yaml中的值 20
    int minThFAST; //提取特征点的描述子门槛（低）  TUM1.yaml中的值 7

    //根据上述变量的值计算出下面成员变量
    std::vector<int> mnFeaturesPerLevel; //金字塔每层级中提取的特征点数正比于图层的边长，总和为nfeatures 这里的值为 {61， 73， 87， 105， 126， 151， 181， 216}

    std::vector<int> umax;
    //计算一个半径为16的，圆的近似坐标
    // 特征点是带方向的
    //计算描述子过程中要将特征点周围像素旋转到主方向上，因此计算一个半径为16的圆的近似坐标，用于后面计算描述子时进行旋转操作

    std::vector<float> mvScaleFactor; //金字塔各层级的缩放系数 这里的值为{1， 1.2， 1.44， 1.728， 2.074， 2.488， 2.986， 3.583}
    std::vector<float> mvInvScaleFactor;    //金字塔各级缩放系数的倒数 这里的值为{1， 0.833， ...}
    std::vector<float> mvLevelSigma2;  //金字塔各层级的缩放系数的平方 这里的值为{1， 1.44，...}
    std::vector<float> mvInvLevelSigma2; //金字塔各层级的缩放系数的平方的倒数 这里的值为{1， 0.689， ...}
};

} //namespace ORB_SLAM

#endif

