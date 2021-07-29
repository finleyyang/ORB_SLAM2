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
    /**
    * @brief 在八叉树分配特征点的过程中，实现一个节点分裂为4个节点的操作
    *
    * @param[out] n1   分裂的节点1
    * @param[out] n2   分裂的节点2
    * @param[out] n3   分裂的节点3
    * @param[out] n4   分裂的节点4
    */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    //保存有当前节点的特征点
    std::vector<cv::KeyPoint> vKeys;
    //当前节点所对应的图像坐标边界
    cv::Point2i UL, UR, BL, BR;
    //存储提取器节点的列表（其实就是双向链表）的一个迭代器,可以参考[http://www.runoob.com/cplusplus/cpp-overloading.html]
    //这个迭代器提供了访问总节点列表的方式，需要结合cpp文件进行分析
    std::list<ExtractorNode>::iterator lit;

    //如果节点中只有一个特征点的话，说明这个节点不能够再进行分裂了，这个标志置位
    //这个节点中如果没有特征点的话，这个节点就直接被删除了
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
    * @brief 构造函数
    * @detials 之所以会有两种响应值的阈值，原因是，程序先使用初始的默认FAST响应值阈值提取图像cell中的特征点；如果提取到的
    * 特征点数目不足，那么就降低要求，使用较小FAST响应值阈值进行再次提取，以获得尽可能多的FAST角点。
    * @param[in] nfeatures         指定要提取出来的特征点数目
    * @param[in] scaleFactor       图像金字塔的缩放系数
    * @param[in] nlevels           指定需要提取特征点的图像金字塔层
    * @param[in] iniThFAST         初始的默认FAST响应值阈值
    * @param[in] minThFAST         较小的FAST响应值阈值
    */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    /**
    * @brief 使用八叉树的方法将提取到的ORB特征点尽可能均匀地分布在整个图像中
    * @details 这里是重载了这个ORBextractor类的括号运算符;函数中实际上并没有用到MASK这个参数。
    *
    * @param[in] image         要操作的图像
    * @param[in] mask          图像掩膜，辅助进行图片处理，可以参考[https://www.cnblogs.com/skyfsm/p/6894685.html]
    * @param[out] keypoints    保存提取出来的特征点的向量
    * @param[out] descriptors  输出用的保存特征点描述子的cv::Mat
    */
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    /**
     * @brief 获取图像金字塔的层数
     * @return int 图像金字塔的层数
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * @brief 获取当前提取器所在的图像的缩放因子，这个不带s的因子表示是相临近层之间的
     * @return float 当前提取器所在的图像的缩放因子，相邻层之间
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * @brief 获取图像金字塔中每个图层相对于底层图像的缩放因子
     * @return std::vector<float> 图像金字塔中每个图层相对于底层图像的缩放因子
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * @brief 获取上面的那个缩放因子s的倒数
     * @return std::vector<float> 倒数
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /**
    * @brief 获取sigma^2，就是每层图像相对于初始图像缩放因子的平方，参考cpp文件中类构造函数的操作
    * @return std::vector<float> sigma^2
    */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
    * @brief 获取上面sigma平方的倒数
    * @return std::vector<float>
    */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    //这个是用来存储图像金字塔的变量，一个元素存储一层图像
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

