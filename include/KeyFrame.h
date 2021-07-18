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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);   //添加共视关键帧
    void EraseConnection(KeyFrame* pKF);             //删除共视关键帧
    void UpdateConnections();                   //基于当前观测帧对地图点的观测构成共视图
    //只要关键帧与地图点间的连接关系发生变换(包括关键帧创建和地图点重新匹配关键帧特征点)函数UpdateConnections()就会被调用
    //Tracking线程中创建关键帧的时候会调用UpdateConnections()初始化共视图
    //Localmapping线程接收到新关键帧时会调用函数LocalMapping::ProcessNewKeyFrame()处理跟踪过程中加入地图点之后会用KeyFrame::updateConnections()初始化共视图信息(关键帧和地图点重新匹配)
    //(实际上这里是处理Tracking线程中函数Tracking::CreateNewKeyFrame()创建的关键帧)
    //LocalMapping线程处理完毕缓冲队列内所有关键帧后会调用LocalMapping::SearchInNeighbours()融合当前关键帧和共视关键帧间的重复地图点之后会调用KeyFrame::UpdateConnects()更新共视图信息
    //LoopClosing线程闭环矫正函数LoopCLosing::CorrectLoop()会多次调用KeyFrame::UpdateConnections()更新共视图信息


    void UpdateBestCovisibles();             //基于共视图信息修改对应变量
     //get方法
     /**
      * @brief 得到与该关键帧连接的关键帧(没有排序的)
      * @return 连接的关键帧
      */
    std::set<KeyFrame *> GetConnectedKeyFrames();



    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    //关键帧增加到生成树中的时机
    //成功创建关键帧之后会调用KeyFrame::UpdateConnections()，该函数第一次被调用时会将该关键帧加入到生成树中
    //共视图的改变（除了删除关键帧以外）不会导致生成树的改变
    //当某个关键帧被删除的时候，与其相连接的结构会发生改变，生成树的结构改变类似最小生成树中的加边法
    void AddChild(KeyFrame* pKF);                  //添加子节点，mpChildrens的set方法
    void EraseChild(KeyFrame* pKF);                //删除子节点，mpChildrens的set方法
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();               //mpParent的set方法
    KeyFrame* GetParent();                        //mpParent的get方法
    bool hasChild(KeyFrame* pKF);                //判断mpChilderns是否为空

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);  //添加该帧的地图点
    void EraseMapPointMatch(const size_t &idx);      //删除该帧第几个地图点
    void EraseMapPointMatch(MapPoint* pMP);          //删除该帧哪一个地图点
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);  //用该地图点替换该帧第几个地图点
    std::set<MapPoint*> GetMapPoints();                 //地图点的get方法（set类）
    std::vector<MapPoint*> GetMapPointMatches();        //地图点的get方法（vector类）
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);          //某一个地图点的get方法
    //关键帧添加对地图点观测的时机
    //1.Tracking线程和LocalMapping线程创建新的地图点之后，会马上调用KeyFrame::AddMapPoint()添加当前关键帧对该地图点的观测
    //2.LocalMapping()两帧图片三角化的时候，创建新的地图点的时候，会调用KeyFrame::AddMapPoint()添加当前关键帧对该地图点的观测
    //LocalMapping线程处理完缓存队列内所有关键帧后会调用LocalMapping::SearchInNeighbors()融合当前关键帧间的重复地图点，其中调用函数ORBmatcher::Fuse()实现融合过程中会调用KeyFrame::AddMapPoint()
    //3.LoopClosing线程闭环矫正函数LoopClosing::CorrectLoop()将闭环关键帧与其匹配关键帧间的地图进行融合，会条用函数KeyFrame::AddMapPoint()

    //关键帧替换地图点和删除对地图点观测的时机：
    //1.MapPoint删除函数MapPoint::SetBadFlag()或替换函数MapPoint::Replace()会调用KeyFrame::ErasMapPointMatch()和KeyFrame::ReplaceMapPointMatch()删除和替换关键帧对题图点的观测
    //2.LocalMapping线程调用进行局部BA优化的函数Optimizer::LocalBundleAdjustment()内部调用函数KeyFrame::EraseMapPointMatch()删除对重投影误差较大的地图点的观测



    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    //若某关键帧参与了回环检测，loopclosing线程就会调用函数KeyFrame::SetNotErase()将该关键帧的成员变量mbNotErase设为true，标记下该关键帧暂时就不要被删除了
    void SetNotErase();        //mbNotErase的set方法
    //成员变量mbToBeErased标记当前KeyFrame是否曾有权力不被删除。LoopClosing线程不再需要某关键帧时，会调用函数KeyFrame::SetErase()剥夺
    //该关键帧不被删除的权力，将成员变量mbNotErase复位为true，若是成员变量mbToErased为true，就会调用函数KeyFrame::SetBadFlag()删除该关键帧
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();        //真正执行删除的函数
    bool isBad();           //mbBad的get方法

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors; // 尺度因子，scale^n，scale=1.2，n为层数
    const std::vector<float> mvLevelSigma2; // 尺度因子的平方
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;  //相机光心坐标

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;                    //当前关键帧观测到的地图点,,,跟观测的特征点是一一对应的
    // （关键帧中有一个变量std::vector<MapPoint*> mvpMapPoints储存该关键帧的地图点）

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;  //当前帧的共视关键帧及权重，无序的保存
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;   //所有共视关键帧列表，按权重从大到小排序
    std::vector<int> mvOrderedWeights;                    //所有共视关键帧的权重列表，按权重从大到小排序

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;                              //当前关键帧是否还未加入到生成树。构造函数中初始化为true，加入生成树后置为false
    KeyFrame* mpParent;                                   //当前关键帧在生成树中的父节点
    std::set<KeyFrame*> mspChildrens;                    //当前关键帧在生成树中的子节点列表
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;                            //当前关键帧是否具有不被删除的权力(即参与后面的回环优化)   初始值是false
    bool mbToBeErased;                          //当前关键帧是否曾有权力不被删除     初始值为false
    bool mbBad;                                //标记是坏帧 初始值是false

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
//KeyFrame的创建：
//tracking线程里面通过函数Tracking::NeedNewKeyFrame()中，判断是否需要关键帧，若需要关键帧，则调用函数Tracking::CreateNewKeyFrame()创建关键帧
//KeyFrame的销毁：
//LocalMapping线程剔除冗余关键帧函数LocalMapping::KeyFrameCulling()中若检查到关键帧为冗余关键帧，则调用函数KeyFrame::SetBadFlag()删除关键帧
