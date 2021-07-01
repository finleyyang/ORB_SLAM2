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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);   //mworldPos的set的方法
    cv::Mat GetWorldPos();     //mworldPos的get方法

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();  //mObservation的get方法
    int Observations();                          //nObs的get方法

    //增删函数维护的是mObservations和nObs
    void AddObservation(KeyFrame* pKF,size_t idx);  //添加当前地图点对某KeyFrame的观测
    void EraseObservation(KeyFrame* pKF);           //删除当前地图点对某KeyFrame的观测

    int GetIndexInKeyFrame(KeyFrame* pKF);         //查询当前地图点在某keyframe中的索引
    bool IsInKeyFrame(KeyFrame* pKF);              //查询当前地图点是否在某KeyFrame中

    void SetBadFlag();                  //删除当前地图点
    //1.先将坏点标记mbBad置为true，逻辑上删除该地图点1
    //2.再依次清空当前地图点的成员变量
    bool isBad();                      //查询当前地图点是否被删除（本质上查询mbBad）

    void Replace(MapPoint* pMP);      //使用地图点pMP替换当前地图点
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();                         //mDescriptor的get（）方法

    void UpdateNormalAndDepth();                         //更新平均观测方向和距离

    float GetMinDistanceInvariance();       //
    float GetMaxDistanceInvariance();       //
    int PredictScale(const float &currentDist, KeyFrame*pKF);   //根据某地图点到某帧的观测深度估计在该帧地图上的层级
    int PredictScale(const float &currentDist, Frame* pF);  //根据某地图点到某帧的观测深度估计在该帧地图上的层级

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;                                        //记录当前地图点被多少相机观测到，单目帧每次观测加一，双目帧每次观测加2
    //为啥不是private？？？？

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;  //该地图点融合的关键帧

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:
    //特征点是2D的，相机图像上的点
    //地图点是3D的，根据同一特征点在多个图片中的不同位置三角化得到的
    //地图点比对应某特征点
    //特征点不一定能够三角化出地图点


     // Position in absolute coordinates
     cv::Mat mWorldPos;   //地图点的世界坐标

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;    //当前地图点在某KeyFrame中的索引   KV流 key是keyframe value是size_t整数，当前地图点在该keyframe中的索引
     //  即哪一个关键帧的第几个是该地图点
     // （关键帧中有一个变量std::vector<MapPoint*> mvpMapPoints储存该关键帧的地图点，mObservations存储的就是该数组的索引）

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;  // 当前关键点的特征描述子的中位数（准确说是该描述子到其他描述子的距离最小）

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;   //理论上观测到该地图点的帧数
     int mnFound;    //实际观测到该地图点的帧数

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;   //坏点标记
     MapPoint* mpReplaced;    //用来替换当前地图点的新地图点

     // Scale invariance distances
     float mfMinDistance; //表示地图点匹配在某特征提取器图像金子塔最下层上的特征点的观测距离
     float mfMaxDistance; //表示地图点匹配在某特征提取器图像金子塔最上层上的特征点的观测距离

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
//创建MapPoint的时机
//1，Tracking线程中初始化过程(Tracking::MonocularInitialization()单目初始化和Tracking::StereoInitialization()双目初始化)
//2，Tracking线程中创建关键帧(Tracking::CreateNewKeyFrame())
//3，Tracking线程中恒速运动模型跟踪(Tracking::TrackWithMotionModel())也会产生临时地图点，但这些临时地图点在跟踪后会被马上删除
//4，LocalMapping线程中创建新地图点的步骤(LocalMapping::CreateNewMapPoints())会将当前关键帧与前一关键帧进行匹配生成新的地图点
//删除MapPoint的时机;
//1，localmapping线程中删除差的地图点的步骤(LocalMapping::MapPointCulling())
//2，删除关键帧的函数KeyFrame::SetBadFlag()会调用函数MapPoint::EraseObservation()删除地图点对关键帧的观测，若地图点对关键帧的观测少于2，则地图点无法三角化，就删除该地图点
//替换MapPoint的时机；
//1，LoopClosing线程中闭环矫正(LoopClosing::CorrectLoop())时当前关键帧和闭环关键帧上的地图点发生冲突的时候，会使用闭环关键帧的地图点替换当前关键帧的地图点
//2，LoopClosing线程中闭环矫正(LoopClosing::CorrectLoop()会调用LoopClosing::SearchAndFuse()将闭环关键帧的共视关键帧组中所有地图点投影到当前关键帧的共视关键帧组中，发生冲突就替换