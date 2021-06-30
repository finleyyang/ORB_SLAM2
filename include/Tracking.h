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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>




namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    // tracking线程每收到一帧图像，就调用函数Tracking::GrabImageMonocular()、GrabImageRGBD()、GrabImageStereo()创建一个Frame对象，赋值给mCurrentFrame
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    // 枚举类c++中一个派生类，由用户定义的若干常数量的集合

    //用于表示跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,  //系统没有准备好，一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0,      //还没有接收到输入图像
        NOT_INITIALIZED=1,    //接收到图像但未初始化成功
        OK=2,                 //跟踪成功
        LOST=3                //跟踪失败
    };


    //Trackingh状态变化
    // SYSTEM_NOT_READY -> 加载词典和配置文件 -> NO_IMAGE -> 接受新图像 -> NOT_INITIALIZED -> 初始化（单目初始化）或者（双目初始化） ->
    // OK->Track（失败）—> LOST ->Relocalizaion()重定位成功 ->OK
    //   ->Track(成功) ->OK    ->Relocalizaion()重定位失败 ->LOST

    eTrackingState mState;                     //当前帧的跟踪状态
    eTrackingState mLastProcessedState;         //上一帧的跟踪状态

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;             //当前正在处理的帧
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;        //单目初始化中参考帧与当前帧的匹配关系
    std::vector<cv::Point2f> mvbPrevMatched;  //单目初始化参考帧地图点
    std::vector<cv::Point3f> mvIniP3D;    //单目初始化中三角化得到的地图点坐标
    Frame mInitialFrame;                //单目初始化参考帧(实际就是前一帧)

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();             //双目初始化，只要左目能找到500个地图点，就算双目初始化成功
    //即完成了初始化，又构建了初始化局部地图

    // Map initialization for monocular
    void MonocularInitialization();      //相机单目初始化，需要用到单目初始器, 单目初始化需要相邻的两帧用来初始化

    //单目相机初始化分为两个步骤，1是初始化估计相机两帧之间的相对运动， 2是建立局部地图

    void CreateInitialMapMonocular();      //单目初始化完成后，创建局部地图

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();                                 //更新局部地图
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();                                    //更新局部地图并优化当前位姿
    void SearchLocalPoints();                                 //将局部地图点投影到当前帧特征点上

    //成功估计当前帧的初始位姿后，基于当前位姿更新局部地图，并优化当前帧位姿，主要流程：
    //1.更新局部地图，包括局部关键帧列表std::vector<KeyFrame*> mvpLocalKeyFrames和局部地图点列表std::vector<MapPoint*> mvpLocalMapPoints
    //2.将局部地图投影到当前帧特征点上
    //3.进行位姿BA，优化当前位姿
    //4.更新地图点的观测数值，统计内点个数
    //这里的地图点观测数值会被当作LocalMapping线程中LocalMapping::MapPointCulling()函数剔除坏的点的标准之一；
    //5.根据内点数来判断是否跟踪成功


    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;               //单目初始化器

    //Local Map
    KeyFrame* mpReferenceKF;       //参考关键帧，初始化成功的帧就会被设为关键参考帧

    //参考关键帧的用途:
    //1.Tracking线程中函数Tracking::TrackReferenKeyFrame()根据参考帧估计初始位姿
    //2.用于初始化创建的MapPoint的参考帧mpRefKF，函数MapPoint::UpdateNormalAndDepth()中根据参考帧mpRefKF更新地图点的平局观测距离
    //参考关键帧的指定:
    //1.Tracking线程中函数Tracking::CreateNewKeyFrame()创建完关键帧后，会将新创建的关键帧作为参考帧
    //2.Tracking线程中函数Tracking::TrackLocalMap()跟踪局部地图过程中调用函数Tracking::UpdateLocalMap()，其中调用函数Tracking::UpdateLocalKeyFrame()，将与当前关键帧共视程度最高的关键帧设为参考关键帧

    std::vector<KeyFrame*> mvpLocalKeyFrames;      //局部关键帧列表，初始化成功后向其中添加局部关键帧
    std::vector<MapPoint*> mvpLocalMapPoints;      //局部地图点列表，初始化成功后向其中添加局部地图点
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;                //判断单目特征点和双目特征点的阀值，深度低于该值的被认为是单目特征点或者是双目特征点

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;                 //上一帧
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;           //相机前一帧运动速度，跟踪完局部地图后跟新该成员变量

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;  //双目/RGBD相机输入时,为前一帧生成的临时地图点跟踪成功后该容器会被清空,其中的地图点会被删除
};

} //namespace ORB_SLAM

#endif // TRACKING_H
