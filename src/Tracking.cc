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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

//Tracking流程
//1.图像预处理，提取特征点
//2.初始估计当前帧位置，
//3.基于当前帧，构建局部地图（不是用来建图，只是用于当前帧的优化）
//4.关键帧的提取

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    //strSettingPath配置文件的路径
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //相机的内参传参数
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    //相机的畸变参数传给参数
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    //创建特征点提取器对象
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

//每次传进来一帧图片就调用GrabImageStereo
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    //图像通道转换
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }
    //每一次进来数据就调用一下Frame
    //创建frame对象
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;
    //图像通道转换
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }
    //构造Frame
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


// Track主函数，进行跟踪
void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        //如果没有图片输入，那么track状态转换乘未初始化
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    // 地图更新时加锁。保证地图不会发生变化
    // 疑问:这样子会不会影响地图的实时更新?
    // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        //初始化
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            //双目初始化
            StereoInitialization();
        else
            //单目初始化
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        //系统已经被初始化了，开始跟踪
        //初始位姿估计主要有3种方法
        //1. 根据恒速运动来估计位姿
        //2. 根据参考帧估计位姿
        //3. 通过重定位估计位姿
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        //mbOnlyTracking等于false表示正常SLAM模式定位加地图更新，mbOnlyTracking等于True表示仅定位模式
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // 检查并更新上一帧被替换的MapPoints
                // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrame();

                //如果运动模型为空，这证明刚开始初始化，或者已经跟丢了
                //如果当前帧紧紧跟着重定位的帧的后面，我们将重定位帧来恢复运动
                //mnLastRelocFrameId 上一次重定位的那一帧
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //用最近的关键帧来跟踪当前的普通帧
                    //通过BoW的方式在参考帧中寻找当前帧特征点的匹配帧的匹配点
                    //优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    //用恒速模型来跟踪当前帧
                    // 用最近的普通帧来跟踪当前的普通帧
                    // 根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        //根据恒速模型失败了，只能根据参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                //如果跟踪状态失败了，只能重新定位了
                //BoW搜索，PnP求解
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            // 仅进行定位算法
            if(mState==LOST)
            {
                //重定位
                bOK = Relocalization();
            }
            else
            {
                //mbVO 是mbOnlyTracking为true时的才有的一个变量
                //mbVO 为false 表示此帧匹配了很多的MapPoints，跟踪正常
                //mbVO 为true 表示此帧匹配了很少的MapPoints，少于10个，跟踪失败
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // 如果跟踪正常，使用恒速模型或者使用参考关键帧跟踪
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                        // 恒速模型失败的情况下是不是需要参考关键帧跟踪
                        //if(!bOK)
                        //    根据恒速模型失败了，只能根据参考关键帧来跟踪
                        //    bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为true，表明此帧匹配了很少的地图点，即需要做跟踪又要做重定位

                    // MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;
                    //通过重定位方法来跟踪的结果
                    bool bOKReloc = false;

                    // 运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    // 在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    // 运动模型得到位姿
                    cv::Mat TcwMM;

                    //当运动模型有效的时候，根据运动模型计算位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        //根据运动模型追踪的时候，该帧的地图点和外点的数量都会发生变化
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }

                    //使用重定位，获得位姿
                    //位姿已经在重定位函数中被赋值了
                    //？为什么重定位的时候要把位姿给赋值呢？
                    //重定位是EPnP优化算法产生的，需要赋值进行优化
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        //恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        //mbVO 为true 表示此帧匹配了很少的MapPoints，少于10个，跟踪失败
                        if(mbVO)
                        {
                            // 更新当前帧的地图点被观测次数
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                //如果这个特征点形成了地图点，并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    //增加能观测到该地图点的帧数
                                    //实际有多少能观测到
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        //只要重定位成功整个跟踪过程正常进行
                        mbVO = false;
                    }
                    // 重定位与运动估计有一个成功就可以
                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        //将最新的关键帧作为当前帧的参考关键帧
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // 在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化


        //位姿估计完之后，进行追踪局部地图点

        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            // 重定位成功
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        //根据上面的流程，来判断是否追踪成功
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        // 更新显示线程中的图像，特征点，地图点等信息
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        // 只有成功追踪时，才考虑生成关键帧问题
        if(bOK)
        {
            // Update motion model
            // 更新成功后，跟新恒速运动模型
            if(!mLastFrame.mTcw.empty())
            {
                // 更新横竖模型
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                // 从相机坐标到世界坐标
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                //否者速度为空
                mVelocity = cv::Mat();

            //更新显示中的位姿
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            // 清除观测不到的地图点
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // 清除恒速模型中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
            // 步骤6中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就舍弃了
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            // 若跟踪正常，根据条件判定是否产生关键帧
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
            // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

            // 删除那些在BA中被检测为外点的地图点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // 如果初始化不了就跟踪失败，并且relocation也没有搞定，只能reset了
        if(mState==LOST)
        {
            //如果地图中的关键帧信息过少的话,直接重新进行初始化了
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        //确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        //将当前帧记录为上一帧
        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // 记录位姿信息，用于最后保存所有的轨迹
    if(!mCurrentFrame.mTcw.empty())
    {
        //计算相对位姿， Tcr = Tcw * Twr, Twr = Trw^-1
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        // 如果跟踪失败，则相对位姿使用上一次值
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

//双目RGBD初始化
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        //基于当前帧构造初始化帧
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        // 将当前帧构造为初始关键帧
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        //在地图中添加初始关键帧
        //KeyFrame中包含了地图，地图中也包含了KeyFrame
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        //构建地图点
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            //只有特征点深度大于0的情况才能被构建成地图点
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                //通过反投影得到特征点在世界坐标系下的3D坐标
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);

                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                // 为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围

                pNewMP->AddObservation(pKFini,i);
                //表示pNewMP该地图点，可以被哪一帧的哪一个特征点
                pKFini->AddMapPoint(pNewMP,i);
                //表示pKFini改关键帧，可以观测到哪一个地图点，在关键帧上是哪一个

                pNewMP->ComputeDistinctiveDescriptors();
                // 从众多观测到该MapPoint的特征点中挑选区分度最高的描述子

                pNewMP->UpdateNormalAndDepth();   //更新平均观测方向和距离
                mpMap->AddMapPoint(pNewMP);

                // 将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        //在局部地图里添加该初始关键帧
        //局部地图的入口(双目初始化)
        mpLocalMapper->InsertKeyFrame(pKFini);

        //更新当前帧为上一帧
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        //把初始化关键帧添加到trackong中的localkeyframe
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    //若单目初始化还没创建，则创建初始化器
    if(!mpInitializer)
    {
        // Set Reference Frame
        //创建初始化器，只要当前帧的特征点的数量大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            //参考帧和上一帧在该时刻都被设置
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            //将当前帧的特征点赋给参考关键帧
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            // 初始化之前，先确定没有被初始化
            // 删除前判断一下，来避免出现段错误。不过在这里是多余的判断
            // 不过在这里是多余的判断，因为前面已经判断过了
            if(mpInitializer)
                delete mpInitializer;

            //将关键参考帧，以及迭代次数（RANSAC）赋给单目初始化对象
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);//单目相机参考帧初始化，即第一帧

            //初始化为-1，表示没有任何的匹配
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        // 如果这一帧特征点不够多，这不能被用来初始化
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            //删除初始化器
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        // 进行初始化，在mInitialFrame（mLastFrame）和当前帧之间进行匹配搜索
        ORBmatcher matcher(0.9,true);
        // 只在周围的3X3网格里寻找相匹配的特征点
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences

        // 如果匹配的特征点数目太少，则匹配失败，删除初始化器
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        //进行单目初始化
        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            //单目初始化成功后，
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                //删除未成功三角化的匹配点对
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            //创建初始化地图，以mInitialFrame为参考系, 设置当前帧的位姿
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            //创建初始化局部地图
            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    //mInitialFrame是上一帧，mCurrentFrame是当前帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    //计算两个关键帧的词袋模型
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // 将两个关键帧插入地图
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    //创建地图点并且处理地图点
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        //直接把初始化化的3D坐标点（三角化成功的）加入到世界地图
        cv::Mat worldPos(mvIniP3D[i]);

        //创建并添加地图点
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        //将该地图点添加到上一帧上，并把该地图点在匹配的特征点里第几个也赋值进去
        //添双向观测，添加关键帧里对地图点的观测同时也添加地图点对关键帧的观测
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
        //添加地图点到关键帧的观测
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        //计算地图点描述子并更新平均观测数据
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth(); //创建单目初始化地图

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    //基于观测到的地图点，更新关键帧共视图
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    // 全局BA，优化所有关键帧位姿和地图点，
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    //将两帧间的平移尺度归一化（以场景的中值深度作为参考）
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    //将坐标尺度也归一化
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    //将关键帧插入局部地图，更新归一化后的位姿和地图坐标
    //单目初始化，创建局部地图
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    //更新跟踪状态变量mStatte
    mState=OK;
}


// 检查上一帧的地图点是否需要被替换
// Local Mapping 线程可能会将关键帧中某些地图点进行替换，由于tracking中需要用到上一帧地图点，所以这里检查并更新上一帧中被替换的地图点

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        //如果该地图点是否存在
        if(pMP)
        {
            // 获取其是否被替换,以及替换后的点
            // 这也是程序不直接删除这个地图点删除的原因
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                // 然后替换掉
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}



/*
 * @brief 用参考关键帧的地图点来对当前普通帧进行跟踪
 *
 * Step 1：将当前普通帧的描述子转化为BoW向量
 * Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
 * Step 3: 将上一帧的位姿态作为当前帧位姿的初始值
 * Step 4: 通过优化3D-2D的重投影误差来获得位姿
 * Step 5：剔除优化后的匹配点中的外点
 * @return 如果匹配数超10，返回true
 *
 */

//每当Tracking线程创建一个新的关键帧时，就将其设为参考关键帧
//跟踪局部地图函数Tracking::TrackLocalMap()内部会将与当前帧共视点最多的局部关键帧设为参考关键帧

bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    //将当前帧的描述子转成BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    //通过词袋模型加速当前帧与参考帧之间的特征点匹配
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    //匹配点数目小于15则认为跟踪失败
    if(nmatches<15)
        return false;

    //将上一帧的位置作为当前帧的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    //应用重投影误差，来优化获取更精准的位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    //剔除优化后的匹配点中的外点，优化过程中有了对外点的标记
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //如果对应的这个点为外点
            if(mCurrentFrame.mvbOutlier[i])
            {
                //从当前帧把这个数据删除
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }
    // 跟踪成功的数目超过10才认为跟踪成功，否则跟踪失败
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    //根据参考关键帧确定当前帧的位姿,使用关键帧是因为参考关键帧位姿更准确
    //上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    //ref_keyframe 到 lastframe的位姿变换
    cv::Mat Tlr = mlRelativeFramePoses.back();

    //将上一帧的世界坐标下的位姿计算出来
    //l:last, r:reference, w:world
    //Tlw = Tlr*Trw
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    //对于双目/RGBD相机,生成新的临时地图点
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // 将上一帧中存在深度的特征点按深度从小到大排序
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;
    //构成深度以及这个特征点的标签后，按深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // 将上一帧中没有三角化的特征点三角化出来，作为临时地图点
    int nPoints = 0;   //统计处理了多少特征点
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        //检查特征点有没有没三角化出来，没有的把特征点三角化出来，或者没有被观测到的，作为临时地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            //地图点创建后，如果没有被观测到，也需要重新创建地图点
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            //首先计算出该特征点的世界坐标
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            //创建地图点 1.三维点 2.追踪的地图 3.这个特征点存在的帧 4.特征点ID
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            //加入上一帧的地图点中
            mLastFrame.mvpMapPoints[i]=pNewMP;
            //添加到临时地图点中，之后地图点会被删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}


/**
 * @brief 根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
 * Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
 * Step 2：根据上一帧特征点对应地图点进行投影匹配
 * Step 3：优化当前帧位姿
 * Step 4：剔除地图点中外点
 * @return 如果匹配数大于10，认为跟踪成功，返回true
 */

bool Tracking::TrackWithMotionModel()
{
    // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    //更新上一帧的位姿;对于双目或者RGBD相机，还会根据深度值产生临时地图点
    UpdateLastFrame();

    //用之前估计的速度，用恒速模型得到当前帧的初始位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    //清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    //设置匹配过程中的搜索半径
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    //用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        //扩大半径再搜索一次
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }
    //如果还不够则认为跟踪失败
    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    //利用3D-2D投影关系，优化当前帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    //删除地图点中的外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

/**
 * @brief 用局部地图进行跟踪，进一步优化位姿
 * @return true if success
 *
 * Step 1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
 * Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
 * Step 3：更新局部所有MapPoints后对位姿再次优化
 * Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
 * Step 5：决定是否跟踪成功
 */

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    // 更新局部地图，局部关键帧 mvpLocalKeyFrame， 局部地图点 mvpLocalMapPoints
    UpdateLocalMap();

    //筛选局部地图中新增在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
    SearchLocalPoints();

    // Optimize Pose
    // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化，
    // 前面新增了更多的匹配关系，BA优化得到更准确的位姿
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // 更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //由于当前帧的地图点可以被当前帧观测到，其被观测统计量加1
            if(!mCurrentFrame.mvbOutlier[i])
            {
                //找到该点的帧数mnFound+1
                //实际有多少能观测到
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                //查看是否在定位模式
                if(!mbOnlyTracking)
                {

                    // 如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    // nObs： 被观测到的相机数目，单目+1，双目或RGB-D则+2
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    //记录当前帧跟踪到的地图点数目，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            //如果这个地图点是外点，并且当前相机输入还是双目的时候，就删除这个点
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // 根据跟踪匹配数目及重定位情况决定是否跟踪成功
    // 如果最近刚刚发生了重定位,那么至少成功匹配50个点才认为是成功跟踪
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    //如果是正常的状态的话只要跟踪的地图点大于30个就认为成功了
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

/**
 * @brief 判断当前帧是否需要插入关键帧
 *
 * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
 * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
 * Step 3：得到参考关键帧跟踪到的地图点数量
 * Step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
 * Step 5：对于双目或RGBD摄像头，统计可以添加的有效地图点总数 和 跟踪到的地图点数量
 * Step 6：决策是否需要插入关键帧
 * @return true         需要
 * @return false        不需要
 */

bool Tracking::NeedNewKeyFrame()
{
    //纯OV模式下，即纯定位模式下，不需要插入关键帧
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // 如果局部地图线程被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // 获取当前地图中的关键帧数目
    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // nCurrentFrame.mnId 是当前帧的ID
    // mMaxFrames 等于图像输入帧率
    // 如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    // 得到参考关键帧跟踪的地图点数量
    // 地图点的最小观测次数
    // 如果该地图点的关键帧小于2，则最小观测次数为2
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;

    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // 查询局部地图线程是否繁忙，当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    // 对于双目或者RGBD摄像头，统计成功追踪的近点数量，如果追踪的近点数量太少，没有跟踪的近点数量较多，则可以插入关键帧
    int nNonTrackedClose = 0;   //双目或者RGBD没追踪到近点数量
    int nTrackedClose= 0;       //双目或者RGBD追踪到的近点数量（三维点）
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            // 深度在有效范围内
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                // 是关键点且不是外点
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    // 双目或RGBD情况下，跟踪到的地图点太少，并且，没有跟踪到的地图点太多，就可以插入关键帧了
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    // 决策是否需要插入关键帧
    // 设置阀值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
    float thRefRatio = 0.75f;

    // 关键帧只有一帧，那么插入关键帧的概率就低一点，插入概率较低
    if(nKFs<2)
        thRefRatio = 0.4f;

    // 单目情况下插入关键帧的概率很高
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // 很长时间没有插入关键帧了，就插入关键帧 当前帧的ID>=上一帧的ID+最大帧率
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;

    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // clb = bLocalMappingIdle ???
    // 若c1a为false 则c1b必为false c1a||c1b为false
    // 若c1a为true 不管c1b为什么 c1a||c1b为true
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);

    //Condition 1c: tracking is weak
    //只考虑双目，RGBD情况
    const bool c1c =  mSensor!=System::MONOCULAR &&   //RGBD情况下
            (mnMatchesInliers<nRefMatches*0.25 ||     //当前帧和地图点匹配的数目非常少
            bNeedToInsertClose) ;                     //需要插入关键帧


    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 和参考帧相比当前帧跟踪的点太小 或者 满足bNeedToInsertClose:同时跟踪的内点还不能太少
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        // Local mapping线程空闲的时候可以插入， 不空闲的时候，另外讨论
        // 这里又对LocalMappingIdle进行判断。所以c1b条件是否必要？？？？？？？？？？？
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                // 队列里不能阻塞太多关键帧
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再逐个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    //队列里的关键帧不是很多，可以插入
                    return true;
                else
                    return false;
            }
            else
                //对于单目情况下就不能插入关键帧了
                return false;
        }
    }
    else
        return false;
}

/**
 * @brief 创建新的关键帧
 * 对于非单目的情况，同时创建新的MapPoints
 *
 * Step 1：将当前帧构造成关键帧
 * Step 2：将当前关键帧设置为当前帧的参考关键帧
 * Step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
 */

void Tracking::CreateNewKeyFrame()
{
    // 如果局部建图关闭了，就无法插入关键帧了
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // 产生关键帧
    // 将当前帧设为关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // 将当前关键帧设为参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    //这段代码和UpdateLastFrame功能相同
    if(mSensor!=System::MONOCULAR)
    {
        //更新mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // 计算当前帧具有深度的特征点
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                // 第一个元素是深度,第二个元素是对应的特征点的id
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            //按深度，从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            // 找出其中不是地图点的，三角化生成临时地图点
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                //这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    // 地图点添加观测帧
                    pNewMP->AddObservation(pKF,i);
                    //帧添加地图点
                    pKF->AddMapPoint(pNewMP,i);
                    //计算描述子
                    pNewMP->ComputeDistinctiveDescriptors();
                    //更新平均观测方向和距离
                    pNewMP->UpdateNormalAndDepth(); //创建新的关键帧的时候
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    //记录其中不需要创建地图点的个数
                    nPoints++;
                }
                // 1、当前的点的深度已经超过了设定的深度阈值（40倍基线）
                // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    //将关键帧传给localmapping线程, 插入关键帧
    mpLocalMapper->InsertKeyFrame(pKF);

    // 插入好了，允许局部建图停止
    mpLocalMapper->SetNotStop(false);

    // 当前帧成为新的关键帧，更新
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

/**
 * @brief 用局部地图点进行投影匹配，得到更多的匹配关系
 * 注意：局部地图点中已经是当前帧地图点的不需要再投影，只需要将此外的并且在视野范围内的点和当前帧进行投影匹配
 */

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //遍历当前帧的地图点，标记这些地图点不参与之后的投影搜索匹配
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                //更新能观测到该点的帧数+1
                //理论上观测的
                pMP->IncreaseVisible();
                //标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                //标记该点在后面搜索匹配时不被投影，因为已经有匹配对应点了
                pMP->mbTrackInView = false;
            }
        }
    }

    // 准备进行投影匹配的点的数目
    int nToMatch=0;

    // Project points in frame and check its visibility
    // 判断所有地图点中除去当前帧地图点外的点，是否在当前帧视野范围内
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        // 已经被当前帧观测到的肯定在视野范围内，跳过
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        // 判断当前点是否在当前帧视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1
            // 局部关键帧中补属于当前帧，但在当前帧视锥范围内, 理论上应该能观察到
            pMP->IncreaseVisible();
            // 只有在视野范围内的地图点才参与之后的投影匹配
            nToMatch++;
        }
    }

    // 如果需要进行投影匹配的点的数目大于0，就进行投影匹配，增加更多的匹配关系
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD) //RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();  //更新局部关键帧列表
    UpdateLocalPoints();     //更新局部地图点列表
}

/*
 * @brief 更新局部关键点。先把局部地图清空，然后将局部关键帧的有效地图点添加到局部地图中
 */

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();
    //遍历每一个局部地图点
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        // 提取每一个关键帧的地图点
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 跟踪局部地图函数里，更新局部关键帧
 * 方法是遍历当前帧的地图点，将观测到这些地图点的关键帧和相邻的关键帧及其父子关键帧，作为mvpLocalKeyFrames
 * Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
 * Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧包括以下3种类型
 *      类型1：能观测到当前帧地图点的关键帧，也称一级共视关键帧
 *      类型2：一级共视关键帧的前十大共视关键帧，然后取其中第一个没有被列到当前局部地图图关键帧中，称为二级共视关键帧
 *      类型3：一级共视关键帧的子关键帧、父关键帧
 * Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
 */

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    //mCurrentFrame.N当前关键帧的关键点的数量
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                // 得到观测到该地图点的关键帧和该地图点在关键帧中的索引
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                // 由于一个地图点可以被多个关键帧观测到,因此对于每一次观测,都对观测到这个地图点的关键帧进行累计投票
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
                    // it->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
                    // 所以最后keyframeCounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧(mCurrentFrame)的地图点，也就是共视程度
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    // 没有当前帧的共视关键帧，返回
    if(keyframeCounter.empty())
        return;

    //储存有最多次观测次数的关键帧
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // 更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有3种类型
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    // 先申请3倍空间
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // 类型1：能观测到当前帧地图点的关键帧作为局部关键帧 （一级共视关键帧）
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        // 如果设定为要删除的，跳过
        if(pKF->isBad())
            continue;

        // 寻找具有最大观测数目的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        //将共视关键帧添加到局部关键帧的列表里 （一级局部关键帧）
        mvpLocalKeyFrames.push_back(it->first);
        // 用该关键帧的成员变量mnTrackReferenceForFrame 记录当前帧的id
        // 表示它已经是当前帧的局部关键帧了，可以防止重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // 遍历局部关键帧列表，寻找更多的局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        // 局部关键帧列表不能超过80个，否者影响计算速度
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        // 对于每个局部关键帧，获得其共视最高的10帧
        // 如果其共视关键帧不超过10帧，那样就全部返回
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
        // vNeighs 是按照共视程度从大到小排列

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    //找到一帧就跳出，所以一级的局部关键帧中，每一帧，只取一帧它的共视关系最强的
                    break;
                }
            }
        }

        // Spanning Tree 的应用1

        //不明白为什么每一帧的孩子帧和父亲帧只添加一帧？？？？
        //共视关键帧有共视强度的排序，所有能明白只添加第一帧，孩子帧和父亲帧呢？？

        //父亲帧和孩子帧理论上也是共视程度最高的帧

        //将一级关键帧的孩子帧的第一帧加入到局部关键帧中
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        //将一级关键帧的父亲帧的第一帧加入到局部关键帧中
        // 跳出大循环？？？
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/**
 * 重定位过程
 * @return true
 * @return false
 *
 * step 1：计算当前帧的词袋向量
 * step 2：找到与当前帧相似的候选关键帧
 * step 3：通过BoW进行匹配
 * step 4：通过EPnP算法进行估计位姿
 * step 5：通过PoseOptimization对位姿进行优化求解
 * setp 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，之后再进行优化求解
 *
 */

//进行位姿估计两次BA优化，主要目的防止误定位

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    //计算当前帧的词袋模型
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // 找到与当前帧相似的候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    // 如果没有候选点则退出
    if(vpCandidateKFs.empty())
        return false;

    //候选关键帧的数量
    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    //初始化每个关键帧的解算器
    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    //每个关键帧与当前帧中特征点的匹配关系
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    //放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    //有效候选关键帧的数目
    int nCandidates=0;

    //遍历所有候选关键帧，通过词袋进行快速匹配，用匹配结果进行初始化PnPSolver
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            //当前帧和候选关键帧用BoW进行快速的匹配，匹配结果记录在vvpMapPointMatches上， nmatches表示匹配的数目
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            //如果和当前帧的匹配数量小于15时，那么只能放弃这个关键帧
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                //EPnP的好处因为计算复杂度小，精度高
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(
                        0.99,  //用于计算RANSAC迭代次数理论值的概率
                        10,  //最小内点数
                        300,  //最大迭代次数
                        4,        //最小集求解这个问题的说需要的最小样本数，对于Sim3是3，Epnp是4
                        0.5,     //(最小内点数/样本总数)，实际上的RANSAC正常退出的时候所需要的最小内点数其实就是根据这个量来计算的
                        5.991);  //自由度为2的卡方检验的阀值
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // 这之前只是初始化，并没有迭代求解

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    // 是否已经找到匹配关键帧的标志
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        //遍历当前所有的候选关键帧
        for(int i=0; i<nKFs; i++)
        {
            //忽略放弃的
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            // 内点标记
            vector<bool> vbInliers;

            // 内点数
            int nInliers;
            // 表示RANSAC已经没有更多的迭代次数可用
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            // 通过EPnP算法估计姿态，迭代5次
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            // bNoMore为true，表示已经超过RANSAC最大迭代次数，就放弃当前关键帧
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                //如果EPnP，计算出了位姿，对内点进行BA优化
                Tcw.copyTo(mCurrentFrame.mTcw);

                //EPnP 里RANSAC后的内点的集合
                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                //遍历所有的内点
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }
                // 只优化位姿，不优化地图点的坐标，返回的是内点的数量
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                //如果优化完之后的内点数量不多，跳过了当前候选关键帧，但是却没有放弃当前的重定位
                if(nGood<10)
                    continue;

                // 删除外点对应的地图点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                //如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                //前面的匹配关系是通过词袋匹配过程得到的
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(
                            mCurrentFrame,   //当前帧
                            vpCandidateKFs[i],   //关键帧
                            sFound,         //已经找到的地图点集合，不会用于PNP
                            10,            //窗口阀值，会乘以金字塔尺度
                            100);    //匹配的ORB描述子距离应该小于这个阀值

                    if(nadditional+nGood>=50)
                    {
                        //根据投影匹配的结果再次采用3D-2D PnP BA优化位姿
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        // 如果BA后内点的数还是比较少，但是没有少于30，可以尝试下，减少匹配的距离（即使用更小的窗口搜索），似的匹配的结果更加精准
                        // 尽量避免误定位
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64); //特征点的距离阀值从100变成64

                            // Final optimization
                            // 如果匹配点的数量达到要求，最后再BA优化下
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                //更新地图点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                            //如果还是不能满足就放弃
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                // 如果对于当前的候选关键帧已经有足够多的内点（50个），那就认为重定位成功
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        //  如果匹配成功，说明当前帧的从定位成功，记录成功重定位的iD，防止短时间内多次重定位
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
