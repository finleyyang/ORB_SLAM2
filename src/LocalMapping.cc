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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
    /*
    * mbStopRequested：    外部线程调用，为true，表示外部线程请求停止 local mapping
    * mbStopped：          为true表示可以并终止localmapping 线程
    * mbNotStop：          true，表示不要停止 localmapping 线程，因为要插入关键帧了。需要和 mbStopped 结合使用
    * mbAcceptKeyFrames：  true，允许接受关键帧。tracking 和local mapping 之间的关键帧调度
    * mbAbortBA：          是否流产BA优化的标志位
    * mbFinishRequested：  请求终止当前线程的标志。注意只是请求，不一定终止。终止要看 mbFinished
    * mbResetRequested：   请求当前线程复位的标志。true，表示一直请求复位，但复位还未完成；表示复位完成为false
    * mbFinished：         判断最终LocalMapping::Run() 是否完成的标志。
    */
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    //死循环，进行两个判断 1是判断是否tracking传过来有关键帧，如果有的话处理关键帧，如果没有的话，判断第二个条件是否停止，如果不停止，则等待3毫秒
    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        // 判断是否有关键帧从tracking线程里传过来
        if(CheckNewKeyFrames())
        {
            // 处理关键帧
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            // 删除劣质地图点
            MapPointCulling();

            // Triangulate new MapPoints
            // 创建新的地图点，当前关键帧，与相邻关键帧通过三角化产生新的地图点，使得跟踪更加稳定
            CreateNewMapPoints();

            // 已经正在处理队列中最后一个关键帧
            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                // 检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                SearchInNeighbors();
            }
            // 终止BA的标志
            mbAbortBA = false;

            // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                // 当局部地图的关键帧大于两个的时候进行局部BA
                // 当前帧的一级共视关键帧会被优化，二级共视关键帧会加入优化图，但是其位姿不会被优化
                // 所有的局部地图点位姿都会被优化
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                // 判断当前帧是否冗余，如果冗余的话，就删除
                // 判断冗余的条件是该帧90%的地图点都能被其他关键帧观察到观测到
                KeyFrameCulling();
            }
            //把localmapping的关键帧传给loopcloser线程
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())   //当要终止当前线程的时候
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                //当未完全结束的时候
                usleep(3000);
            }
            if(CheckFinish())
                //然后确定终止，就跳出循环
                break;
        }

        //查看是否有外部线程要求复位该线程
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        // 如果当前线程已经结束了就跳出主循环
        if(CheckFinish())
            break;

        usleep(3000);
    }

    // 设置线程已经终止
    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

/**
 * @brief 处理列表中的关键帧，
 * 计算BoW
 * 更新观测、描述子、共视图
 * 插入到地图
 */

//(!pMP->IsInKeyFrame(mpCurrentKeyFrame),该处通过判断该地图点是否观测都当前关键帧，来判断该地图点是否是当前关键帧中新生成的
//1. 若地图点是本关键帧跟踪过程中匹配得到的（Tracking::TrackwithMOtionModel(), Tracking::TrackReferenceKeyFrame()和Tracking::SearchLocalPoints()中调用
// 了ORBmatcher::SearchByProjection()和ORBmatcher::SearchByBow()方法），则是之前关键帧中创建的地图点，只需要添加其对当前帧的观测即可。是不用做判断的
//2. 若地图点是本关键帧跟踪过程中新生成的(包括：1.单目或双目初始化Tracking::MonocularInitialization(), Tracking::StereoInitialization(); 2.创建新的关键帧
// Tracking::CreateNewKeyFrame())，则该地图点中有对当前关键帧的观测，是新生成的地图点，放入容器mlNewKeyFrames中供LocalMapping::MapPointCulling()函数筛选

void LocalMapping::ProcessNewKeyFrame()
{
    // 取出队列头的关键帧
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    // 计算该关键帧的词袋向量
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    //根据地图点中是否观测到当前帧来判断该地图点是否是新生成的
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                //如果地图点不是在该关键帧中，则说明地图点不是新生成的，是根据以往的关键帧生成的，则添加地图点对当前帧的观测
                //如果地图点在该关键帧中，则说明地图点是来自双目或RGBD跟踪过程中生成的新的地图点
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    // 该地图点是跟踪本关键帧时匹配得到的，在地图点中加入对当前帧关键帧的观测
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();  //局部地图添加关键帧的观测时
                    // 更新地图点的最佳描述子
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 如果当前帧中已经包含了这个地图点,但是这个地图点中却没有包含这个关键帧的信息
                    // 这些地图点可能来自双目或RGBD跟踪过程中新生成的地图点，或者是CreateNewMapPoints 中通过三角化产生
                    // 将上述地图点放入mlpRecentAddedMapPoints，等待后续MapPointCulling函数的检验
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    // 更新关键帧间的连接关系
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    // 将该关键帧插入地图点中
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

/**
 * @brief 检查新增地图点，根据地图点的观测情况剔除质量不好的新增的地图点
 * mlpRecentAddedMapPoints：存储新增的地图点，这里是要删除其中不靠谱的
 */

//满足以下两个条件的之一的就算是冗余地图点
//1.召回率=实际观察到该地图点的帧数/理论上应当观测到该地图点的帧数 < 0.25 坏点
//2.在创建的3帧内观测数目小于2(双目小于3) 坏点

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            //标准0:地图点在其他地方被删除了
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            //标准1:召回率小于0.25
            // (mnFound/mnVisible） < 25%
            // mnFound ：地图点被多少帧（包括普通帧）看到，次数越多越好
            // mnVisible：地图点应该被看到的次数
            // (mnFound/mnVisible）：对于大FOV镜头这个比例会高，对于窄FOV镜头这个比例会低
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        //mnFirstKFid:第一次观测带该地图点的帧ID
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            //标准2:从创建开始连续3个关键帧内观测数目小于cnThObs（单目小于2），（双目小于3）
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            //通过3个关键帧的考察，认为是好的地图点
            // 从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
            // 因此没有SetBadFlag()，仅从队列中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

/**
 * @brief 用当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
 * Step 1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻关键帧
 * Step 2：遍历相邻关键帧，搜索匹配并用极线约束剔除误匹配，最终三角化
 * Step 3：判断相机运动的基线是不是足够长
 * Step 4：根据两个关键帧的位姿计算它们之间的基础矩阵
 * Step 5：通过词袋对两关键帧的未匹配的特征点快速匹配，用极线约束抑制离群点，生成新的匹配点对
 * Step 6：对每对匹配通过三角化生成3D点,和 Triangulate函数差不多
 * Step 6.1：取出匹配特征点
 * Step 6.2：利用匹配点反投影得到视差角
 * Step 6.3：对于双目，利用双目得到视差角；单目相机没有特殊操作
 * Step 6.4：三角化恢复3D点
 * Step 6.5：检测生成的3D点是否在相机前方,不在的话就放弃这个点
 * Step 6.7：检查尺度连续性
 * Step 6.8：三角化生成3D点成功，构造成MapPoint
 * Step 6.9：为该MapPoint添加属性：a.观测到该MapPoint的关键帧, b.该MapPoint的描述子, c.该MapPoint的平均观测方向和深度范围
 * Step 6.10：将新产生的点放入检测队列
 **/
//将当前关键帧分别与共视程度最高的前10(单目取20)个共视关键帧两两进行特征匹配，生成特征点

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    // nn表示搜索最佳共视关键帧的数目
    // 不同传感器要求不一样，单目的时候需要更多的具有较好共视关系的关键帧来建立地图
    int nn = 10;
    if(mbMonocular)
        nn=20;
    // 从当前关键帧中取出共视程度最好的nn帧相邻共视关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    //特征点匹配设置 最佳距离<0.6*次最佳距离，比较严苛了，不检查旋转
    ORBmatcher matcher(0.6,false);

    // 取出当前帧从世界坐标系到相机坐标系的变换矩阵
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation(); //当前相机旋转的pose
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();   //当前相机的平移pose
    cv::Mat Tcw1(3,4,CV_32F);   //当前相机的pose
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    // 得到当前关键帧（左目）光心在世界坐标系中的坐标
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    // 内参
    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    // 用于后面的点深度的验证，这里的1.5是经验值
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    //记录三角化成功的地图点
    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    // 遍历相邻关键帧，搜索匹配并用极线约束剔除误匹配，最终三角化
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        //相邻关键帧光心在世界的坐标
        cv::Mat vBaseline = Ow2-Ow1;
        //基线向量，两个关键帧之间的相机位移
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            //如果是双目相机，关键帧之间的位移，小于本身的基线距离不生成3D点
            //因为太短的基线下能够恢复的地图点不太稳定
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            //单目相机的情况
            //相邻关键帧的场景深度中值
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);

            //基线与景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            //如果比例特别小，基线太短恢复3D点不准，那么跳过当前邻接的关键帧，不生成3D点
            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        //根据两个关键帧的位姿计算他们的基础矩阵
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        // 通过词袋模型对两个关键帧的未匹配的特征点进行快速匹配，用极线约束抑制离群点，生成新的匹配关系
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        // pKF2的世界坐标的旋转
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        // 对每对匹配点通过三角化生成3D点,和 Triangulate函数差不多
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // 当前匹配点在当前帧中的索引
            const int &idx1 = vMatchedIndices[ikp].first;
            // 当前匹配点在邻接关键帧中的索引
            const int &idx2 = vMatchedIndices[ikp].second;


            //为什么当前帧是双目的情况下，还要判断邻近帧是否是双目呢？？？？

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            // 特征点反投影,其实得到的是在各自相机坐标系下的一个非归一化的方向向量,和这个点的反投影射线重合
            // 正投影方程 u= fx(X/Z)+cx v = fy(Y/Z)+cy
            // 这里首先把光心作为原点，进行反投影，求射线
            // 不考虑深度问题
            // X/Z=(u-cx)*invfx, Y/Z=(v-cy)*invfz;
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            //射线转到世界坐标系下
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;

            //求两射线的夹角的cos值
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
            float cosParallaxStereo = cosParallaxRays+1;
            // cosParallaxStereo1、cosParallaxStereo2在后面可能不存在，需要初始化为较大的值
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            //这里取近似值，将双视图近似当成一个等腰三角形
            //这样atan2(b/2, d)就是视差角的一半
            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            // 得到双目观测的视差角中最小的那个
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            //三角化恢复三维点
            cv::Mat x3D;
            // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视差角正常,0.9998 对应1°
            // cosParallaxRays < cosParallaxStereo 表明匹配点对夹角大于双目本身观察三维点夹角
            // 匹配点对夹角大，用三角法恢复3D点
            // 参考：https://github.com/raulmur/ORB_SLAM2/issues/345
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                //匹配的视角更大，角越大，cos越小
                // Linear Triangulation Method
                // 参考https://zhuanlan.zhihu.com/p/63179478
                // 这里射线与特征点等价，因为都是归一化坐标
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                //归一化前的检查
                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                // 归一化3D点
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            // 匹配点对夹角小，用双目恢复3D点
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            //为了方便计算转换为行向量
            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            // 生成的三维点，转换成相机坐标，检查是否在相机的正前方
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            //计算重投影误差
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];

            //转换到相机坐标系下
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                //单目情况
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                //卡方检验，2个自由度的卡方检验阀值是5.991
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                //双目情况
                //双目为什么只算u的误差，已经极线对准了，说以右图的v方向的误差跟左图一样
                //那为什么不是双倍？？？
                float u1 = fx1*x1*invz1+cx1;
                //视差公式
                //特征点坐标ul， ur
                //则根据三角形相识  b/Z = 【b-(ul-0.5x)-(0.5x-ur)】/(Z-f)=>Z = fb/d d = ul-ur
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            // 检查尺度连续性
            // 在不同比例下的金字塔的距离比例

            //世界坐标系下，由光心指向3D点
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            // ratioDist是不考虑金字塔尺度下的距离比例
            const float ratioDist = dist2/dist1;
            // 金字塔尺度因子的比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            // 距离的比例和图像金字塔的比例不应该差太多，否则就跳过
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            //添加观察到该地图点的关键帧
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            //计算该地图点的描述子
            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();  //创建新的地图点的时候

            //添加地图点
            mpMap->AddMapPoint(pMP);

            //添加到队列准被检验
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}


/**
 * @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的地图点
 *  Step 1：获得当前关键帧在共视图中权重排名前nn的邻接关键帧
 *  Step 2：存储一级相邻关键帧及其二级相邻关键帧
 *  Step 3：将当前帧的地图点分别投影到两级相邻关键帧，寻找匹配点对应的地图点进行融合，称为正向投影融合
 *  Step 4：将两级相邻关键帧地图点分别投影到当前关键帧，寻找匹配点对应的地图点进行融合，称为反向投影融合
 *  Step 4.1：遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpFuseCandidates
 *  Step 4.2：进行地图点投影融合,和正向融合操作是完全相同的,进行反向投影融合
 *  Step 5：更新当前帧地图点的描述子、深度、平均观测方向等属性
 *  Step 6：更新当前帧与其它帧的共视连接关系
 */


void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    // 获得当前关键帧在共视图中权重排名前nn的邻接关键帧
    // 开始之前先定义几个概念
    // 当前关键帧的邻接关键帧，称为一级相邻关键帧，也就是邻居
    // 与一级相邻关键帧相邻的关键帧，称为二级相邻关键帧，也就是邻居的邻居

    // 单目情况要20个邻接关键帧，双目或者RGBD则要10个
    int nn = 10;
    if(mbMonocular)
        nn=20;

    // 和当前关键帧共视的关键帧，也就是一级关键帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    // 一级关键帧和二级关键帧用于融合地图点
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        //没有跟当前关键帧进行融合过的
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        //加入一级相邻关键帧
        vpTargetKFs.push_back(pKFi);
        //并且标记已经融合过
        //为什么不用一个数组？？？只跟一帧进行融合???
        //因为是按顺序的，前面都已经融合过了
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        //添加二级关键帧
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    // 使用默认参数, 最优和次优比例0.6,匹配时检查特征点的旋转
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    // 将当前帧的地图点分别投影到两级相邻关键帧，寻找匹配点对应的地图点进行融合，称为正向投影融合
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        // 将地图点投影到关键帧中进行匹配和融合；融合策略如下
        // 1.如果地图点能匹配关键帧的特征点，并且该点有对应的地图点，那么选择观测数目多的替换两个地图点
        // 2.如果地图点能匹配关键帧的特征点，并且该点没有对应的地图点，那么为该点添加该投影地图点
        // 注意这个时候对地图点融合的操作是立即生效的
        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    // 将两级相邻关键帧地图点分别投影到当前关键帧，寻找匹配点对应的地图点进行融合，称为反向投影融合
    // 用于进行存储要融合的一级邻接和二级邻接关键帧所有MapPoints的集合
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    // 遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpFuseCandidates
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }
    // 进行地图点投影融合,和正向融合操作是完全相同的
    // 不同的是正向操作是"每个关键帧和当前关键帧的地图点进行融合",而这里的是"当前关键帧和所有邻接关键帧的地图点进行融合"
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    // 更新地图点的描述子，深度，平均观测方向等属性
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 在所有找到pMP的关键帧中，获得最佳的描述子
                pMP->ComputeDistinctiveDescriptors();

                // 更新平均观测方向和观测距离
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    //F=K.t.inv*t^*R*K.inv
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
    // 关键帧1的坐标 R1w*Ow + t1w
    // 关键帧2的坐标 R2w*Ow + t2w
    // 则从关键帧2算关键帧1的坐标 R1w*Ow + t1w = R12*(R2w*Ow + t2w) + t12;
    // t12 = t1w - R12*t2w
    // t12 = -R1w*R2w.t()*t2w+t1w

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

/**
 * @brief 检测当前关键帧在共视图中的关键帧，根据地图点在共视图中的冗余程度剔除该共视关键帧
 * 冗余关键帧的判定：90%以上的地图点能被其他关键帧（至少3个）观测到
 */

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    // 该函数里变量层层深入，这里列一下：
    // mpCurrentKeyFrame：当前关键帧，本程序就是判断它是否需要删除
    // pKF： mpCurrentKeyFrame的某一个共视关键帧
    // vpMapPoints：pKF对应的所有地图点
    // pMP：vpMapPoints中的某个地图点
    // observations：所有能观测到pMP的关键帧
    // pKFi：observations中的某个关键帧
    // scaleLeveli：pKFi的金字塔尺度
    // scaleLevel：pKF的金字塔尺度

    // 提取本帧的共视关键帧
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 对所有的共视关键帧进行遍历
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        //第一个关键帧不能删除
        if(pKF->mnId==0)
            continue;
        //提取当前共视关键帧的地图点
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        // 记录某个点被观测次数，后面并未使用
        int nObs = 3;
        // 观测次数阀值，默认为3
        const int thObs=nObs;
        // 记录冗余观测点的数目
        int nRedundantObservations=0;

        int nMPs=0;

        // 遍历该共视关键帧的所有地图点，其中能被其它至少3个关键帧观测到的地图点为冗余地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        // 对于双目或RGB-D，仅考虑近处（不超过基线的40倍 ）的地图点
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    // pMP->Observations() 是观测到该地图点的相机总数目（单目1，双目2）
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        // Observation存储的是可以看到该地图点的所有关键帧的集合
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        // 记录下观测到该点的帧数
                        int nObs=0;
                        // 遍历观测到该地图点的关键帧
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            // 尺度约束：为什么pKF 尺度+1 要大于等于 pKFi 尺度？
                            // 回答：因为同样或更低金字塔层级的地图点更准确
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                // 已经找到3个满足条件的关键帧，就停止不找了
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        // 地图点至少被3个关键帧观测到，就记录为冗余点，更新冗余点计数数目
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  
        //如果该关键帧的地图点，90%都能被其他关键帧观察到，则该帧为冗余关键帧
        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
