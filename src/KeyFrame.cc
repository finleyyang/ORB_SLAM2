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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

//关键帧的构造函数
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    // 获取id
    mnId=nNextId++;

    // 根据指定的普通帧, 初始化用于加速匹配的网格对象信息; 其实就把每个网格中有的特征点的索引复制过来
    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

// Bag of Words Representation 计算词袋表示
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

// 设置当前关键帧的位姿
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    // Ow：    当前相机光心在世界坐标系下坐标
    // Tcw：   世界坐标系到相机坐标系的变换矩阵
    // Rcw：   世界坐标系到相机坐标系的旋转矩阵
    // tcw：   世界坐标系到相机坐标系的平移向量
    // Rwc：   相机坐标系到世界坐标系的旋转矩阵
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    // 计算当前位姿的逆
    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));

    // center为相机坐标系（左目）下，立体相机中心的坐标
    // 立体相机中心点坐标与左目相机坐标之间只是在x轴上相差mHalfBaseline,
    // 因此可以看出，立体相机中两个摄像头的连线为x轴，正方向为左目相机指向右目相机 (齐次坐标)
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    // 世界坐标系下，左目相机中心到立体相机中心的向量，方向由左目相机指向立体相机中心
    Cw = Twc*center;
}

// 获取位姿
cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

// 获取位姿的逆
cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

// 获取(左目)相机的中心在世界坐标系下的坐标
cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

// 获取双目相机的中心,没找到在那用
cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

// 获取姿态
cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

// 获取位置
cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

/**
 * @brief 为当前关键帧新建或更新和其他关键帧的连接权重, 共视关键帧
 *
 * @param[in] pKF       和当前关键帧共视的其他关键帧
 * @param[in] weight    当前关键帧和其他关键帧的权重（共视地图点数目）
 */
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    //1.改变变量mConnectedKeyFrameWeights
    {
        // 互斥锁，防止同时操作共享数据产生冲突
        unique_lock<mutex> lock(mMutexConnections);
        // 新建或更新连接权重
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    //2.调用函数UpdateBestCovisibles()修改变量mvpOrderedConnectedKeyFrame和mvOrderdeweights
    // 连接关系变化就要更新最佳共视，主要是重新进行排序
    UpdateBestCovisibles();
}

/**
 * @brief 按照权重从大到小对连接（共视）的关键帧进行排序
 *
 * 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
 */
void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    //取出所有关键帧进行排序，排序结果存入变量mvpOrderedConnectedKeyFrames和mvOrderedWeights中
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    // 取出所有连接的关键帧，mConnectedKeyFrameWeights的类型为std::map<KeyFrame*,int>，而vPairs变量将共视的地图点数放在前面，利于排序
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    //对所有数据直接进行排序，是否可以分添加和删除，然后插入排序？
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;   // 所有连接关键帧
    list<int> lWs;          // 所有连接关键帧对应的权重（共视地图点数目）
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    // 权重从大到小排列的连接关键帧
    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    // 从大到小排列的权重，和mvpOrderedConnectedKeyFrames一一对应
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

// 得到与该关键帧连接（>15个共视地图点）的关键帧(没有排序的)
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

// 得到与该关键帧连接的关键帧(已按权值排序)
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

/**
 * @brief 得到与该关键帧连接的前N个最强共视关键帧(已按权值排序)
 *
 * @param[in] N                 设定要取出的关键帧数目
 * @return vector<KeyFrame*>    满足权重条件的关键帧集合
 */
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        // 如果总数不够，就返回所有的关键帧
        return mvpOrderedConnectedKeyFrames;
    else
        // 取前N个最强共视关键帧
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

/**
 * @brief 得到与该关键帧连接的权重超过w的关键帧
 *
 * @param[in] w                 权重阈值
 * @return vector<KeyFrame*>    满足权重条件的关键帧向量
 */
vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    // 如果没有和当前关键帧连接的关键帧，直接返回空
    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    // 从mvOrderedWeights找出第一个大于w的那个迭代器
    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    // 如果没有找到，说明最大的权重也比给定的阈值小，返回空
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        // 如果存在，返回满足要求的关键帧
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

// 得到该关键帧与pKF的权重
int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

// Add MapPoint to KeyFrame
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    //该帧的第几个特征点是哪一个地图点
    mvpMapPoints[idx]=pMP;
}

/**
 * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,将该地图点置为NULL
 *
 * @param[in] idx   地图点在该关键帧中的id
 */
void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    // NOTE 使用这种方式表示其中的某个地图点被删除
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

/**
 * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,将该地图点置为NULL
 *
 * @param[in] pMP   某地图点
 */
void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    //获取当前地图点在某个关键帧的观测中，对应的特征点的索引，如果没有观测，索引为-1
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

// 地图点的替换
void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}


// 获取当前关键帧中的所有地图点
set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        // 判断是否被删除了
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        // 如果是没有来得及删除的坏点也要进行这一步
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

// 关键帧中，大于等于最少观测数目minObs的MapPoints的数量.这些特征点被认为追踪到了
int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    // 是否检查数目
    const bool bCheckObs = minObs>0;
    // N是当前帧中特征点的个数
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)     //没有被删除
        {
            if(!pMP->isBad())  //并且不是坏点
            {
                if(bCheckObs)
                {
                    // 满足输入阈值要求的地图点计数加1
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

// 获取当前关键帧的具体的地图点
vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

// 获取当前关键帧的具体的某个地图点
MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}


// 更新共视关系
/*
 * 更新关键帧之间的连接图
 *
 * 1. 首先获得该关键帧的所有MapPoint点，统计观测到这些3d点的每个关键帧与其它所有关键帧之间的共视程度
 *    对每一个找到的关键帧，建立一条边，边的权重是该关键帧与当前关键帧公共3d点的个数。
 * 2. 并且该权重必须大于一个阈值，如果没有超过该阈值的权重，那么就只保留权重最大的边（与其它关键帧的共视程度比较高）
 * 3. 对这些连接按照权重从大到小进行排序，以方便将来的处理
 *    更新完covisibility图之后，如果没有初始化过，则初始化为连接权重最大的边（与其它关键帧共视程度最高的那个关键帧），类似于最大生成树
 */
void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;  //关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数

    //1.通过遍历当前地图点获取与其他关键帧的共视程度，存入变量KFcounter中
    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    //对该关键帧所有地图点进行遍历,然后对于含有该地图点的所有关键帧进行遍历累加
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations(); //当前地图点在某KeyFrame中的索引   KY流 key是keyframe value是size_t整数，当前地图点在该keyframe中的索引

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)  //与当前帧不算共视
                continue;
            // map[key] = value, 当要插入的键存在，会覆盖原来的值，如果不存在，则添加一组键值对
            // mit->first 是地图点对应的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
            // 所以最后KFcounter是第一个参数表示与当前帧的共视关键帧，第二个参数表示该帧看到了关键帧多少个地图点，也就是共视程度
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    //2.找到与当前关键帧共视程度超过15(即有15个关键帧与之共视)的关键帧，存入变量vPairs中
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    // vPairs 第一位是共视点的数目，第二位是关键帧
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        // 建立共视关系至少需要大于等于th个共视地图点
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            // 对方关键帧也要知道这个信息
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其他KeyFrame的mConnectedKeyFrameWeights
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    // 如果没有连接到关键帧(超过阀值的权重)，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
        // 如果每个关键帧与它共视的关键帧的个数都少于th，
        // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }
    // 对关键帧按照共视权重排序，存入变量mvpOrderedConnectedKeyFrames和mvOrderedWe
    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        // push_front从大到小排列
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        // 更新当前帧与其他关键帧的连接权重
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        // 对于第一次加入生成树的关键帧，取共视程度最高的关键帧为父关键帧

        // 最早生成树的连接关系
        if(mbFirstConnection && mnId!=0)
        {
            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
            mpParent = mvpOrderedConnectedKeyFrames.front();
            // 建立双向连接关系，将当前关键帧作为其子关键帧
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

// 添加子关键帧（即和子关键帧具有最大共视关系的关键帧就是当前关键帧）
void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

// 删除某个子关键帧
void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

// 改变当前关键帧的父关键帧
void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    //pKF为该节点的父节点
    unique_lock<mutex> lockCon(mMutexConnections);
    //该节点认pKF为父节点
    mpParent = pKF;
    //pKF添加该节点作为子节点
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        //1.先把mbNotErase设置为false，即现在该关键帧已经参于过回环检测
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }
    //2.检查之前是否因为mbNotErase为true,导致该帧需要删除却没有被删除，如果是的话，就删除该帧
    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

//当一帧keyframe被删除的时候，需要用加边法来构建最小生成树
//当该帧的父节点被删除的时候，该帧，只能以被删除节点的父节点（即该帧的爷爷节点）和被删除节点的子节点（即该帧的兄弟节点）作为父节点的候选点
//先将爷爷节点作为父节点集合中，然后把爷爷节点与所有子节点进行共视图权重比较，然后选取当前权重最高的一个点，与爷爷节点构成父子连接关系，然后把该点放到父节点的集合里
//然后再将剩下的与父节点集合里的两个节点进行共视图权重比较，然后再选取当前权重较高的一对关系，然后把两个点构成父子连接关系，然后再把该点放到父节点的集合里
//然后循环

/**
 * @brief 真正地执行删除关键帧的操作
 * 需要删除的是该关键帧和其他所有帧、地图点之间的连接关系
 *
 * mbNotErase作用：表示要删除该关键帧及其连接关系但是这个关键帧有可能正在回环检测或者计算sim3操作，这时候虽然这个关键帧冗余，但是却不能删除，
 * 仅设置mbNotErase为true，这时候调用setbadflag函数时，不会将这个关键帧删除，只会把mbTobeErase变成true，代表这个关键帧可以删除但不到时候,先记下来以后处理。
 * 在闭环线程里调用 SetErase()会根据mbToBeErased 来删除之前可以删除还没删除的帧。
 *
 */

void KeyFrame::SetBadFlag()
{
    //1.删除过程中，如果该帧参与回环优化，则该帧有权力不被删除
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }
    //2.从共视关键帧的共视图中删除此关键帧
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    //3.删除当前关键帧中地图对于本帧的观测
    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        //4.删除共视关系
        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();


        // Update Spanning Tree
        //5.更新生成树结构
        // 候选父关键帧
        set<KeyFrame*> sParentCandidates;  // 当前可以选作为父节点的节点的集合
        // 将当前帧的父关键帧放入候选父关键帧
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        // 每迭代一次就为其中一个子关键帧寻找父关键帧（最高共视程度），找到父的子关键帧可以作为其他子关键帧的候选父关键帧
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            // 遍历每一个子关键帧，让它们更新它们指向的父关键帧
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;//选取一帧作为操作对象
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();   //选取的子节点的所有共视关系的帧按照顺序排列
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    //然后选取所有当前可以作为父节点的节点
                    // sParentCandidates 中刚开始存的是这里子关键帧的“爷爷”，也是当前关键帧的候选父关键帧
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        //当选取的子节点的共视关系关键帧，与父节点集合中的节点相同, 即它的共视关键帧与它的爷爷帧一样
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            //则取该子节点，与共视图里和父节点集合里面相同的关键帧的共视权重
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF; // 然后把该子节点赋给pC
                                pP = vpConnected[i];// 该子节点的父节点赋给pP
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                //pP为pC的父节点
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        // 如果该子节点没有一个有共视关系存在，则该节点的父节点作为该子节点的父节点（即该子节点的爷爷节点作为该子节点的父节点）
        // 如果还有子节点没有找到新的父节点
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                // 直接把父节点的父节点作为自己的父节点 即对于这些子节点来说,他们的新的父节点其实就是自己的爷爷节点
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        // mTcp 表示原父关键帧到当前关键帧的位姿变换，在保存位姿的时候使用
        mTcp = Tcw*mpParent->GetPoseInverse();
        // 标记当前关键帧已经挂了
        mbBad = true;
    }

    // 地图和关键帧数据库中删除该关键帧
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

// 删除当前关键帧和指定关键帧之间的共视关系
void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    //1.修改变量mConnectedKeyFrameWeights
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    //2.调用函数UpdateBestCovisibles()修改变量mvpOrderedConnectedKeyFrame和mvOrderedWeights
    if(bUpdate)
        UpdateBestCovisibles();
}

// 获取某个特征点的邻域中的特征点id,其实这个和 Frame.cc 中的那个函数基本上都是一致的; r为边长（半径）
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    //储存搜索结果的vector
    vector<size_t> vIndices;
    vIndices.reserve(N);

    // Step 1 计算半径为r圆左右上下边界所在的网格列和行的id
    // 查找半径为r的圆左侧边界所在网格列坐标。这个地方有点绕，慢慢理解下：
    // (mnMaxX-mnMinX)/FRAME_GRID_COLS：表示列方向每个网格可以平均分得几个像素（肯定大于1）
    // mfGridElementWidthInv=FRAME_GRID_COLS/(mnMaxX-mnMinX) 是上面倒数，表示每个像素可以均分几个网格列（肯定小于1）
    // (x-mnMinX-r)，可以看做是从图像的左边界mnMinX到半径r的圆的左边界区域占的像素列数
    // x-mnMinX-r可以写成x-r-mnMinX
    // 两者相乘，就是求出那个半径为r的圆的左侧边界在哪个网格列中
    // 保证nMinCellX 结果大于等于0
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));

    // 如果最终求得的圆的左边界所在的网格列超过了设定了上限，那么就说明计算出错，找不到符合要求的特征点，返回空vector
    //这里mnGridCols等于FRAME_GRID_COLS
    if(nMinCellX>=mnGridCols)
        return vIndices;

    // 计算圆所在的右边界网格列索引
    // x-mnMinX+r可以看成x+r-mnMinX
    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    // 同理可求得上下边界所在的网格的iD
    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    //遍历圆形区域内的所有网格，寻找满足条件的候选特征点，并将其index放到输出里
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

// 判断某个点是否在当前关键帧的图像中
bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

/**
 * @brief 在双目和RGBD情况下将特征点反投影到空间中得到世界坐标系下三维点
 *
 * @param[in] i                 第i个特征点
 * @return cv::Mat              返回世界坐标系下三维点
 */
cv::Mat KeyFrame::UnprojectStereo(int i)
{
    // 双目，求深度
    const float z = mvDepth[i];
    if(z>0)
    {
        // 由2维图像反投影到相机坐标系
        // 双目中mvDepth是在ComputeStereoMatches函数中求取的，rgbd中是直接测量的
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        // 由相机坐标系转换到世界坐标系
        // Twc为相机坐标系到世界坐标系的变换矩阵
        // Twc.rosRange(0,3).colRange(0,3)取Twc矩阵的前3行与前3列
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

// 其实过程就是对当前关键帧下所有地图点的深度进行从小到大排序,返回距离头部其中1/q处的深度值作为当前场景的平均深度
float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    //相机的位姿
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    // 遍历每一个地图点,计算并保存其在当前关键帧下的深度
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;   // (R*x3Dw+t)的第三行，即z
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
