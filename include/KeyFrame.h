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
    /**
    * @brief 构造函数
    * @param[in] F         父类普通帧的对象
    * @param[in] pMap      所属的地图指针
    * @param[in] pKFDB     使用的词袋模型的指针
    */
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    // 这里的set,get需要用到锁

    /**
     * @brief 设置当前关键帧的位姿
     * @param[in] Tcw 位姿
     */
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    /**
    * @brief Bag of Words Representation
    * @detials 计算mBowVec，并且将描述子分散在第4层上，即mFeatVec记录了属于第i个node的ni个描述子
    * @see ProcessNewKeyFrame()
    */
    void ComputeBoW();

    // Covisibility graph functions
    /**
    * @brief 为关键帧之间添加连接
    * @details 更新了mConnectedKeyFrameWeights
    * @param pKF    关键帧
    * @param weight 权重，该关键帧与pKF共同观测到的3d点数量
    */
    void AddConnection(KeyFrame* pKF, const int &weight);   //添加共视关键帧
    /**
    * @brief 删除当前关键帧和指定关键帧之间的共视关系
    * @param[in] pKF 要删除的共视关系
    */
    void EraseConnection(KeyFrame* pKF);             //删除共视关键帧
    /** @brief 更新图的连接  */
    void UpdateConnections();                   //基于当前观测帧对地图点的观测构成共视图
    //只要关键帧与地图点间的连接关系发生变换(包括关键帧创建和地图点重新匹配关键帧特征点)函数UpdateConnections()就会被调用
    //Tracking线程中创建关键帧的时候会调用UpdateConnections()初始化共视图
    //Localmapping线程接收到新关键帧时会调用函数LocalMapping::ProcessNewKeyFrame()处理跟踪过程中加入地图点之后会用KeyFrame::updateConnections()初始化共视图信息(关键帧和地图点重新匹配)
    //(实际上这里是处理Tracking线程中函数Tracking::CreateNewKeyFrame()创建的关键帧)
    //LocalMapping线程处理完毕缓冲队列内所有关键帧后会调用LocalMapping::SearchInNeighbours()融合当前关键帧和共视关键帧间的重复地图点之后会调用KeyFrame::UpdateConnects()更新共视图信息
    //LoopClosing线程闭环矫正函数LoopCLosing::CorrectLoop()会多次调用KeyFrame::UpdateConnections()更新共视图信息

    /**
     * @brief 按照权重对连接的关键帧进行排序
     * @detials 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
     */
    void UpdateBestCovisibles();             //基于共视图信息修改对应变量
     //get方法
     /**
      * @brief 得到与该关键帧连接的关键帧(没有排序的)
      * @return 连接的关键帧
      */
    std::set<KeyFrame *> GetConnectedKeyFrames();

    /**
    * @brief 得到与该关键帧连接的关键帧(已按权值排序)
    * @return 连接的关键帧
    */
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();

    /**
    * @brief 得到与该关键帧连接的前N个关键帧(已按权值排序)
    * NOTICE 如果连接的关键帧少于N，则返回所有连接的关键帧,所以说返回的关键帧的数目其实不一定是N个
    * @param N 前N个
    * @return 连接的关键帧
    */
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);

    /**
    * @brief 得到与该关键帧连接的权重大于等于w的关键帧
    * @param w 权重
    * @return 连接的关键帧
    */
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

    /**
    * @brief 得到该关键帧与pKF的权重
    * @param  pKF 关键帧
    * @return     权重
    */
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    //关键帧增加到生成树中的时机
    //成功创建关键帧之后会调用KeyFrame::UpdateConnections()，该函数第一次被调用时会将该关键帧加入到生成树中
    //共视图的改变（除了删除关键帧以外）不会导致生成树的改变
    //当某个关键帧被删除的时候，与其相连接的结构会发生改变，生成树的结构改变类似最小生成树中的加边法
    /**
    * @brief 添加子关键帧（即和子关键帧具有最大共视关系的关键帧就是当前关键帧）
    * @param[in] pKF 子关键帧句柄
    */
    void AddChild(KeyFrame* pKF);                  //添加子节点，mpChildrens的set方法

    /**
    * @brief 删除某个子关键帧
    * @param[in] pKF 子关键帧句柄
    */
    void EraseChild(KeyFrame* pKF);                //删除子节点，mpChildrens的set方法

    /**
     * @brief 改变当前关键帧的父关键帧
     * @param[in] pKF 父关键帧句柄
     */
    void ChangeParent(KeyFrame* pKF);

    /**
     * @brief 获取获取当前关键帧的子关键帧
     * @return std::set<KeyFrame*>  子关键帧集合
     */
    std::set<KeyFrame*> GetChilds();

    /**
     * @brief 获取当前关键帧的父关键帧
     * @return KeyFrame* 父关键帧句柄
     */
    KeyFrame* GetParent();                        //mpParent的get方法
    /**
    * @brief 判断某个关键帧是否是当前关键帧的子关键帧
    * @param[in] pKF 关键帧句柄
    * @return true
    * @return false
    */
    bool hasChild(KeyFrame* pKF);                //判断mpChilderns是否为空

    // Loop Edges
    /**
    * @brief 给当前关键帧添加回环边，回环边连接了形成闭环关系的关键帧
    * @param[in] pKF  和当前关键帧形成闭环关系的关键帧
    */
    void AddLoopEdge(KeyFrame* pKF);

    /**
     * @brief 获取和当前关键帧形成闭环关系的关键帧
     * @return std::set<KeyFrame*> 结果
     */
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    /**
     * @brief Add MapPoint to KeyFrame
     * @param pMP MapPoint
     * @param idx MapPoint在KeyFrame中的索引
     */
    void AddMapPoint(MapPoint* pMP, const size_t &idx);  //添加该帧的地图点

    /**
     * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,这里是"通知"当前关键帧这个地图点已经被删除了
     * @param[in] idx 被删除的地图点索引
     */
    void EraseMapPointMatch(const size_t &idx);      //删除该帧第几个地图点

    /**
     * @brief 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,这里是"通知"当前关键帧这个地图点已经被删除了
     * @param[in] pMP 被删除的地图点指针
     */
    void EraseMapPointMatch(MapPoint* pMP);          //删除该帧哪一个地图点

    /**
    * @brief 地图点的替换
    * @param[in] idx 要替换掉的地图点的索引
    * @param[in] pMP 新地图点的指针
    */
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);  //用该地图点替换该帧第几个地图点

    /**
     * @brief 获取当前帧中的所有地图点
     * @return std::set<MapPoint*> 所有的地图点
     */
    std::set<MapPoint*> GetMapPoints();                 //地图点的get方法（set类）

    /**
     * @brief Get MapPoint Matches 获取该关键帧的MapPoints
     */
    std::vector<MapPoint*> GetMapPointMatches();        //地图点的get方法（vector类）

    /**
     * @brief 关键帧中，大于等于minObs的MapPoints的数量
     * @details minObs就是一个阈值，大于minObs就表示该MapPoint是一个高质量的MapPoint \n
     * 一个高质量的MapPoint会被多个KeyFrame观测到.
     * @param  minObs 最小观测
     */
    int TrackedMapPoints(const int &minObs);

    /**
     * @brief 获取获取当前关键帧的具体的某个地图点
     * @param[in] idx id
     * @return MapPoint* 地图点句柄
     */
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
    /**
     * @brief 获取某个特征点的邻域中的特征点id
     * @param[in] x 特征点坐标
     * @param[in] y 特征点坐标
     * @param[in] r 邻域大小(半径)
     * @return std::vector<size_t> 在这个邻域内找到的特征点索引的集合
     */
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

    /**
     * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
     * @param  i 第i个keypoint
     * @return   3D点（相对于世界坐标系）
     */
    cv::Mat UnprojectStereo(int i);

    // Image
    /**
     * @brief 判断某个点是否在当前关键帧的图像中
     * @param[in] x 点的坐标
     * @param[in] y 点的坐标
     * @return true
     * @return false
     */
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    //若某关键帧参与了回环检测，loopclosing线程就会调用函数KeyFrame::SetNotErase()将该关键帧的成员变量mbNotErase设为true，标记下该关键帧暂时就不要被删除了
    /** @brief 设置当前关键帧不要在优化的过程中被删除  */
    void SetNotErase();        //mbNotErase的set方法
    //成员变量mbToBeErased标记当前KeyFrame是否曾有权力不被删除。LoopClosing线程不再需要某关键帧时，会调用函数KeyFrame::SetErase()剥夺
    //该关键帧不被删除的权力，将成员变量mbNotErase复位为true，若是成员变量mbToErased为true，就会调用函数KeyFrame::SetBadFlag()删除该关键帧
    /** @brief 准备删除当前的这个关键帧,表示不进行回环检测过程;由回环检测线程调用 */
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();        //真正执行删除的函数
    bool isBad();           //mbBad的get方法

    // Compute Scene Depth (q=2 median). Used in monocular.
    /**
    * @brief 评估当前关键帧场景深度，q=2表示中值
    * @param q q=2
    * @return Median Depth
    */
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    // nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号
    static long unsigned int nNextId;
    // 在nNextID的基础上加1就得到了mnID，为当前KeyFrame的ID号
    long unsigned int mnId;
    // 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，
    // mnFrameId记录了该KeyFrame是由哪个Frame初始化的
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    // 每个网格的宽度和高度,以及他们的倒数
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame; //记录它是哪一帧得关键依赖帧
    long unsigned int mnFuseTargetForKF; // 标记在局部建图线程中,和哪个关键帧进行融合的操作

    // Variables used by the local mapping
    // local mapping中记录当前处理的关键帧的mnId，表示当前局部BA的关键帧id。mnBALocalForKF 在map point.h里面也有同名的变量。
    long unsigned int mnBALocalForKF;
    // local mapping中记录当前处理的关键帧的mnId, 只是提供约束信息但是却不会去优化这个关键帧
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database 下面的这些变量都是临时的,由外部调用暂时存放一些数据
    // 标记了当前关键帧是id为mnLoopQuery的回环检测的候选关键帧，记录作为某一帧帧的回环检查关键帧
    long unsigned int mnLoopQuery;
    // 当前关键帧和这个形成回环的候选关键帧中,具有相同word的个数
    int mnLoopWords;
    // 和那个形成回环的关键帧的词袋匹配程度的评分
    float mLoopScore;

    // 记录重定位信息的参数
    // 用来存储在辅助进行重定位的时候，要进行重定位的那个帧的id
    long unsigned int mnRelocQuery;
    // 和那个要进行重定位的帧,所具有相同的单词的个数
    int mnRelocWords;
    // 还有和那个帧的词袋的相似程度的评分
    float mRelocScore;

    // Variables used by loop closing
    // 经过全局BA优化后的相机的位姿
    cv::Mat mTcwGBA;
    // 进行全局BA优化之前的当前关键帧的位姿. 之所以要记录这个是因为在全局优化之后还要根据该关键帧在优化之前的位姿来更新地图点,which地图点的参考关键帧就是该关键帧
    cv::Mat mTcwBefGBA;
    // 记录是由于哪个"当前关键帧"触发的全局BA,用来防止重复写入的事情发生(浪费时间)
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
    // Vector of words to represent images
    // mBowVec 内部实际存储的是std::map<WordId, WordValue>
    // WordId 和 WordValue 表示Word在叶子中的id 和权重
    DBoW2::BowVector mBowVec;

    // Vector of nodes with indexes of local features
    // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
    // NodeId 表示节点id，std::vector<unsigned int> 中实际存的是该节点id下所有特征点在图像中的索引
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;  // 尺度因子，scale^n，scale=1.2，n为层数
    const std::vector<float> mvLevelSigma2;  // 尺度因子的平方
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
