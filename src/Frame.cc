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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{
//下一个生成的帧的ID,这里是初始化类的静态成员变量
long unsigned int Frame::nNextId=0;
//是否要进行初始化操作的标志
//这里给这个标志置位的操作是在最初系统开始加载到内存的时候进行的，下一帧就是整个系统的第一帧，所以这个标志要置位
bool Frame::mbInitialComputations=true;

float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    //逐个复制，其实这里也是深拷贝
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
        //这里没有使用前面的深拷贝方式的原因可能是mGrid是由若干vector类型对象组成的vector，
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        //这里说的是给新的帧设置Pose
        SetPose(frame.mTcw);
}

//双目模型
//frame类的两个ORBextractor是在调用构造函数的时候传入的，构造函数中调用ExtractORB（）提取特征点
/**
 * @brief 为双目相机准备的构造函数
 *
 * @param[in] imLeft            左目图像
 * @param[in] imRight           右目图像
 * @param[in] timeStamp         时间戳
 * @param[in] extractorLeft     左目图像特征点提取器句柄
 * @param[in] extractorRight    右目图像特征点提取器句柄
 * @param[in] voc               ORB字典句柄
 * @param[in] K                 相机内参矩阵
 * @param[in] distCoef          相机去畸变参数
 * @param[in] bf                相机基线长度和焦距的乘积
 * @param[in] thDepth           远点和近点的深度区分阈值
 *
 */
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    //0.帧ID自增
    mnId=nNextId++;

    // Scale Level Info
    //1.计算金字塔参数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    //特征点是采用子线程
    //2.提取双目图像特征点
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    //3.畸变矫正
    // 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
    // 实际上由于双目输入的图像已经预先经过矫正,所以实际上并没有对特征点进行任何处理操作
    UndistortKeyPoints();
    //4.双目特征点匹配
    // 计算双目间特征点的匹配，只有匹配成功的特征点会计算其深度,深度存放在 mvDepth
    // mvuRight中存储的应该是左图像中的点所匹配的在右图像中的点的横坐标（纵坐标相同）
    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL)); //初始化本身地图点
    mvbOutlier = vector<bool>(N,false);  //标记当前帧的地图点不是外点


    // This is done only for the first Frame (or after a change in the calibration)
    //5.第一次调用构造函数计算static变量
    if(mbInitialComputations)
    {
        //计算去畸变后的图像边界
        ComputeImageBounds(imLeft);

        //每一个小框的像素点数量的倒数
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;


        //计算完成
        mbInitialComputations=false;
    }

    mb = mbf/fx; //计算双目基线长度
    //6.特征点分配到网络中
    AssignFeaturesToGrid();
}

//RGB-D模型
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


//单目模型
/**
 * @brief 单目帧构造函数
 *
 * @param[in] imGray                            //灰度图
 * @param[in] timeStamp                         //时间戳
 * @param[in & out] extractor                   //ORB特征点提取器的句柄
 * @param[in] voc                               //ORB字典的句柄
 * @param[in] K                                 //相机的内参数矩阵
 * @param[in] distCoef                          //相机的去畸变参数
 * @param[in] bf                                //baseline*f
 * @param[in] thDepth                           //区分远近点的深度阈值
 */
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    // 帧的自增
    mnId=nNextId++;

    // Scale Level Info
    //配置文件的参数，配置好
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

/**
 * @brief 将提取的ORB特征点分配到图像网格中
 *
 */

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    //开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);
    //遍历特征点，将每个特征点索引加入到对应网格中
    for(int i=0;i<N;i++)
    {
        //从类的成员变量中获取已经去畸变后的特征点
        const cv::KeyPoint &kp = mvKeysUn[i];

        //存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS], nGridPosY范围：[0,FRAME_GRID_ROWS]
        int nGridPosX, nGridPosY;
        // 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            //记录每个网格中的特征点的数量
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    //判断是左图还是右图
    if(flag==0)
        //flag==0,表示对左图提取ORB特征点
        //左图的话就套用左图指定的特征点提取器，并将提取结果保存到对应的变量中
        //这里使用了仿函数来完成，重载了括号运算符 ORBextractor::operator()
        (*mpORBextractorLeft)(im,   //待提取特征点图像
                cv::Mat(),      //掩摸图像，实际并没有使用
                mvKeys,            //输出变量，用于保存提取后的特征点
                mDescriptors);//输出变量，用于保运特征点的描述子
    else
        //flag==1，表示对右图提取ORB特征点
        //右图的话就套用右图指定的特征点提取器，并将提取结果保存到对应的变量中
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
// 设置相机姿态
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

//根据Tcw计算mRcw、mtcw和mRwc、mOw
void Frame::UpdatePoseMatrices()
{
    // mOw：    当前相机光心在世界坐标系下坐标
    // mTcw：   世界坐标系到相机坐标系的变换矩阵
    // mRcw：   世界坐标系到相机坐标系的旋转矩阵
    // mtcw：   世界坐标系到相机坐标系的平移向量
    // mRwc：   相机坐标系到世界坐标系的旋转矩阵

    //从变换矩阵中提取出旋转矩阵
    //注意，rowRange这个只取到范围的左边界，而不取右边界
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    // mTcw 求逆后是当前相机坐标系变换到世界坐标系下，对应的光心变换到世界坐标系下就是 mTcw的逆 中对应的平移向量
    mOw = -mRcw.t()*mtcw;
}

/**
 * @brief 判断地图点是否在视野中
 * 步骤
 * Step 1 获得这个地图点的世界坐标，经过以下层层关卡的判断，通过的地图点才认为是在视野中
 * Step 2 关卡一：将这个地图点变换到当前帧的相机坐标系下，如果深度值为正才能继续下一步。
 * Step 3 关卡二：将地图点投影到当前帧的像素坐标，如果在图像有效范围内才能继续下一步。
 * Step 4 关卡三：计算地图点到相机中心的距离，如果在有效距离范围内才能继续下一步。
 * Step 5 关卡四：计算当前相机指向地图点向量和地图点的平均观测方向夹角，小于60°才能进入下一步。
 * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
 * Step 7 记录计算得到的一些参数
 * @param[in] pMP                       当前地图点
 * @param[in] viewingCosLimit           当前相机指向地图点向量和地图点的平均观测方向夹角余弦阈值
 * @return true                         地图点合格，且在视野内
 * @return false                        地图点不合格，抛弃
 */

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    // 获取特征点的世界坐标
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    // 转换成相机坐标下的位置
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    // 检查深度是否为正
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    // 转换成图像上的坐标，检查是否超过图像的范围
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    // 计算特征点是否在相机的有效视觉范围内
    // 特征点的有效范围，在每一次观察的时候，都已经更新了
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    // 计算当前相机指向地图点向量和地图点的平均观测方向夹角，小于60°才能进入下一步。
    cv::Mat Pn = pMP->GetNormal();

    // 计算视角差，cos值
    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    // 计算特征点的尺度
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

/**
 * @brief 找到在 以x,y为中心,半径为r的圆形内且金字塔层级在[minLevel, maxLevel]的特征点
 *
 * @param[in] x                     特征点坐标x
 * @param[in] y                     特征点坐标y
 * @param[in] r                     搜索半径
 * @param[in] minLevel              最小金字塔层级
 * @param[in] maxLevel              最大金字塔层级
 * @return vector<size_t>           返回搜索到的候选匹配点id
 */

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
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
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    // 计算圆所在的右边界网格列索引
    // x-mnMinX+r可以看成x+r-mnMinX
    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        // 如果计算出的圆右边界所在的网格不合法，说明该特征点不好，直接返回空vector
        return vIndices;

    // 同理可求得上下边界所在的网格的iD
    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    // 检查需要搜索的图像金字塔层数范围是否符合要求
    //? 疑似bug。(minLevel>0) 后面条件 (maxLevel>=0)肯定成立
    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    //遍历圆形区域内的所有网格，寻找满足条件的候选特征点，并将其index放到输出里
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            // 获取这个网格内的所有特征点在 Frame::mvKeysUn 中的索引
            //vCell 特征点的索引 mGrid 表示每个网格中的特征点索引
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    // cv::KeyPoint::octave中表示的是从金字塔的哪一层提取的数据
                    // 保证特征点是在金字塔层级minLevel和maxLevel之间，不是的话跳过
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                // 通过检查，计算候选特征点到圆中心的距离，查看是否是在这个圆形区域之内
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

/**
 * @brief 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
 *
 * @param[in] kp                    给定的特征点
 * @param[in & out] posX            特征点所在网格坐标的横坐标
 * @param[in & out] posY            特征点所在网格坐标的纵坐标
 * @return true                     如果找到特征点所在的网格坐标，返回true
 * @return false                    没找到返回false
 */

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    // x,y减去mnMinX和mnMinY是因为为了确定畸变后特征点的位置
    // mfGridElementWidthInv=(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
    // mfGridElementHeightInv=(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    // 因为特征点进行了去畸变，而且前面计算是round取整，所以有可能得到的点落在图像网格坐标外面
    // 如果网格坐标posX，posY超出了[0,FRAME_GRID_COLS] 和[0,FRAME_GRID_ROWS]，表示该特征点没有对应网格坐标，返回false
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

/**
 * @brief 计算当前帧特征点对应的词袋Bow，主要是mBowVec 和 mFeatVec
 *
 */
void Frame::ComputeBoW()
{
    // 判断是否以前已经计算过了，计算过了就跳过
    // mBowVec是空表示没有计算过
    if(mBowVec.empty())
    {
        // 将描述子mDescriptors转换为DBOW要求的输入格式
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // 将特征点的描述子转换成词袋向量mBowVec以及特征向量mFeatVec
        mpORBvocabulary->transform(vCurrentDesc,    //当前描述子的vector
                                   mBowVec,      //输出，词袋向量，记录的是单词的id及其对应权重TF-IDF值
                                   mFeatVec,    //输出，记录node id及其对应的图像 feature对应的索引
                                   4);      //4表示从叶节点向前数的层数
    }
}

void Frame::UndistortKeyPoints()
{   //1, 若输入图像是双目图像，则已经做好了双目矫正，其畸变参数为0
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    //2，将特征点坐标转为undistortPoints()函数要求的格式
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    //进行畸变矫正
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    //记录校正后的特征点
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

//通过计算图片顶尖畸变校正后的坐标来计算畸变矫正后的图片有效范围
/**
 * @brief 计算去畸变图像的边界
 *
 * @param[in] imLeft            需要计算边界的图像
 */
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    // 如果畸变参数不为0，用Opencv函数矫正畸变
    if(mDistCoef.at<float>(0)!=0.0)
    {
        //4个顶点坐标
        // 保存矫正前的图像四个边界点坐标： (0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;  //左上
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0; //右上
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;  //左下
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows; //右下

        // Undistort corners
        //畸变矫正
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);
        //记录图片有效范围
        //畸变矫正后图像不一定是一个标准的矩形
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));   //左上和左下横坐标
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));   //右上和右下横坐标
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));   //左上和右上纵坐标
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));   //左下和右下纵坐标

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

/*
 * 双目匹配函数
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n ‘
 * 这里所说的SAD是一种双目立体视觉匹配算法，可参考[https://blog.csdn.net/u012507022/article/details/51446891]
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 这里所谓的亚像素精度，就是使用这个拟合得到一个小于一个单位像素的修正量，这样可以取得更好的估计结果，计算出来的点的深度也就越准确
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */

void Frame::ComputeStereoMatches()
{
    /*两帧图像稀疏立体匹配（即：ORB特征点匹配，非逐像素的密集匹配，但依然满足行对齐）
     * 输入：两帧立体矫正后的图像img_left 和 img_right 对应的orb特征点集
     * 过程：
      1. 行特征点统计. 统计img_right每一行上的ORB特征点集，便于使用立体匹配思路(行搜索/极线搜索）进行同名点搜索, 避免逐像素的判断.
      2. 粗匹配. 根据步骤1的结果，对img_left第i行的orb特征点pi，在img_right的第i行上的orb特征点集中搜索相似orb特征点, 得到qi
      3. 精确匹配. 以点qi为中心，半径为r的范围内，进行块匹配（归一化SAD），进一步优化匹配结果
      4. 亚像素精度优化. 步骤3得到的视差为uchar/int类型精度，并不一定是真实视差，通过亚像素差值（抛物线插值)获取float精度的真实视差
      5. 最优视差值/深度选择. 通过胜者为王算法（WTA）获取最佳匹配点。
      6. 删除离群点(outliers). 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配
    * 输出：稀疏特征点视差图/深度图（亚像素精度）mvDepth 匹配结果 mvuRight
    */

    // 为匹配结果预先分配内存，数据类型为float型
    // mvuRight存储右图匹配点索引
    // mvDepth存储特征点的深度信息
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    // orb特征相似度阈值  -> mean ～= (max  + min) / 2
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    // 金字塔顶层（0层）图像高 nRows
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    //0.右目图像特征点逐行统计，将右目图像中每个特征点注册到附近几行
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    // 二维vector存储每一行的orb特征点的列坐标，为什么是vector，因为每一行的特征点有可能不一样，例如
    // vRowIndices[0] = [1，2，5，8, 11]   第1行有5个特征点,他们的列号（即x坐标）分别是1,2,5,8,11
    // vRowIndices[1] = [2，6，7，9, 13, 17, 20]  第2行有7个特征点.etc
    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    // 右图特征点数量，N表示数量 r表示右图，且不能被修改
    const int Nr = mvKeysRight.size();

    // 将右面的特征点的坐标近似找到左边对应的坐标
    for(int iR=0; iR<Nr; iR++)
    {
        // 获取特征点ir的y坐标，即行号
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算特征点ir在行方向上，可能的偏移范围r，即可能的行号为[kpY + r, kpY -r]
        // 表示在全尺寸(scale = 1)的情况下，假设有2个像素的偏移，随着尺度变化，r也跟着变化
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];

        // maxr和minr对应左图的位置，行的位置
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        // 将特征点ir保证在可能的行号中
        // yi是左图的行好,iR是右图特征点的编号
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }


    // 1和2，粗匹配加精匹配
    // Set limits for search
    // 粗匹配 + 精匹配
    // 对于立体矫正后的两张图，在列方向(x)存在最大视差maxd和最小视差mind
    // 也即是左图中任何一点p，在右图上的匹配点的范围为应该是[p - maxd, p - mind], 而不需要遍历每一行所有的像素
    // maxd = baseline * length_focal / minZ
    // mind = baseline * length_focal / maxZ
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;      //跟及视差公式计算两个特征点的最大深度

    // For each left keypoint search a match in the right image
    // 保存sad块匹配相似度和左图特征点索引
    vector<pair<int, int> > vDistIdx;  //保存特征点匹配
    vDistIdx.reserve(N);

    // 为左图每一个特征点il，在右图搜索最相似的特征点ir
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        //vCandidates是左图这个特征点对应的行数的集合
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        // 计算理论上的最佳搜索范围，根据视差来计算的
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        // 最大搜索范围小于0，说明无匹配点
        if(maxU<0)
            continue;

        //1.粗匹配，根据特征点描述子和金子塔层级进行粗匹配
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        // 左图特征点的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // 计算右图特征点的描述子
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 左图特征点il与带匹配点ic的空间尺度差超过2，放弃
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            // 超出理论搜索范围[minU, maxU]，可能是误匹配，放弃
            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);
                //计算汉明距离

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        //2.精匹配，滑动窗口匹配，根据匹配点周围5*5的窗口寻找精确匹配
        // 如果刚才匹配过程中的最佳描述子距离小于给定的阈值
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // 计算右图特征点x坐标和对应的金字塔尺度
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            // 尺度缩放后的左右图特征点坐标
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            //构建滑动窗口
            // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像快patch
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            // 图像块均值归一化，降低亮度变化对相似度计算的影响
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            //初始化最佳相似度
            // 标准头文件中 #define INT_MAX 2147483647
            int bestDist = INT_MAX;
            // 通过滑动窗口搜索优化，得到的列坐标偏移量
            int bestincR = 0;
            //滑动窗口的滑动范围为（-L, L）
            const int L = 5;
            // 初始化存储图像块相似度
            vector<float> vDists;
            vDists.resize(2*L+1);

            // 计算滑动窗口滑动范围的边界，因为是块匹配，还要算上图像块的尺寸
            // 列方向起点 iniu = r0 - 最大窗口滑动范围 - 图像块尺寸
            // 列方向终点 eniu = r0 + 最大窗口滑动范围 + 图像块尺寸 + 1
            // 此次 + 1 和下面的提取图像块是列坐标+1是一样的，保证提取的图像块的宽是2 * w + 1
            // ! 源码： const float iniu = scaleduR0+L-w; 错误
            const float iniu = scaleduR0-L-w;
            const float endu = scaleduR0+L+w+1;

            // 判断搜索是否越界
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            // 在搜索范围内从左到右滑动，并计算图像块相似度
            for(int incR=-L; incR<=+L; incR++)
            {
                // 提取左图中，以特征点(scaleduL,scaledvL)为中心, 半径为w的图像块patch
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                // 图像块均值归一化，降低亮度变化对相似度计算的影响
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                // sad 计算
                float dist = cv::norm(IL,IR,cv::NORM_L1);
                // 统计最小sad和偏移量
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                //L+incR 为refine后的匹配点列坐标(x)
                vDists[L+incR] = dist;
            }
            // 搜索窗口越界判断
            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            //3.亚像素插值，将特征点匹配距离拟合成二次曲线，寻找二次曲线最低点作为最优解
            // 亚像素插值, 使用最佳匹配点及其左右相邻点构成抛物线
            // 使用3点拟合抛物线的方式，用极小值代替之前计算的最优是差值
            //    \                 / <- 由视差为14，15，16的相似度拟合的抛物线
            //      .             .(16)
            //         .14     .(15) <- int/uchar最佳视差值
            //              .
            //           （14.5）<- 真实的视差值
            //   deltaR = 15.5 - 16 = -0.5

            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            // 亚像素精度的修正量应该是在[-1,1]之间，否则就是误匹配
            //  论文<<On Building an Accurate Stereo Matching System on Graphics Hardware>> 公式7
            // C(p, d+) - C(p, d-)/(2 * (C(p, d+) + C(p, d-) - 2 (C(p, d)))
            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            //4.记录特征点的右目和深度信息
            // 右目匹配的特征点实际横坐标
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            // 左右目的像素点的差值
            float disparity = (uL-bestuR);


            if(disparity>=minD && disparity<maxD)
            {
                // 如果存在负视差，则约束为0.01
                // 但是前面已经 minD = 0， 已经确定，不会小于零了，为什么还要判断？？？？
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                //解求深度信息，双目视差公式
                mvDepth[iL]=mbf/disparity;
                //记录右侧相机上特征点的坐标值
                mvuRight[iL] = bestuR;
                // 记录特征点的最佳距离，以及特征点在左图的索引
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }
    //5.删除离群点，匹配距离大于平均匹配距离2.1倍的视为误匹配。而且该处平均距离，是中值距离
    // 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是匹配的，比如光照变化、弱纹理、无纹理等同样会造成误匹配
    // 误匹配判断条件  norm_sad > 1.5 * 1.4 * median ？？？ 为什么以2.1倍作为阀值
    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    //大于阀值后，删除相对应的特征点
    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


//计算RGBD图像的立体深度信息
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    //初始化右目和深度信息
    /** 主要步骤如下:.对于彩色图像中的每一个特征点:<ul>  */
    // mvDepth直接由depth图像读取`
    //这里是初始化这两个存储“右图”匹配特征点横坐标和存储特征点深度值的vector
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        //从未畸变矫正的深度图中获取深度信息，从矫正过后的左图获取特征点位置信息，构建虚拟右目
        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}
//三角化特征点坐标
cv::Mat Frame::UnprojectStereo(const int &i)
{
    //计算该特征点的世界三维坐标
    const float z = mvDepth[i];
    if(z>0)
    {
        //u = fx(X/Z) + cx
        //v = fy(Y/Z) + cy
        // X = (u-cx) * Z / fx
        // Y = (v-cy) * Z / fy
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
