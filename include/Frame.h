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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
/**
 * @name 定义一帧中有多少个图像网格
 * @{
 */

/**
 * @brief 网格的行数
 *
 */
#define FRAME_GRID_ROWS 48
/**
 * @brief 网格的列数
 *
 */
#define FRAME_GRID_COLS 64
    //网格行数、列数

class MapPoint;
class KeyFrame;


class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
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
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    /**
     * @brief 为RGBD相机准备的帧构造函数
     *
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    /**
     * @brief 为单目相机准备的帧构造函数
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
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    /**
     * @brief 提取图像的ORB特征，提取的关键点存放在mvKeys，描述子存放在mDescriptors
     *
     * @param[in] flag          标记是左图还是右图。0：左图  1：右图
     * @param[in] im            等待提取特征点的图像
     */
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    /**
     * @brief 计算词袋模型
     * @details 计算词包 mBowVec 和 mFeatVec ，其中 mFeatVec 记录了属于第i个node（在第4层）的ni个描述子
     * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
     */
    void ComputeBoW();

    // Set the camera pose.
    /**
     * @brief 用 Tcw 更新 mTcw 以及类中存储的一系列位姿
     *
     * @param[in] Tcw 从世界坐标系到当前帧相机位姿的变换矩阵
     */
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    /**
     * @brief 根据相机位姿,计算相机的旋转,平移和相机中心等矩阵.
     * @details 其实就是根据Tcw计算mRcw、mtcw和mRwc、mOw.
     */
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    /**
     * @brief 判断路标点是否在视野中
     * 步骤
     * Step 1 获得这个地图点的世界坐标
     * Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
     * Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
     * Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
     * Step 5 关卡四：计算当前视角和“法线”夹角的余弦值, 若小于设定阈值，返回false
     * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
     * Step 7 记录计算得到的一些参数
     * @param[in] pMP                       当前地图点
     * @param[in] viewingCosLimit           夹角余弦，用于限制地图点和光心连线和法线的夹角
     * @return true                         地图点合格，且在视野内
     * @return false                        地图点不合格，抛弃
     */
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    /**
     * @brief 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
     *
     * @param[in] kp                    给定的特征点
     * @param[in & out] posX            特征点所在网格坐标的横坐标
     * @param[in & out] posY            特征点所在网格坐标的纵坐标
     * @return true                     如果找到特征点所在的网格坐标，返回true
     * @return false                    没找到返回false
     */
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    //获取半径为r的圆域内的特征点编号列表

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
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;



    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    //对于双目相机，知道特征点的左目坐标、右目坐标，不知道特征点深度
    //RGBD相机，知道特征点左目坐标、特征点深度，不知道右目坐标

    //双目视差公式为z/(z-f) = b/(b-d) 其中d = xl-xr
    //z为特征点深度，f为焦距，b为基线长度，d为左右视差

    //双目相机分别提取到左右目特征点之后对特征点进行双目匹配，并通过双目视差估计特征点深度，双目特征点匹配步骤
    //1.粗匹配：根据特征点距离和特征金字塔层级判断匹配，粗匹配关系是按行寻找的，对于左目图像中每个特征点，在右目图像对应行上寻找匹配特征点。
    //2.精匹配：根据特征点周围窗口内容相似度判断。
    //3.亚像素插值，将特征点相似度与匹配坐标之间拟合成二次曲线，寻找最佳匹配位置(得到的是一个小数)。
    //4.记录右目匹配mvuRight和深度mvDepth信息。
    //5.离群点筛选，以平均相似度的2.1倍为标准，筛选离群点。

    /**
     * @brief 计算双目图像之间的匹配关系 \n
     * @details 如果对于一对特征点确定有匹配关系存在,那么这个特征点在空间中的深度将会被计算,并且和左特征点相对应的右特征点的坐标将会被存储.
     * \n 说白了，就是为左图的每一个特征点在右图中找到匹配点
     * \n 方法是根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位
     * \n 这里所说的SAD是一种双目立体视觉匹配算法，可参考[https://blog.csdn.net/u012507022/article/details/51446891]
     * \n 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配
     * \n 这里所谓的亚像素精度，就是使用这个拟合得到一个小于一个单位像素的修正量，这样可以取得更好的估计结果，计算出来的点的深度也就越准确
     * \n 匹配成功后会更新 Frame::mvuRight (ur) 和 Frame::mvDepth (Z)
     */
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    /**
     * @brief 对于RGBD输入,如果某个特征点的深度值有效,那么这里将会反向计算出假想的"右目图像中对应特征点的坐标".
     * @detials 博客[https://www.cnblogs.com/panda1/p/7001052.html]中说，这个是将地图点和其深度对应起来.
     * 是不是可以这样理解，为了能够将特征点反投影到三维空间中得到其在相机坐标系以及在世界坐标系下的坐标，我们需要获得它
     * 在当前相机下的深度。对于双目相机，我们是通过计算左侧图像中特征点在右图中的坐标，然后计算其深度；对于RGBD图像我们可以直接
     * 从深度图像上获得特征点的深度，不过为了处理上的一致这里使用这个深度计算了彩色图像（左图）中的特征点在假想的“右图”中的
     * 坐标。这就是这个函数的工作.
     *
     * @param[in] imDepth 深度图像
     */
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //双目相机不需要畸变矫正
    /**
     * @brief 当某个特征点的深度信息或者双目信息有效时,将它反投影到三维世界坐标系中
     *
     * @param[in] i     特征点的ID
     * @return cv::Mat  反投影后得到的特征点的反投影点的世界坐标
     */
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    //用于重定位的ORB特征字典
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;               //左右目的特征点提取器

    // Frame timestamp.
    //帧的时间戳
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    //相机的内参矩阵都是固定的，而且所有帧公用
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;    //相机畸变矫正参数

    // Stereo baseline multiplied by fx.
    //基线距离乘以内参
    float mbf;

    // Stereo baseline in meters.
    //基线距离
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;        //判断单目特征点和双目特征点的阀值，深度低于该值的被认为是单目特征点或者是双目特征点

    // Number of KeyPoints.
    int N;  //关键特征点点的数量

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant(多余的) as images must be rectified(改正的，整流的).
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;          //畸变校正前的左右目特征点
    std::vector<cv::KeyPoint> mvKeysUn;                     //畸变校正后的左目特征点

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;               //左目特征点在右目中匹配特征点的横坐标（左右目匹配特征点的纵坐标相同）
    std::vector<float> mvDepth;                //特征点深度

    //对于第i个图像特征点来说
    //其畸变矫正前的左目特征点是mvKeys[i]
    //其畸变矫正前的左目特征点是mvKeysUn[i]
    //其在右目图片中对应的特征点的横坐标为mvuRight[i],纵坐标与mvKeys[i]的纵坐标相同
    //特征点的深度是mvDepth[i]

    //对于单目特征点(单目相机输入的特征点或没有找到右目匹配的左目图像特征点），其mvuRight和mvDepth均为-1

    // Bag of Words Vector structures.
    // 内部实际存储的是std::map<WordId, WordValue>
    // WordId 和 WordValue 表示Word在叶子中的id 和权重
    DBoW2::BowVector mBowVec;
    // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
    // NodeId 表示节点id，std::vector<unsigned int> 中实际存的是该节点id下所有特征点在图像中的索引
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;                   //左右目的特征点描述子

    // MapPoints associated to keypoints, NULL pointer if no association.
    // 每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    // 属于外点的特征点标记,在 Optimizer::PoseOptimization 使用了
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // 每个网格的宽度和高度的倒数
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;

    //每个网格内特征点编号列表
    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
    // FRAME_GRID_ROWS 48
    // FRAME_GRID_COLS 64
    //这个向量中存储的是每个图像网格内特征点的id
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; //相机坐标

    // Current and Next Frame id.
    // 类的静态成员变量，这些变量则是在整个系统开始执行的时候被初始化的——它在全局区被初始化
    static long unsigned int nNextId; // Next Frame id 下一帧编号
    long unsigned int mnId;//Current Frame id 当前帧编号

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;    // 普通帧与自己共视程度最高的关键帧作为参考关键帧

    // Scale pyramid info.
    // 图像金字塔信息
    int mnScaleLevels;     //图像金字塔的层数
    float mfScaleFactor;    // 图像金字塔的尺度因子
    float mfLogScaleFactor;     //图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度
    vector<float> mvScaleFactors;		//图像金字塔每一层的缩放因子
    vector<float> mvInvScaleFactors;	//以及上面的这个变量的倒数
    vector<float> mvLevelSigma2;		//尺度因子的平方,用在pnp求解的阀值
    vector<float> mvInvLevelSigma2;		//上面变量的倒数


    // Undistorted Image Bounds (computed once).
    //畸变矫正后的图像边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    //是否为第一帧的标志，如果是第一帧，则需要初始化相机内参
    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    // 对所有特征点进行畸变矫正
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    //计算畸变矫正后的图像边界
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    // 将特征点分配到48行64列的网格中以加速匹配
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center

    // ....cw,即为世界到相机坐标，即为当前目标的世界坐标
    cv::Mat mRcw;  //旋转，从世界坐标到相机坐标  //即为当前相机的pose
    cv::Mat mtcw;  //平移，从世界坐标到相机坐标
    cv::Mat mRwc;  //旋转，从相机坐标到世界坐标
    cv::Mat mOw; //==mtwc 平移，从世界坐标到相机坐标
};

}// namespace ORB_SLAM

#endif // FRAME_H
