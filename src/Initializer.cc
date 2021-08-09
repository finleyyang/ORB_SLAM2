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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"

#include<thread>

namespace ORB_SLAM2
{

/**
 * @brief 根据参考帧构造初始化器
 *
 * @param[in] ReferenceFrame        参考帧
 * @param[in] sigma                 测量误差
 * @param[in] iterations            RANSAC迭代次数
 */
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    //获取估计误差
    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

/**
 * @brief 计算基础矩阵和单应性矩阵，选取最佳的来恢复出最开始两帧之间的相对姿态，并进行三角化得到初始地图点
 * Step 1 重新记录特征点对的匹配关系
 * Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
 * Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算
 * Step 4 计算得分比例来判断选取哪个模型来求位姿R,t
 *
 * @param[in] CurrentFrame          当前帧，也就是SLAM意义上的第二帧
 * @param[in] vMatches12            当前帧（2）和参考帧（1）图像中特征点的匹配关系
 *                                  vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
 *                                  没有匹配关系的话，vMatches12[i]值为 -1
 * @param[in & out] R21                   相机从参考帧到当前帧的旋转
 * @param[in & out] t21                   相机从参考帧到当前帧的平移
 * @param[in & out] vP3D                  三角化测量之后的三维地图点
 * @param[in & out] vbTriangulated        标记三角化点是否有效，有效为true
 * @return true                     该帧可以成功初始化，返回true
 * @return false                    该帧不满足初始化条件，返回false
 */
bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    // 初始化器Initializer对象创建时就已经指定mvKeys1，调用本函数只需要指定mvKey2即可
    mvKeys2 = CurrentFrame.mvKeysUn;        //current frame中的特征点

    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    //reserve：改变当前vector所分配空间的大小，对应capacity()，表示在发生重新分配之前允许存放多少元素
    mvbMatched1.resize(mvKeys1.size());
    //resize：改变当前使用数据的大小，如果他比当前使用的大，填充默认值
    //对应size()，表示实际容器中保存元素的个数

    //1.将vMatches12拷贝到mvMatches12，mvMatches12只保存匹配上的特征点对
    // 将vMatches12（有冗余） 转化为 mvMatches12（只记录了匹配关系）
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        //vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
        //没有匹配关系的话，vMatches12[i]值为 -1
        if(vMatches12[i]>=0)
        {
            //mvMatches12 中只记录有匹配关系的特征点对的索引值
            //i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    // 有匹配的特征点的对数
    const int N = mvMatches12.size();

    // Indices for minimum set selection
    //2.准备RANSAC运算中需要的特征点对
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);

    //在RANSAC的某次迭代中，还可以被抽取来作为数据样本的特征点对的索引，所以这里起的名字叫做可用的索引
    vector<size_t> vAvailableIndices;

    //初始化所有特征点对的索引，索引值0到N-1
    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
    // 共选择 mMaxIterations (默认200) 组
    // mvSets用于保存每次迭代时所使用的向量
    mvSets = vector< vector<size_t> >(mMaxIterations,   //最大的RANSAC迭代次数
                                      vector<size_t>(8,0)); //这个则是第二维元素的初始值，也就是第一维。这里其实也是一个第一维的构造函数，第一维vector有8项，每项的初始值为0.

    DUtils::Random::SeedRandOnce(0);

    //这里最大迭代次数是200次
    for(int it=0; it<mMaxIterations; it++)
    {
        // 将所有的特征点数据赋给vAvailableIndices
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        // 每次选取8个点来准备RANSAC运算中的特征点--采用随机取下标的情况
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;
            //把随机取得点放在vector的最上面，并弹出去
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }
    // 计算F矩阵和H矩阵及其可置信程度
    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    //多线程处理单应矩阵和基础矩阵来判断哪个效果更好
    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    float RH = SH/(SH+SF);
    //如果单应矩阵的分数占比大于0.4，则用单应矩阵求导,ORBSLAM中更倾向使用单应矩阵

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    // 注意这里更倾向于用H矩阵恢复位姿。如果单应矩阵的评分占比达到了0.4以上,则从单应矩阵恢复运动,否则从基础矩阵恢复运动
    if(RH>0.40)
        //更偏向于平面，此时从单应矩阵恢复，函数ReconstructH返回bool型结果
        return ReconstructH(vbMatchesInliersH,   //输入，匹配成功的特征点对Inliers标记
                            H,      //输入，前面RANSAC计算后的单应矩阵
                            mK,     //输入，相机的内参数矩阵
                            R21,t21,    //输出，计算出来的相机从参考帧1到当前帧2所发生的旋转和位移变换
                            vP3D,       //特征点对经过三角测量之后的空间坐标，也就是地图点
                            vbTriangulated, //特征点对是否成功三角化的标记
                            1.0,    //这个对应的形参为minParallax，即认为某对特征点的三角化测量中，认为其测量有效时 //需要满足的最小视差角（如果视差角过小则会引起非常大的观测误差）,单位是角度
                            50); //为了进行运动恢复，所需要的最少的三角化测量成功的点个数
    else //if(pF_HF>0.6)
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

    return false;
}

/**
 * @brief 计算单应矩阵，假设场景为平面情况下通过前两帧求取Homography矩阵，并得到该模型的评分
 * 原理参考Multiple view geometry in computer vision  P109 算法4.4
 * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
 * Step 2 选择8个归一化之后的点对进行迭代
 * Step 3 八点法计算单应矩阵矩阵
 * Step 4 利用重投影误差为当次RANSAC的结果评分
 * Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
 *
 * @param[in & out] vbMatchesInliers          标记是否是外点
 * @param[in & out] score                     计算单应矩阵的得分
 * @param[in & out] H21                       单应矩阵结果
 */

//H矩阵本来只需要4对点就能解出结果，但是为了能使H和F能在同一维度下比较，所以也取八对点
void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();  //用于恢复原始尺度

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        // 把匹配上的点提取出来，都是匹配好一对一对的
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        //恢复原始尺度
        H21i = T2inv*Hn*T1; //单应矩阵
        H12i = H21i.inv(); //单应矩阵逆矩阵

        //分值越高说明，计算解的H或F越好
        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        // 获取最好的得分对应的单应矩阵
        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

//以下两种场景更适合使用单应矩阵进行初始化
//相机看到的场景是一个平面
//连续两帧之间没发生平移，只发生旋转

/**
 * @brief 计算基础矩阵，假设场景为非平面情况下通过前两帧求取Fundamental矩阵，得到该模型的评分
 * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
 * Step 2 选择8个归一化之后的点对进行迭代
 * Step 3 八点法计算基础矩阵矩阵
 * Step 4 利用重投影误差为当次RANSAC的结果评分
 * Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
 *
 * @param[in & out] vbMatchesInliers          标记是否是外点
 * @param[in & out] score                     计算基础矩阵得分
 * @param[in & out] F21                       从特征点1到2的基础矩阵
 */

void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    // const int N = vbMatchesInliers.size();

    const int N = mvMatches12.size();
    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    // 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    // 具体来说,就是将mvKeys1和mvKey2归一化到均值为0，一阶绝对矩为1，归一化矩阵分别为T1、T2
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值
    // 归一化矩阵就是把上述归一化的操作用矩阵来表示。这样特征点坐标乘归一化矩阵可以得到归一化后的坐标
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);

    // ! 注意这里取的是归一化矩阵T2的转置,因为基础矩阵的定义和单应矩阵不同，两者去归一化的计算也不相同
    cv::Mat T2t = T2.t();  //用于恢复原始尺度

    // Best Results variables
    // 最优结果
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    // 某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
    // 某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;

    // 每次RANSAC记录的Inliers与得分
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    // 下面进行每次的RANSAC迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            // 把匹配上的点提取出来，都是匹配好一对一对的
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }


        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);
        // 基础矩阵约束：p2^t*F21*p1 = 0，其中p1,p2 为齐次化特征点坐标
        // 特征点归一化：vPn1 = T1 * mvKeys1, vPn2 = T2 * mvKeys2
        // 根据基础矩阵约束得到:(T2 * mvKeys2)^t* Hn * T1 * mvKeys1 = 0
        // 进一步得到:mvKeys2^t * T2^t * Hn * T1 * mvKeys1 = 0

        //恢复原始尺度
        F21i = T2t*Fn*T1;

        //根据从投影误差进行卡方检验
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        //记录最优解
        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief 用DLT方法求解单应矩阵H
 * 这里最少用4对点就能够求出来，不过这里为了统一还是使用了8对点求最小二乘解
 *
 * @param[in] vP1               参考帧中归一化后的特征点
 * @param[in] vP2               当前帧中归一化后的特征点
 * @return cv::Mat              计算的单应矩阵H
 */

cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    // |x'|     | h1 h2 h3 ||x|
    // |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
    // |1 |     | h7 h8 h9 ||1|

    // 构造用于计算的矩阵 A , SLAM十四讲172页， 将右侧坐标也提到左侧
    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    //奇异值分解，取vt最后一行
    // 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    // 返回最小奇异值所对应的右奇异向量
    // 注意前面说的是右奇异值矩阵的最后一列，但是在这里因为是vt，转置后了，所以是行；由于A有9列数据，故最后一列的下标为8
    return vt.row(8).reshape(0, 3);
}

//超定方程使用SVD分解求解最小二乘解

/**
 * @brief 根据特征点匹配求fundamental matrix（normalized 8点法）
 * 注意F矩阵有秩为2的约束，所以需要两次SVD分解
 *
 * @param[in] vP1           参考帧中归一化后的特征点
 * @param[in] vP2           当前帧中归一化后的特征点
 * @return cv::Mat          最后计算得到的基础矩阵F
 */
cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);
    //A*H=0
    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    //2.奇异值分解A，A为8*9矩阵，取vt的最后一行
    cv::Mat u,w,vt;

    // 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    // 第二次SVD分解是为了保证基础矩阵的秩为2
    // 转换成基础矩阵的形式
    cv::Mat Fpre = vt.row(8).reshape(0, 3);//v的最后一行

    //基础矩阵的秩为2,而我们不敢保证计算得到的这个结果的秩为2,所以需要通过第二次奇异值分解,来强制使其秩为2
    //将F矩阵的秩强制设为2
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    // 重新组合好满足秩约束的基础矩阵，作为最终计算结果返回
    return  u*cv::Mat::diag(w)*vt;
}

//卡方检验通过构造检验统计量x2来比较期望结果和实际结果之间的差别，从而得出观察频数极值的发生概率
//x2 = ∑（O-E）2/E;
//根据重投影误差构造统计量X2，其值越大，观察结果和期望结果之间的差别越明显，某次计算结果越可能用到外点
/**
 * @brief 对给定的homography matrix打分,需要使用到卡方检验的知识
 *
 * @param[in] H21                       从参考帧到当前帧的单应矩阵
 * @param[in] H12                       从当前帧到参考帧的单应矩阵
 * @param[in] vbMatchesInliers          匹配好的特征点对的Inliers标记
 * @param[in] sigma                     方差，默认为1
 * @return float                        返回得分
 */
float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);           //用来标记是否是外点

    float score = 0;            //置信度得分

    const float th = 5.991;
    //卡方统计表，两个自由度的时候，它的阀值为5.99时，它的正确率达到95；
    //若统计量大于该阀值，则认为计算矩阵使用到了外电，将其分数设为0；
    //若统计量大于该阀值，则将统计量裕值设为该解的置信度分数

    const float invSigmaSquare = 1.0/(sigma*sigma);  //信息矩阵方差平方的倒数

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        //提取特征点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2
        // 计算img2到img1的重投影误差
        // |u1|   |h11inv h12inv h13inv||u2|   |u2in1|
        // |v1| = |h21inv h22inv h23inv||v2| = |v2in1| * w2in1inv
        // |1 |   |h31inv h32inv h33inv||1 |   |  1  |
        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        //计算比例误差
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);
        const float chiSquare1 = squareDist1*invSigmaSquare;
        //然后计算重投影误差

        //将离群点标记上，非离群点累加计算得分
        if(chiSquare1>th)
            bIn = false;
        else
            // 误差越大，得分越低
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1
        // 计算img1到img2的重投影误差
        // 计算从img1 到 img2 的投影变换误差

        // 将图像2中的特征点通过单应变换投影到图像1中
        // |u2|   |h11 h12 h13||u1|   |u1in2|
        // |v2| = |h21 h22 h23||v1| = |v1in2| * w1in2inv
        // |1 |   |h31 h32 h33||1 |   |  1  |
        // 计算投影归一化坐标
        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        //计算比例误差
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;


        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
        const float chiSquare2 = squareDist2*invSigmaSquare;

        //将离群点标记上，非离群点累加计算得分
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        // 问题？？如果1到2的重投影超过阀值，2-1没有超过，就会被标记为离群点，为什么还要算分数？？？

        //如果是1-2或者2-1有一个被标记为离群点，则false
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

/**
 * @brief 对给定的Fundamental matrix打分
 *
 * @param[in] F21                       当前帧和参考帧之间的基础矩阵
 * @param[in] vbMatchesInliers          匹配的特征点对属于inliers的标记
 * @param[in] sigma                     方差，默认为1
 * @return float                        返回得分
 */

float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{

    // 获取匹配的特征点对的总对数
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)
        // 根据重投影，来计算误差
        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
        //img1上的特征点与F相乘，即为img2上的射线，理论上特征点的投影应该在射线上

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)
        // 根据重投影，来计算误差
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;
        //img2上的特征点与F相乘，即为img1上的射线，理论上特征点的投影应该在射线上

        const float num1 = a1*u1+b1*v1+c1;
        const float squareDist2 = num1*num1/(a1*a1+b1*b1);
        //然后根据点到线的距离来计算误差
        //然后平方来求平方误差

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;
    //首先计算出本质矩阵

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);
    //结算出两个R和两个t
    cv::Mat t1=t;
    cv::Mat t2=-t;
    //形成4对解
    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    //最优解应该与其他的解有区分度，除了最优解，其他的都应该小于最优解的70%，否者返回false
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    // 选择记录最佳结果，检验三角化出的特征点数和视角差
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

// H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang SVD-based decomposition
// 参考文献：Motion and structure from motion in a piecewise plannar environment
// 这篇参考文献和下面的代码使用了Faugeras SVD-based decomposition算法

//把单应矩阵解出旋转矩阵和位移向量
/**
 * @brief 从基础矩阵F中求解位姿R，t及三维点
 * F分解出E，E有四组解，选择计算的有效三维点（在摄像头前方、投影误差小于阈值、视差角大于阈值）最多的作为最优的解
 * @param[in] vbMatchesInliers          匹配好的特征点对的Inliers标记
 * @param[in] F21                       从参考帧到当前帧的基础矩阵
 * @param[in] K                         相机的内参数矩阵
 * @param[in & out] R21                 计算好的相机从参考帧到当前帧的旋转
 * @param[in & out] t21                 计算好的相机从参考帧到当前帧的平移
 * @param[in & out] vP3D                三角化测量之后的特征点的空间坐标
 * @param[in & out] vbTriangulated      特征点三角化成功的标志
 * @param[in] minParallax               认为三角化有效的最小视差角
 * @param[in] minTriangulated           最小三角化点数量
 * @return true                         成功初始化
 * @return false                        初始化失败
 */
bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;
    //A=dR+tnt
    //A=U∑Vt
    //∑=UtAV=dUtRV+(Vt*t)(Vtn)t

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //共有八种结果
    //d=d2,d=-d2 (两种)

    //     |ctheta,   0,   stheta|           | x1|
    //   R=|   0,     1,     0  |        t = | 0 |
    //     |stheta,    0,  ctheta|           | x3|
    // x1和x2，有同正，同负，一正一负， 一负一正四种情况；
    // R会根据x1，x3的正负情况儿改变
    //
    //   stheta = (d1-d3)/d2x1x3
    //
    //   ctheta = (d1x3^2 - d3x1^2)/d2;

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};
    // step3.2：计算四种旋转矩阵R，t
    // 计算旋转矩阵 R‘，计算ppt中公式18

    //      | ctheta      0    aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       | aux3|

    //      | ctheta      0    aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      |-aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0   -aux_stheta|       | aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       |-aux3|

    //      | ctheta      0   -aux_stheta|       |-aux1|
    // Rp = |    0        1       0      |  tp = |  0  |
    //      | aux_stheta  0    ctheta    |       | aux3|

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    // 然后根据求得八种情况，进行比较计算好点的情况
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            //如果该次还原出来的Rt比较好
            secondBestGood = bestGood;
            //把上一次最好的点的数量赋给第二好
            bestGood = nGood;
            //当前好点的数量为最好
            bestSolutionIdx = i;
            //记录最好是那一次
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
            //把视角差，还原出来的三维点，以及是否三角化，赋给最好那组
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    //如果最好的情况的75%还大于第二好的情况，90%的点是好点

    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}
/** 给定投影矩阵P1,P2和图像上的匹配特征点点kp1,kp2，从而计算三维点坐标
 * @brief
 *
 * @param[in] kp1               特征点, in reference frame
 * @param[in] kp2               特征点, in current frame
 * @param[in] P1                投影矩阵P1
 * @param[in] P2                投影矩阵P2
 * @param[in & out] x3D         计算的三维点
 */

void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    // 原理
    // Trianularization: 已知匹配特征点对{x x'} 和 各自相机矩阵{P P'}, 估计三维点 X
    // x' = P'X  x = PX
    // 它们都属于 x = aPX模型
    //                         |X|
    // |x|     |p1 p2  p3  p4 ||Y|     |x|    |--p0--||.|
    // |y| = a |p5 p6  p7  p8 ||Z| ===>|y| = a|--p1--||X|
    // |z|     |p9 p10 p11 p12||1|     |z|    |--p2--||.|
    // 采用DLT的方法：x叉乘PX = 0
    // |yp2 -  p1|     |0|
    // |p0 -  xp2| X = |0|
    // |xp1 - yp0|     |0|
    // 两个点:
    // |yp2   -  p1  |     |0|
    // |p0    -  xp2 | X = |0| ===> AX = 0
    // |y'p2' -  p1' |     |0|
    // |p0'   - x'p2'|     |0|
    // 变成程序中的形式：
    // |xp2  - p0 |     |0|
    // |yp2  - p1 | X = |0| ===> AX = 0
    // |x'p2'- p0'|     |0|
    // |y'p2'- p1'|     |0|
    // 然后就组成了一个四元一次正定方程组，SVD求解，右奇异矩阵的最后一行就是最终的解.

    // 参考https://zhuanlan.zhihu.com/p/63179478
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

//使用均值和一阶中心矩归一化，归一化可以增强计算稳定性
/**
 * @brief 归一化特征点到同一尺度，作为后续normalize DLT的输入
 *  [x' y' 1]' = T * [x y 1]'
 *  归一化后x', y'的均值为0，sum(abs(x_i'-0))=1，sum(abs((y_i'-0))=1
 *
 *  为什么要归一化？
 *  在相似变换之后(点在不同的坐标系下),他们的单应性矩阵是不相同的
 *  如果图像存在噪声,使得点的坐标发生了变化,那么它的单应性矩阵也会发生变化
 *  我们采取的方法是将点的坐标放到同一坐标系下,并将缩放尺度也进行统一
 *  对同一幅图像的坐标进行相同的变换,不同图像进行不同变换
 *  缩放尺度是为了让噪声对于图像的影响在一个数量级上
 *
 *  Step 1 计算特征点X,Y坐标的均值
 *  Step 2 计算特征点X,Y坐标离均值的平均偏离程度
 *  Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1
 *  Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
 *
 * @param[in] vKeys                               待归一化的特征点
 * @param[in & out] vNormalizedPoints             特征点归一化后的坐标
 * @param[in & out] T                             归一化特征点的变换矩阵
 */
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    //这里的归一化，归一的是这些点在x方向和在y方向上的一阶绝对矩。步骤如下
    const int N = vKeys.size();// 点总数
    vNormalizedPoints.resize(N);//标准化后的点

    float meanX = 0;//横坐标均值
    float meanY = 0;//纵坐标均值
    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;// 横坐标之和
        meanY += vKeys[i].pt.y;// 纵坐标之和
    }
    meanX = meanX/N;//横坐标均值
    meanY = meanY/N;//纵坐标均值

    // 分别累计这些特征点偏离横纵坐标均值的多少
    float meanDevX = 0;//绝对矩
    float meanDevY = 0;//绝对矩

    // 将所有vKeys点减去中心坐标，使x坐标和y坐标均值分别为0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;// 去均值点坐标
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;
        //平移目的使其形心位于原点
        //累计这些特征点偏离横纵坐标均值的程度
        meanDevX += fabs(vNormalizedPoints[i].x);// 总绝对矩
        meanDevY += fabs(vNormalizedPoints[i].y);
    }
    // 求出平均到每个点上，其坐标偏离横纵坐标均值的程度；将其倒数作为一个尺度缩放因子
    meanDevX = meanDevX/N; //均值绝对矩
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;
    //
    // 将x坐标和y坐标分别进行尺度缩放，使得x坐标和y坐标的一阶绝对矩分别为1
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值, 归一化就体现在这里
    for(int i=0; i<N; i++)
    {
        // 归一化后的点坐标   //就是简单地对特征点的坐标进行进一步的缩放
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;  // 去均值点坐标 * 绝对矩倒数
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }
    //计算归一化矩阵
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0       1   |
    //  标准化矩阵
    //  标准化矩阵 * 点坐标 = 标准化后的坐标
    //  点坐标 = 标准化矩阵的逆矩阵 * 标准化后的的坐标
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

//通过成功三角化的特征点个数判断结果的好坏；若某特征点的重投影误差小于4且视角差大于0.36度，则认为该特征点三角化成功
int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    //相机的投影矩阵P = KRt^
    // 投影方程 zp=KRt^P
    // Camera 1 Projection Matrix K[I|0]
    // 以相机1光心为世界坐标系原点，计算相机的投影矩阵和光心位置
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3)); //P1=[I|0]，P1的投影矩阵P1=K[I|0]

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);           //o1表示世界坐标下相机1的光心位置o1 = 0


    // Camera 2 Projection Matrix K[R|t]
    // 相机2的位置以及投影矩阵
    cv::Mat P2(3,4,CV_32F);                                 //P2=[R|t]
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;           //P2的投影矩阵P2=K[R|t]

    //根据下面的注释可得P2的光心为-R.t()*t

    // A->B的变换矩阵为|R t|，则B->A的变换矩阵则为|Rt -Rt*t|
    //               |0 1|                   |0     1|
    //

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        //跳过外点
        if(!vbMatchesInliers[i])
            continue;

        //获取特征点，调用Triangulate()函数进行三角化，得到三角化测量之后的3D坐标
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        //储存三维点的坐标
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);
        //只要三角化中的结果有一个是无穷大的就说明三角化失败，跳过对当前特征的处理，进行下一对特征点的遍历
        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            //可以不添加这一句，默认就是false
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        //通过检查深度以及视角差来判断是否合法
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);
        //得到三维特征点到光心O1的距离
        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);
        //得到三维特征点到光心O1的距离

        //根据公式axb=|a||b|cos_theta，来计算视角的差值
        float cosParallax = normal1.dot(normal2)/(dist1*dist2);


        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)

        //三维特征点对于第一个相机的位置，来检查深度是否合理，cos0度是1，
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;
        //三维特征点对于第二个相机的位置，来检查深度是否合理，cos0度是1，
        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        // 计算从投影误差，如果大于阀值，则放弃
        // 应该这一个只针对与单应矩阵H分解情况，FE分解只会有一种情况特征点只在两个相机前方
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;
        //计算3D点在图像上的位置

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);
        //计算平方差

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        //计算第二个的点的重投影误差，三维特征点已经转换到第二个相机坐标下面
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        //统计好点的个数
        vCosParallax.push_back(cosParallax);
        //储存这个三维点在世界坐标下的位置，即相当与第一个相机光心坐标下的位置
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;
        //只是把好点标记上
        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        //对视差点进行重排序
        sort(vCosParallax.begin(),vCosParallax.end());
        //如果检测后有效点大于50个，则取第五十个，不大于50个就取最大的哪一个
        size_t idx = min(50,int(vCosParallax.size()-1));
        //将弧度转角度记录下来
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);
    //对E进行SVD分解

    u.col(2).copyTo(t);
    //取u的最后一行为t;
    t=t/cv::norm(t);
    //因为t的具体长度是不能确定的；
    //取正则化的；

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;
    //      |0, -1, 0|
    //    W=|1, 0, 0|
    //      |0, 0, 1|

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
    //取行列式为正的矩阵
}
//设本质矩阵E=[t]xR,它的SVD分解E=U∑Vt，其中∑的奇异值一定是[1，1，0]T，不考虑本质矩阵的符号，则存在两种可能的解：
//S=UZUt, R=UWVt
//S=UZUt, R=UWtVt
//其中   |0, -1, 0|   |0, -1, 0|
//    W=|1, 0, 0|  Z=|1, 0, 0|
//      |0, 0, 1|    |0, 0, 0|
//推论：已知本质矩阵E=U∑Vt 和第一个相机的矩阵P=[I|0]， 那么第二个摄像头的矩阵有以下四种可能的选择：
//P=[UWVt|u];[UWVt|-u];[UWtVt|u];[UWtVt|-u];
//u是U的最后一列
//u=t的具体空间长度是不能恢复出来的，只能在相差一个尺度因子的情况下恢复平移单位向量
//


//H矩阵的为3x3的矩阵，考虑到尺度等价性的约束，实际自由度为8
//F矩阵最大自由度为9，虽然R+T+K+K的自由度远大于9；然后考虑到尺度等价性和行列式det(F)=0两个约束，实际自由度为9-2=7
//E的自由度是由R和t决定的，R和t分别3个自由度，然后减去一个尺度等价性，有5个自由度
//AX=0，只A满秩的情况下，A只有唯一零解。
//XEX=0,有非零解，则可知，E不满秩
} //namespace ORB_SLAM
