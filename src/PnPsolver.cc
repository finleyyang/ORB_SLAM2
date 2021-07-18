/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include <iostream>

#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{

    //PnP求解器只在track中调用

    // 在大体的pipeline上和Sim3Solver差不多,都是 构造->设置RANSAC参数->外部调用迭代函数,进行计算->得到计算的结果

// pcs表示3D点在camera坐标系下的坐标
// pws表示3D点在世界坐标系下的坐标
// us表示图像坐标系下的2D点坐标
// alphas为真实3D点用4个虚拟控制点表达时的系数
// 构造函数
PnPsolver::PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches):
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    // 根据点数初始化容器的大小
    mvpMapPointMatches = vpMapPointMatches;             //匹配关系
    mvP2D.reserve(F.mvpMapPoints.size());               //2D特征点
    mvSigma2.reserve(F.mvpMapPoints.size());            //特征点金字塔层级
    mvP3Dw.reserve(F.mvpMapPoints.size());              //世界坐标系下的3D点
    mvKeyPointIndices.reserve(F.mvpMapPoints.size());   //记录被使用特征点在原始特征点容器中的索引，因为有些3D点不一定存在，所以索引是不连续的
    mvAllIndices.reserve(F.mvpMapPoints.size());        //记录被使用特征点的索引，是连续的

    // 生成地图点、对应2D特征点，记录一些索引坐标
    int idx=0;

    // 遍历给出每一个地图点
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];// 依次获取每个地图点

        if(pMP)
        {
            if(!pMP->isBad())
            {
                const cv::KeyPoint &kp = F.mvKeysUn[i];//得到2维特征点, 将KeyPoint类型变为Point2f

                mvP2D.push_back(kp.pt); //存放2维特征点
                mvSigma2.push_back(F.mvLevelSigma2[kp.octave]); //记录特征点是在哪一层取出来的

                cv::Mat Pos = pMP->GetWorldPos();  //世界坐标系下的3D点
                mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));

                mvKeyPointIndices.push_back(i);  //记录被使用特征点在原始特征点容器中的索引, mvKeyPointIndices是跳跃的
                mvAllIndices.push_back(idx);        //记录被使用特征点的索引, mvAllIndices是连续的

                idx++;
            }
        }
    }

    // Set camera calibration parameters
    fu = F.fx;
    fv = F.fy;
    uc = F.cx;
    vc = F.cy;

    SetRansacParameters(); //????为什么这边重新调用了一次
}

PnPsolver::~PnPsolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}

/**
 * @brief 设置RANSAC迭代的参数
 * @param[in] probability       用于计算RANSAC理论迭代次数所用的概率
 * @param[in] minInliers        退出RANSAC所需要的最小内点个数, 注意这个只是给定值,最终迭代的时候不一定按照这个来
 * @param[in] maxIterations     设定的最大RANSAC迭代次数
 * @param[in] minSet            表示求解这个问题所需要的最小的样本数目,简称最小集;参与到最小内点数的确定过程中，默认是4
 * @param[in] epsilon           希望得到的 内点数/总体数 的比值,参与到最小内点数的确定过程中
 * @param[in] th2               内外点判定时的距离的baseline(程序中还会根据特征点所在的图层对这个阈值进行缩放的)
 */

void PnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon;
    mRansacMinSet = minSet;

    //计算理论内点数,并且选 min(给定内点数,最小集,理论内点数) 作为最终在迭代过程中使用的最小内点数
    N = mvP2D.size(); // number of correspondences

    mvbInliersi.resize(N); // inlier index, mvbInliersi记录每次迭代inlier的点

    // 再根据 epsilon 来计算理论上的内点数;
    // NOTICE 实际在计算的过程中使用的 mRansacMinInliers = min(给定内点数,最小集,理论内点数)
    // Adjust Parameters according to number of correspondences

    // 希望得到的内点的数量
    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;
    //mRansacMinInliers和nMinInliners取mRansacMinInliers和minSet的最大值

    // 根据敲定的"最小内点数"来调整 内点数/总体数 这个比例 epsilon
    // 这个变量却是希望取得高一点,也可以理解为想让和调整之后的内点数 mRansacMinInliers 保持一致吧
    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    // 根据给出的各种参数计算RANSAC的理论迭代次数,并且敲定最终在迭代过程中使用的RANSAC最大迭代次数
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    // 为什么还要取max
    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    // 计算不同图层上的特征点在进行内点检验的时候,所使用的不同判断误差阈值
    mvMaxError.resize(mvSigma2.size()); // 图像提取特征的时候尺度层数
    for(size_t i=0; i<mvSigma2.size(); i++)  // 不同的尺度，设置不同的最大偏差
        mvMaxError[i] = mvSigma2[i]*th2;
}

cv::Mat PnPsolver::find(vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers,nInliers);    
}

/**
 * @brief EPnP迭代计算
 *
 * @param[in] nIterations   迭代次数
 * @param[in] bNoMore       达到最大迭代次数的标志
 * @param[in] vbInliers     内点的标记
 * @param[in] nInliers      总共内点数
 * @return cv::Mat          计算出来的位姿
 */

cv::Mat PnPsolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;    //已经达到最大迭代次数的标志
    vbInliers.clear();
    nInliers=0;     // 当前次迭代时的内点数

    // mRansacMinSet 为每次RANSAC需要的特征点数，默认为4组3D-2D对应点
    set_maximum_number_of_correspondences(mRansacMinSet);

    // 如果已有匹配点数目比要求的内点数目还少，直接退出
    // N为所有2D点的个数, mRansacMinInliers 为正常退出RANSAC迭代过程中最少的inlier数
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    // mvAllIndices为所有参与PnP的2D点的索引
    // vAvailableIndices为每次从mvAllIndices中随机挑选mRansacMinSet组3D-2D对应点进行一次RANSAC
    vector<size_t> vAvailableIndices;

    // 当前的迭代次数ID
    int nCurrentIterations = 0;

    // 进行迭代的条件:
    // 条件1: 历史进行的迭代次数少于最大迭代值
    // 条件2: 当前进行的迭代次数少于当前函数给定的最大迭代值
    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        // 清空已有的匹配点的计数,为新的一次迭代作准备
        reset_correspondences();

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        // 随机选取四组点
        for(short i = 0; i < mRansacMinSet; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            // 将生成的这个索引映射到给定帧的特征点id
            int idx = vAvailableIndices[randi];

            // 将对应的3D-2D压入到pws和us. 这个过程中需要知道将这些点的信息存储到数组中的哪个位置,这个就由变量 number_of_correspondences 来指示了
            add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);

            // 从"可用索引表"中删除这个已经被使用的点
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Compute camera pose
        // 计算相机的位姿，mRi旋转矩阵，mti平移矩阵
        compute_pose(mRi, mti);

        // Check inliers
        // 通过之前求解的位姿来进行3D-2D投影，统计内点数目
        CheckInliers();

        // 如果当前次迭代得到的内点数已经达到了合格的要求了
        if(mnInliersi>=mRansacMinInliers)
        {
            // If it is the best solution so far, save it
            //更新最佳的计算结果
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_64F,mRi);
                cv::Mat tcw(3,1,CV_64F,mti);
                Rcw.convertTo(Rcw,CV_32F);
                tcw.convertTo(tcw,CV_32F);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            } //更新最佳的计算结果

            // 看能不能求的更精确
            if(Refine()) // 如果求精成功(即表示求精之后的结果能够满足退出RANSAC迭代的内点数条件了)
            {
                nInliers = mnRefinedInliers;
                vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
                for(int i=0; i<N; i++)
                {
                    if(mvbRefinedInliers[i])
                        vbInliers[mvKeyPointIndices[i]] = true;
                }
                // 对直接返回了求精之后的相机位姿
                return mRefinedTcw.clone();
            }
            // 如果求精之后还是打不到能够RANSAC的结果,那么就继续进行RANSAC迭代了
        }// 如果当前次迭代得到的内点数已经达到了合格的要求了
    }

    // 如果执行到这里,说明可能已经超过了上面的两种迭代次数中的一个了
    // 如果是超过了程序中给定的最大迭代次数
    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;
        // 没有更多的允许迭代次数了
        // 但是如果我们目前得到的最好结果看上去还不错的话
        if(mnBestInliers>=mRansacMinInliers)
        {
            //返回计算结果
            nInliers=mnBestInliers;
            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
            for(int i=0; i<N; i++)
            {
                if(mvbBestInliers[i])
                    vbInliers[mvKeyPointIndices[i]] = true;
            }
            return mBestTcw.clone();
        }
    }

    return cv::Mat();
}

// 使用新的内点来继续对位姿进行精求解
bool PnPsolver::Refine()
{
    //先备份一下之前最好的内点数据
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    // 然后……重新根据这些点构造用于RANSAC迭代的匹配关系
    // 分配空间
    // NOTE 注意这里其实的点应该是大于4个的,因为如果这里求精不成功,那么退出到上一层的迭代函数中的时候,这个 set_maximum_number_of_correspondences 并不会被重新上设定
    set_maximum_number_of_correspondences(vIndices.size());

    // 清空已有的匹配点的计数,为新的一次迭代作准备
    reset_correspondences();

    //添加匹配关系
    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();

    // 通过CheckInliers函数得到那些inlier点用来提纯 -- 其实应该说是通过提纯的过程，哪些点被再一次标注为了内点
    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    // 如果达到了要求
    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,mRi);
        cv::Mat tcw(3,1,CV_64F,mti);
        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

    return false;
}

/**
 * @brief 通过之前求解的位姿来进行3D-2D投影，统计内点数目
 *
 */

void PnPsolver::CheckInliers()
{
    mnInliersi=0;

    //遍历当前帧中说有的匹配点
    for(int i=0; i<N; i++)
    {
        // 取出对应的3D点和2D点
        cv::Point3f P3Dw = mvP3Dw[i];
        cv::Point2f P2D = mvP2D[i];

        // 将3D点转换成平面2D点
        //  | u |        |fu  0  u|    |x|
        //  | V | = (1/Z)|0  fv  v|（R*|y|+ t）
        //  | 1 |        |0   0  0|    |z|
        float Xc = mRi[0][0]*P3Dw.x+mRi[0][1]*P3Dw.y+mRi[0][2]*P3Dw.z+mti[0];
        float Yc = mRi[1][0]*P3Dw.x+mRi[1][1]*P3Dw.y+mRi[1][2]*P3Dw.z+mti[1];
        float invZc = 1/(mRi[2][0]*P3Dw.x+mRi[2][1]*P3Dw.y+mRi[2][2]*P3Dw.z+mti[2]);

        double ue = uc + fu * Xc * invZc;
        double ve = vc + fv * Yc * invZc;

        float distX = P2D.x-ue;
        float distY = P2D.y-ve;
        //计算误差
        float error2 = distX*distX+distY*distY;
        //当误差小于阀值的时候认为是内点
        if(error2<mvMaxError[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }
}


void PnPsolver::set_maximum_number_of_correspondences(int n)
{
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pcs != 0) delete [] pcs;

    maximum_number_of_correspondences = n;
    pws = new double[3 * maximum_number_of_correspondences];
    us = new double[2 * maximum_number_of_correspondences];
    alphas = new double[4 * maximum_number_of_correspondences];
    pcs = new double[3 * maximum_number_of_correspondences];
  }
}

void PnPsolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}

/**
 * @brief 将给定的3D,2D点的数据压入到数组中
 *
 * @param[in] X       3D点X坐标
 * @param[in] Y       3D点Y坐标
 * @param[in] Z       3D点Z坐标
 * @param[in] u       3D点对应2D点的横坐标
 * @param[in] v       3D点对应2D点的纵坐标
 */

void PnPsolver::add_correspondence(double X, double Y, double Z, double u, double v)
{
  pws[3 * number_of_correspondences    ] = X;
  pws[3 * number_of_correspondences + 1] = Y;
  pws[3 * number_of_correspondences + 2] = Z;

  us[2 * number_of_correspondences    ] = u;
  us[2 * number_of_correspondences + 1] = v;

  number_of_correspondences++;
}

/**
 * @brief 从给定的匹配点中计算出四个控制点
 *
 */

//EPnP算法的第一步
//现有四个三维点坐标pws
//求得四个控制点坐标cws
//第一个是质心坐标（四个三维点坐标的平均值）
//然后计算PW0=|pws0-cws0|
//           |pws1-cws0|
//           |pws2-csw0|
//           |pws3-cws0|
// 然后计算PW0t*PW0t得到一个3*3矩阵，然后进行SVD分解，得到特征值和特征向量
// 然后计算另外三个控制点
// cws1 = cws0 + sqrt(lamda0/n)*V0;
// cws2 = cws0 + sqrt(lamda1/n)*V1;
// cws3 = cws0 + sqrt(lamda2/n)*V2;
void PnPsolver::choose_control_points(void)
{
  // Take C0 as the reference points centroid:
  // Step 1：第一个控制点：参与PnP计算的参考3D点的质心（均值）
  // cws[4][3] 存储控制点在世界坐标系下的坐标，第一维表示是哪个控制点，第二维表示是哪个坐标(x,y,z)
  // 计算前先把第1个控制点坐标清零
    cws[0][0] = cws[0][1] = cws[0][2] = 0;

  // 遍历每个匹配点中世界坐标系3D点，然后对每个坐标轴加和
    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            cws[0][j] += pws[3 * i + j];


    // 再对每个轴上取均值(质心)
    for(int j = 0; j < 3; j++)
        cws[0][j] /= number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  // Step 2：计算其它三个控制点，C1, C2, C3通过特征值分解得到
  // ref: https://www.zhihu.com/question/38417101

  // 将所有的3D参考点写成矩阵，(number_of_correspondences * 3)的矩阵
    CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

    double pw0tpw0[3 * 3], dc[3], uct[3 * 3];           // 下面变量的数据区
    CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);   // PW0^T * PW0，为了进行特征值分解
    CvMat DC      = cvMat(3, 1, CV_64F, dc);            //特征值
    CvMat UCt     = cvMat(3, 3, CV_64F, uct);           //特征向量

    // Step 2.1：将存在pws中的参考3D点减去第一个控制点(均值中心)的坐标（相当于把第一个控制点作为原点）, 并存入PW0
    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j];

    //计算PW0t*PW0
    cvMulTransposed(PW0, &PW0tPW0, 1);

  //SVD分解
    cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

    cvReleaseMat(&PW0);

    // Step 2.3：得到C1, C2, C3三个3D控制点，最后加上之前减掉的第一个控制点这个偏移量
    // ?这里的循环次数不应写成4,而应该是变量 number_of_correspondences，虽然number_of_correspondences的默认值也为4
    for(int i = 1; i < 4; i++) {
        double k = sqrt(dc[i - 1] / number_of_correspondences);
            for(int j = 0; j < 3; j++)
        cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
    }
}

/**
 * @brief 求解世界坐标系下四个控制点的系数alphas，在相机坐标系下系数不变
 *
 */

// pwsi = [xi, yi, zi]t = [cws0, cws1, cws2, cws3]*[ai1, ai2, ai3, ai4]t 而且ai1到ai4的和为1；
// 可得 [xi - cws0x, yi - cws0y, zi - cws0z] = [cws1 - cws0, cws2 - cws0, cws3 - cws0] * [ai2, ai3, ai4]t
void PnPsolver::compute_barycentric_coordinates(void)
{
    // pws为世界坐标系下3D参考点的坐标
    // cws1 cws2 cws3 cws4为世界坐标系下四个控制点的坐标
    // alphas 四个控制点的系数，每一个pws，都有一组alphas与之对应
    double cc[3 * 3], cc_inv[3 * 3];
    CvMat CC     = cvMat(3, 3, CV_64F, cc);    // 除第1个控制点外，另外3个控制点在控制点坐标系下的坐标
    CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);  // 上面这个矩阵的逆矩阵

    // Step 1：第一个控制点在质心的位置，后面三个控制点减去第一个控制点的坐标（以第一个控制点为原点）
    // 减去质心后得到x y z轴
    //
    // cws的排列 |cws1_x cws1_y cws1_z|  ---> |cws1|
    //          |cws2_x cws2_y cws2_z|       |cws2|
    //          |cws3_x cws3_y cws3_z|       |cws3|
    //          |cws4_x cws4_y cws4_z|       |cws4|
    //
    // cc的排列  |cc2_x cc3_x cc4_x|  --->|cc2 cc3 cc4|
    //          |cc2_y cc3_y cc4_y|
    //          |cc2_z cc3_z cc4_z|

    // 将后面3个控制点cws 去重心后 转化为 cc
    for(int i = 0; i < 3; i++)                         //哪一个轴
        for(int j = 1; j < 4; j++)                      //哪一个控制点
            cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

    cvInvert(&CC, &CC_inv, CV_SVD);
    double * ci = cc_inv;
    // pi[]-cws[0][]表示去质心
    // a0,a1,a2,a3 对应的是四个控制点的齐次重心坐标
    // 每一个三维点都有自己的系数
    for(int i = 0; i < number_of_correspondences; i++) {
        double * pi = pws + 3 * i;     // pi指向第i个3D点的首地址
        double * a = alphas + 4 * i;   // a指向第i个控制点系数alphas的首地址

        for(int j = 0; j < 3; j++)
            a[1 + j] =
	        ci[3 * j    ] * (pi[0] - cws[0][0]) +
	        ci[3 * j + 1] * (pi[1] - cws[0][1]) +
	        ci[3 * j + 2] * (pi[2] - cws[0][2]);
        a[0] = 1.0f - a[1] - a[2] - a[3];
    }
}

void PnPsolver::fill_M(CvMat * M,
		  const int row, const double * as, const double u, const double v)
{
    double * M1 = M->data.db + row * 12;
    double * M2 = M1 + 12;

    //|a11fu, 0, a11(uc-u1), a12fu, 0, a12(uc-u1), a13fu, 0, a13(uc-u1), a14fu, 0, a14(uc-u1)|
    //|0, a11fv, a11(vc-v1), 0, a12fv, a12(vc-v1), 0, a13fv, a13(vc-v1), 0, a14fv, a14(vc-v1)|
    for(int i = 0; i < 4; i++) {
        M1[3 * i    ] = as[i] * fu;
        M1[3 * i + 1] = 0.0;
        M1[3 * i + 2] = as[i] * (uc - u);

        M2[3 * i    ] = 0.0;
        M2[3 * i + 1] = as[i] * fv;
        M2[3 * i + 2] = as[i] * (vc - v);
    }
}

/**
 * @brief 通过给出的beta和vi,计算控制点在相机坐标系下的坐标
 * @param[in] betas       beta
 * @param[in] ut          其实是vi
 */

void PnPsolver::compute_ccs(const double * betas, const double * ut)
{
    // 清空控制点
    for(int i = 0; i < 4; i++)
        ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

    // 根据前面计算的beta和v计算控制点坐标
    for(int i = 0; i < 4; i++) {
        // 注意这里传过来的向量ut中,最后的部分才是v,依次是  x  x  x  ... x v4 v3 v2 v1
        // 这里就是在最后面一次取出 v1 ~ v4
        const double * v = ut + 12 * (11 - i);
        for(int j = 0; j < 4; j++)    // j表示当前计算的是第几个控制点
            for(int k = 0; k < 3; k++)   // k表示当前计算的是控制点的哪个坐标
	    ccs[j][k] += betas[i] * v[3 * j + k];
    }
}

/**
 * @brief 根据相机坐标系下控制点坐标ccs 和控制点系数 alphas（通过世界坐标系下3D点计算得到），得到相机坐标系下3D点坐标 pcs
 * 过程可以参考 https://blog.csdn.net/jessecw79/article/details/82945918
 */
void PnPsolver::compute_pcs(void)
{
    // 遍历所有的空间点
    for(int i = 0; i < number_of_correspondences; i++) {
        double * a = alphas + 4 * i;
        double * pc = pcs + 3 * i;

        for(int j = 0; j < 3; j++)
            pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
    }
}

/**
 * @brief 使用EPnP算法计算相机的位姿.其中匹配点的信息由类的成员函数给定
 * @param[out] R    求解位姿里的旋转矩阵
 * @param[out] T    求解位姿里的平移向量
 * @return double   使用这对旋转和平移的时候, 匹配点对的平均重投影误差
 */

// https://zhuanlan.zhihu.com/p/149017276
//s* [u, v, 1]t = Kp = |fu, 0, cx|
//                     |0, fx, cy|*∑aij*[xj, yj, zj]t
//                     |0, 0,  1|
double PnPsolver::compute_pose(double R[3][3], double t[3])
{
    //EPnP第一步，选取控制点
    choose_control_points();
    //求解世界坐标系下四个控制点的系数alphas，在相机坐标系下系数不变
    compute_barycentric_coordinates();

    // Step 3：构造M矩阵，EPnP原始论文中公式(3)(4)-->(5)(6)(7); 矩阵的大小为 2n*12 ,n 为使用的匹配点的对数
    CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);

    for(int i = 0; i < number_of_correspondences; i++)
        fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

    double mtm[12 * 12], d[12], ut[12 * 12];
    CvMat MtM = cvMat(12, 12, CV_64F, mtm);
    CvMat D   = cvMat(12,  1, CV_64F, d);    //特征值
    CvMat Ut  = cvMat(12, 12, CV_64F, ut);  //特征向量

    cvMulTransposed(M, &MtM, 1);
    cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T); //对MtM进行SVD分解
    cvReleaseMat(&M);

    // 论文中13式中的L
    double l_6x10[6 * 10], rho[6];
    CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
    // 论文中13式的ρ
    CvMat Rho    = cvMat(6,  1, CV_64F, rho);

    compute_L_6x10(ut, l_6x10);
    compute_rho(rho);


    // 分情况计算N=2,3,4时能够求解得到的相机位姿R,t并且得到平均重投影误差
    double Betas[4][4],         // 本质上就四个beta1~4,但是这里有四种情况(第一维度表示)
    rep_errors[4];       // 重投影误差
    double Rs[4][3][3],         //每一种情况迭代优化后得到的旋转矩阵
    ts[4][3];            //每一种情况迭代优化后得到的平移向量

    // 不管什么情况，都假设论文中N=4，并求解部分betas（如果全求解出来会有冲突）
    // 通过优化得到剩下的 betas
    // 最后计算R t

    // N=4的情况, 估计出b的值
    find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
    //高斯近似求解
    gauss_newton(&L_6x10, &Rho, Betas[1]);

    rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

    // N=2的情况, 估计出b的值
    find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
    //高斯近似求解
    gauss_newton(&L_6x10, &Rho, Betas[2]);
    rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

    // N=3的情况, 估计出b的值
    find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
    //高斯近似求解
    gauss_newton(&L_6x10, &Rho, Betas[3]);
    rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

    int N = 1;
    if (rep_errors[2] < rep_errors[1]) N = 2;
    if (rep_errors[3] < rep_errors[N]) N = 3;

    //把N=2， 3， 4，比较，重投影误差，取出最优解，把R，t拷贝到R t。
    copy_R_and_t(Rs[N], ts[N], R, t);

    //返回从投影误差
    return rep_errors[N];
}

void PnPsolver::copy_R_and_t(const double R_src[3][3], const double t_src[3],
			double R_dst[3][3], double t_dst[3])
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++)
            R_dst[i][j] = R_src[i][j];
        t_dst[i] = t_src[i];
    }
}

double PnPsolver::dist2(const double * p1, const double * p2)
{
    return
        (p1[0] - p2[0]) * (p1[0] - p2[0]) +
        (p1[1] - p2[1]) * (p1[1] - p2[1]) +
        (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double PnPsolver::dot(const double * v1, const double * v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/**
 * @brief 计算在给定位姿的时候的3D点投影误差
 * @param[in] R      给定旋转
 * @param[in] t      给定平移
 * @return double    重投影误差,是平均到每一对匹配点上的误差
 */
double PnPsolver::reprojection_error(const double R[3][3], const double t[3])
{
    //统计误差的平方
    double sum2 = 0.0;

    // 遍历每个3D点
    for(int i = 0; i < number_of_correspondences; i++) {
        // 指针定位
        double * pw = pws + 3 * i;
        // 计算这个3D点在相机坐标系下的坐标,逆深度表示
        double Xc = dot(R[0], pw) + t[0];
        double Yc = dot(R[1], pw) + t[1];
        double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        // 计算投影点
        double ue = uc + fu * Xc * inv_Zc;
        double ve = vc + fv * Yc * inv_Zc;
        // 计算投影点与匹配2D点的欧氏距离的平方
        double u = us[2 * i], v = us[2 * i + 1];
        // 得到其欧式距离并累加
        sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
    }
    // 返回累计误差
    return sum2 / number_of_correspondences;
}

/**
 * @brief 用3D点在世界坐标系和相机坐标系下对应的坐标，用ICP求取R t
 * @param[out] R   旋转
 * @param[out] t   平移
 */

void PnPsolver::estimate_R_and_t(double R[3][3], double t[3])
{
    double pc0[3], pw0[3];//定义相机坐标的质心，和世界坐标的质心

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        const double * pc = pcs + 3 * i;
        const double * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            pc0[j] += pc[j];
            pw0[j] += pw[j];
        }
    }
    for(int j = 0; j < 3; j++) {
        pc0[j] /= number_of_correspondences;  //求平均值，即质心
        pw0[j] /= number_of_correspondences;  //求平均值，即质心
    }

    double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
    CvMat ABt   = cvMat(3, 3, CV_64F, abt);
    CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
    CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
    CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

    // 构造矩阵H=B^T*A,不过这里是隐含的构造
    cvSetZero(&ABt);
    // 遍历每一个3D点
    //W = qi*qit
    for(int i = 0; i < number_of_correspondences; i++) {
        double * pc = pcs + 3 * i;
        double * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
            abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
            abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
        }
    }

    //对ABt 即公式里面的 W = U∑Vt

    cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

    // Step 4 R=U*V^T, 并且进行合法性检查
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

    // 注意在得到了R以后,需要保证 det(R)=1>0
    const double det =
        R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
        R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

    // 如果小于0那么就要加负号
    if (det < 0) {
        R[2][0] = -R[2][0];
        R[2][1] = -R[2][1];
        R[2][2] = -R[2][2];
    }

    // Step 5 根据R计算t
    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);
}

void PnPsolver::print_pose(const double R[3][3], const double t[3])
{
    cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
    cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
    cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

// 保持所有点在相机坐标系下的深度为正,调整符号
// 当z < 0时，对x, y, z分量同时取相反数，保证z分量是正数。
void PnPsolver::solve_for_sign(void)
{
    // 根据第一个3D点在当前相机坐标系下的深度,调整所有的3D点的深度为正(因为正常地来讲,这些3D点都应该是在相机前面的)
    // 如果第一个点的深度是负的话
    if (pcs[2] < 0.0) {
        // 先调整控制点的坐标
        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 3; j++)
	    ccs[i][j] = -ccs[i][j];

            // 然后调整3D点的坐标
        for(int i = 0; i < number_of_correspondences; i++) {
            pcs[3 * i    ] = -pcs[3 * i];
            pcs[3 * i + 1] = -pcs[3 * i + 1];
            pcs[3 * i + 2] = -pcs[3 * i + 2];
        }
    }
}

/**
 * @brief 根据已经得到的控制点在当前相机坐标系下的坐标来恢复出相机的位姿
 * @param[in]  ut         vi
 * @param[in]  betas      betas
 * @param[out] R          计算得到的相机旋转R
 * @param[out] t          计算得到的相机位置t
 * @return double         使用这个位姿,所得到的重投影误差
 */

double PnPsolver::compute_R_and_t(const double * ut, const double * betas,
			     double R[3][3], double t[3])
{
    compute_ccs(betas, ut);
    compute_pcs();

    solve_for_sign();

    estimate_R_and_t(R, t);

    return reprojection_error(R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

/**
 * @brief 计算N=4时候的粗糙近似解，暴力将其他量置为0
 *
 * @param[in]  L_6x10  矩阵L
 * @param[in]  Rho     非齐次项 \rho, 列向量
 * @param[out] betas   计算得到的beta
 */

void PnPsolver::find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
    // 计算N=4时候的粗糙近似解，暴力将其他量置为0
    // betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]  -- L_6x10中每一行的内容
    // betas_approx_1 = [B11 B12     B13         B14            ]  -- L_6x4 中一行提取出来的内容
    double l_6x4[6 * 4], b4[4];
    CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
    CvMat B4    = cvMat(4, 1, CV_64F, b4);

    // 提取L_6x10矩阵中每行的第0,1,3,6个元素，得到L_6x4
    for(int i = 0; i < 6; i++) {
        cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
        cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
        cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
        cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
    }

    // SVD方式求解方程组 L_6x4 * B4 = Rho
    cvSolve(&L_6x4, Rho, &B4, CV_SVD);

    // 得到的解是 b00 b01 b02 b03 因此解出来b0即可
    if (b4[0] < 0) {
        betas[0] = sqrt(-b4[0]);
        betas[1] = -b4[1] / betas[0];
        betas[2] = -b4[2] / betas[0];
        betas[3] = -b4[3] / betas[0];
    } else {
        betas[0] = sqrt(b4[0]);
        betas[1] = b4[1] / betas[0];
        betas[2] = b4[2] / betas[0];
        betas[3] = b4[3] / betas[0];
    }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void PnPsolver::find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
    double l_6x3[6 * 3], b3[3];
    CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);
    CvMat B3     = cvMat(3, 1, CV_64F, b3);

    for(int i = 0; i < 6; i++) {
        cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
        cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
        cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
    }

    cvSolve(&L_6x3, Rho, &B3, CV_SVD);

    if (b3[0] < 0) {
        betas[0] = sqrt(-b3[0]);
        betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
    } else {
        betas[0] = sqrt(b3[0]);
        betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
    }

    if (b3[1] < 0) betas[0] = -betas[0];

    betas[2] = 0.0;
    betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

/**
 * @brief 计算N=3时候的粗糙近似解，暴力将其他量置为0
 *
 * @param[in]  L_6x10  矩阵L
 * @param[in]  Rho     非齐次项 \rho, 列向量
 * @param[out] betas   计算得到的beta
 */

void PnPsolver::find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
    double l_6x5[6 * 5], b5[5];
    CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
    CvMat B5    = cvMat(5, 1, CV_64F, b5);

    for(int i = 0; i < 6; i++) {
        cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
        cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
        cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
        cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
        cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
    }

    cvSolve(&L_6x5, Rho, &B5, CV_SVD);

    //开方求得b1
    if (b5[0] < 0) {
        betas[0] = sqrt(-b5[0]);
        betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
    } else {
        betas[0] = sqrt(b5[0]);
        betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
    }
    if (b5[1] < 0) betas[0] = -betas[0];
    //求解b3
    betas[2] = b5[3] / betas[0];
    // N=3的时候没有B4
    betas[3] = 0.0;
}

/**
 * @brief 计算矩阵L,论文式13中的L矩阵,不过这里的是按照N=4的时候计算的
 *
 * @param[in]  ut               特征值分解之后得到的12x12特征矩阵
 * @param[out] l_6x10           计算的L矩阵结果，维度6x10
 */
void PnPsolver::compute_L_6x10(const double * ut, double * l_6x10)
{
    // Step 1 获取最后4个零特征值对应的4个12x1的特征向量
    const double * v[4];

    // 对应EPnP里N=4的情况。直接取特征向量的最后4行
    // 以这里的v[0]为例，它是12x1的向量，会拆成4个3x1的向量v[0]^[0]，v[0]^[1]，v[0]^[1]，v[0]^[3]，对应4个相机坐标系控制点
    v[0] = ut + 12 * 11;
    // v[0] : v[0][0]~v[0][2]  => v[0]^[0]  , * \beta_0 = c0  (理论上)
    //        v[0][3]~v[0][5]  => v[0]^[1]  , * \beta_0 = c1
    //        v[0][6]~v[0][8]  => v[0]^[2]  , * \beta_0 = c2
    //        v[0][9]~v[0][11] => v[0]^[3]  , * \beta_0 = c3
    v[1] = ut + 12 * 10;
    v[2] = ut + 12 *  9;
    v[3] = ut + 12 *  8;

    // Step 2 提前计算中间变量dv
    // dv表示中间变量，是difference-vector的缩写
    // 4 表示N=4时对应的4个12x1的向量v, 6 表示4对点一共有6种两两组合的方式，3 表示v^[i]是一个3维的列向量
    double dv[4][6][3];
    // v112表示v11,和v12的差，对应控制点c1和c2的差
    // dv[0][0]:v112 dv[1][0]: v212 dv[2][0]: v312 dv[3][0]: v412
    // dv[0][1]:v113 dv[1][1]: v213 dv[2][1]: v313 dv[3][1]: v413
    // dv[0][2]:v114 dv[1][2]: v214 dv[2][2]: v314 dv[3][2]: v414
    // dv[0][3]:v123 dv[1][3]: v223 dv[2][3]: v323 dv[3][3]: v423
    // dv[0][4]:v124 dv[1][4]: v224 dv[2][4]: v324 dv[3][4]: v424
    // dv[0][5]:v134 dv[1][5]: v234 dv[2][5]: v334 dv[3][5]: v434
    for(int i = 0; i < 4; i++) {
        int a = 0, b = 1;
        for(int j = 0; j < 6; j++) {
            dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
            dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
            dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

            b++;
            if (b > 3) {
	            a++;
	            b = a + 1;
            }
        }
    }

    for(int i = 0; i < 6; i++) {
        double * row = l_6x10 + 10 * i;

        // v112       |    v113    |   v114     |     v123  |    v124    |   v134   |
        row[0] =        dot(dv[0][i], dv[0][i]);  //*b11
        // v112, v212|  v113, v213 | v114, v214 | v123, v223| v124, v224| v123, v234|
        row[1] = 2.0f * dot(dv[0][i], dv[1][i]);  //*b12
        // v212      |    V213     |  v214   |  v223      |  v224  |   v234  |
        row[2] =        dot(dv[1][i], dv[1][i]);  //*b22
        // v112, v312| v113, v313 | v114， v314| v123, v323| v124, v324| v124, v324|
        row[3] = 2.0f * dot(dv[0][i], dv[2][i]);  //*b13
        // v212, v312|
        row[4] = 2.0f * dot(dv[1][i], dv[2][i]);  //*b23
        // v312      |
        row[5] =        dot(dv[2][i], dv[2][i]);  //*b33
        // v112, v412|
        row[6] = 2.0f * dot(dv[0][i], dv[3][i]);  //*b14
        // v212, v412|
        row[7] = 2.0f * dot(dv[1][i], dv[3][i]);  //*b24
        // v312, v412|
        row[8] = 2.0f * dot(dv[2][i], dv[3][i]);  //*b34
        // v412, v412|
        row[9] =        dot(dv[3][i], dv[3][i]);  //*b44
    }
}

/**
 * @brief 计算四个控制点任意两点间的距离，总共6个距离，对应论文式13中的向量\rho
 * @param[in] rho  计算结果
 */

void PnPsolver::compute_rho(double * rho)
{
    // 四个点两两组合一共有6中组合方式: 01 02 03 12 13 23
    rho[0] = dist2(cws[0], cws[1]);
    rho[1] = dist2(cws[0], cws[2]);
    rho[2] = dist2(cws[0], cws[3]);
    rho[3] = dist2(cws[1], cws[2]);
    rho[4] = dist2(cws[1], cws[3]);
    rho[5] = dist2(cws[2], cws[3]);
}

/**
 * @brief 计算高斯牛顿法优化时,增量方程中的系数矩阵和非齐次项
 * @param[in]  l_6x10 L矩阵
 * @param[in]  rho    Rho矩向量
 * @param[in]  cb     当前次迭代得到的beta1~beta4
 * @param[out] A      计算得到的增量方程中的系数矩阵
 * @param[out] b      计算得到的增量方程中的非齐次项
 */



void PnPsolver::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
					double betas[4], CvMat * A, CvMat * b)
{
    // 以下推导就是求解一阶雅克比矩阵
    for(int i = 0; i < 6; i++) {
        //该处的目标函数为Error(β)=∑（||cci-ccj||^2 - ||cwi-cwj||^2）
        const double * rowL = l_6x10 + i * 10;
        double * rowA = A->data.db + i * 4;
        //计算当前的雅克比，也就是J6*4的一行
        rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
        rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
        rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
        rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

        cvmSet(b, i, 0, rho[i] -
	    (
	        rowL[0] * betas[0] * betas[0] +
	        rowL[1] * betas[0] * betas[1] +
	        rowL[2] * betas[1] * betas[1] +
	        rowL[3] * betas[0] * betas[2] +
	        rowL[4] * betas[1] * betas[2] +
	        rowL[5] * betas[2] * betas[2] +
	        rowL[6] * betas[0] * betas[3] +
	        rowL[7] * betas[1] * betas[3] +
	        rowL[8] * betas[2] * betas[3] +
	        rowL[9] * betas[3] * betas[3]
	    ));
    }
}

/**
 * @brief 对计算出来的Beta结果进行高斯牛顿法优化,求精. 过程参考EPnP论文中式(15)
 *
 * @param[in] L_6x10
 * @param[in] Rho
 * @param[in] betas
 */

void PnPsolver::gauss_newton(const CvMat * L_6x10, const CvMat * Rho,
			double betas[4])
{
    //只进行5次迭代
    const int iterations_number = 5;
    /** 这里是求解增量方程组Ax=B,其中的x就是增量. 根据论文中的式15,可以得到优化的目标函数为:
   *  Error(β)=∑（||cci - ccj||^2 - ||cwi - cwj||^2）
   *
   * 而根据高斯牛顿法,增量方程为:
   *   Hδx = g
   * 也就是:(参考视觉SLAM十四讲第一版P112式6.21 6.22)
   *  Jt*J*δx = -J*f(x)
   * 不过这里在计算的时候将等式左右两边的雅克比 J 都给约去了,得到精简后的增量方程:
   * Jtδx =-f(x)
   * 然后分别对应为程序代码中的系数矩阵A，J和非齐次项Bf（x）.
   */

    double a[6*4], b[6], x[4];
    CvMat A = cvMat(6, 4, CV_64F, a);
    CvMat B = cvMat(6, 1, CV_64F, b);
    CvMat X = cvMat(4, 1, CV_64F, x);

    for(int k = 0; k < iterations_number; k++) {
        compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db,
				 betas, &A, &B);
        // 根据QR分解得到x 即为δx
        qr_solve(&A, &B, &X);

        for(int i = 0; i < 4; i++)
            //betas加上增量
            betas[i] += x[i];
    }
}

/**
 * @brief 使用QR分解来求解增量方程
 * @param[in]  A   洗漱矩阵
 * @param[in]  b   非齐次项
 * @param[out] X   增量
 */

void PnPsolver::qr_solve(CvMat * A, CvMat * b, CvMat * X)
{
    static int max_nr = 0;
    static double * A1, * A2;

    const int nr = A->rows;   // 系数矩阵A的行数  6
    const int nc = A->cols;   // 系数矩阵A的列数  4

    // 判断是否需要重新分配A1 A2的内存区域
    if (max_nr != 0 && max_nr < nr) {
        // 如果 max_nr != 0 说明之前已经创建了一个 last_max_nr < nr 的数组,不够我们现在使用了,需要重新分配内存;但是在重新分配之前我们需要先删除之前创建的内容
        delete [] A1;
        delete [] A2;
    }
    if (max_nr < nr) {
        max_nr = nr;
        A1 = new double[nr];
        A2 = new double[nr];
    }

    double * pA = A->data.db,  // 指向系数矩阵A的数据区
           * ppAkk = pA;  // 一直都会指向对角线上的元素

    // 对系数矩阵的列展开遍历
    for(int k = 0; k < nc; k++) {
        double * ppAik = ppAkk,     // 只是辅助下面的for循环中,遍历对角线元素下的当前列的所有元素
               eta = fabs(*ppAik);  // 存储当前列对角线元素下面的所有元素绝对值的最大值

        // 遍历当前对角线约束下,当前列的所有元素,并且找到它们中的最大的绝对值
        for(int i = k + 1; i < nr; i++) {
            double elt = fabs(*ppAik);
            if (eta < elt) eta = elt;
            ppAik += nc; // 指向下一列
        }

        //? 判断靠谱不? 由于系数矩阵是雅克比,并且代价函数中的L元素都是二次项的形式,所以原则上都应该是大于0的
        if (eta == 0) {
            A1[k] = A2[k] = 0.0;
            cerr << "God damnit, A is singular, this shouldn't happen." << endl;
            return;
        } else {
            // 感觉这里面使用的ription provided.是数值分析中的计算方法,和矩阵论中的定义的算法还是不一样的
            // 注意在这里面,ppAik被重ription provided.定义了,在这个结构中以这里定义的这个为准
            double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
            for(int i = k; i < nr; i++) {
	            *ppAik *= inv_eta;
	            sum += *ppAik * *ppAik;
	            ppAik += nc;
            }
            double sigma = sqrt(sum);
            if (*ppAkk < 0)
	            sigma = -sigma;
            *ppAkk += sigma;
            A1[k] = sigma * *ppAkk;
            A2[k] = -eta * sigma;
            for(int j = k + 1; j < nc; j++) {
	            double * ppAik = ppAkk, sum = 0;
	            for(int i = k; i < nr; i++) {
	                sum += *ppAik * ppAik[j - k];
	                ppAik += nc;
	            }
	            double tau = sum / A1[k];
	            ppAik = ppAkk;
	            for(int i = k; i < nr; i++) {
	                ppAik[j - k] -= tau * *ppAik;
	                ppAik += nc;
	                }
            }
        }
        ppAkk += nc + 1;
    }

    // b <- Qt b
    double * ppAjj = pA, * pb = b->data.db;
    for(int j = 0; j < nc; j++) {
        double * ppAij = ppAjj, tau = 0;
        for(int i = j; i < nr; i++)	{
            tau += *ppAij * pb[i];
            ppAij += nc;
        }
        tau /= A1[j];
        ppAij = ppAjj;
        for(int i = j; i < nr; i++) {
            pb[i] -= tau * *ppAij;
            ppAij += nc;
        }
        ppAjj += nc + 1;
    }

    // X = R-1 b
    double * pX = X->data.db;
    pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
    for(int i = nc - 2; i >= 0; i--) {
        double * ppAij = pA + i * nc + (i + 1), sum = 0;

        for(int j = i + 1; j < nc; j++) {
            sum += *ppAij * pX[j];
            ppAij++;
        }
        pX[i] = (pb[i] - sum) / A2[i];
    }
}



void PnPsolver::relative_error(double & rot_err, double & transl_err,
			  const double Rtrue[3][3], const double ttrue[3],
			  const double Rest[3][3],  const double test[3])
{
    double qtrue[4], qest[4];

    mat_to_quat(Rtrue, qtrue);
    mat_to_quat(Rest, qest);

    double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
             sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
             sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    rot_err = min(rot_err1, rot_err2);

    transl_err =
        sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	    (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	    (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
        sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void PnPsolver::mat_to_quat(const double R[3][3], double q[4])
{
    double tr = R[0][0] + R[1][1] + R[2][2];
    double n4;

    if (tr > 0.0f) {
        q[0] = R[1][2] - R[2][1];
        q[1] = R[2][0] - R[0][2];
        q[2] = R[0][1] - R[1][0];
        q[3] = tr + 1.0f;
        n4 = q[3];
    } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
        q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
        q[1] = R[1][0] + R[0][1];
        q[2] = R[2][0] + R[0][2];
        q[3] = R[1][2] - R[2][1];
        n4 = q[0];
    } else if (R[1][1] > R[2][2]) {
        q[0] = R[1][0] + R[0][1];
        q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
        q[2] = R[2][1] + R[1][2];
        q[3] = R[2][0] - R[0][2];
        n4 = q[1];
    } else {
        q[0] = R[2][0] + R[0][2];
        q[1] = R[2][1] + R[1][2];
        q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
        q[3] = R[0][1] - R[1][0];
        n4 = q[2];
    }
    double scale = 0.5f / double(sqrt(n4));

    q[0] *= scale;
    q[1] *= scale;
    q[2] *= scale;
    q[3] *= scale;
}

} //namespace ORB_SLAM
