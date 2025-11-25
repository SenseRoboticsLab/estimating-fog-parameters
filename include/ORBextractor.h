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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;        // 图像金字塔每层的图像

    std::vector<cv::Mat> ComputePyramidColour(cv::Mat image);       // compute pyramid for colour image

protected:

    void ComputePyramid(cv::Mat image);
    void ComputePyramidMatlab(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;      // 所有层级提取到的特征点数之和金字塔层数, 在yaml中的ORBextractor.nFeatures
    double scaleFactor; // 图像金字塔相邻层级间的缩放系数, 在yaml中的ORBextractor.scaleFactor
    int nlevels;        // 金字塔层级数, 在yaml中的ORBextractor.nLevels
    int iniThFAST;      // 提取特征点的描述子门槛(高), 在yaml中的ORBextractor.iniThFAST
    int minThFAST;      // 提取特征点的描述子门槛(低), 在yaml中的ORBextractor.minThFAST

    std::vector<int> mnFeaturesPerLevel;    // 金字塔每层级中提取的特征点数,正比于图层边长,总和为nfeatures

    std::vector<int> umax;                  // 逼近圆的第一象限内圆周上每个v(纵)坐标对应的u(横)坐标

    std::vector<float> mvScaleFactor;       // 各层级的缩放系数
    std::vector<float> mvInvScaleFactor;    // 各层级缩放系数的倒数
    std::vector<float> mvLevelSigma2;       // 各层级缩放系数的平方
    std::vector<float> mvInvLevelSigma2;    // 各层级缩放系数的平方倒数
};

} //namespace ORB_SLAM

#endif
