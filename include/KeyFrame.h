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
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);   // 添加共视关键帧。只在UpdateConnections内部被调用，应设为private
    void EraseConnection(KeyFrame* pKF);                    // 删除共视关键帧。只在SetBadFlag内部被调用，应设为private
    void UpdateConnections();                       // 基于当前关键帧对地图点的观测构造共视图, 应为private
    void UpdateBestCovisibles();                    // 只在AddConnection和EraseConnection内部被调用，应设为private
    std::set<KeyFrame *> GetConnectedKeyFrames();                       // get方法
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();              // get方法
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);  // get方法
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);         // get方法
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);       // 添加子节点,mspChildrens的set方法
    void EraseChild(KeyFrame* pKF);     // 删除子节点,mspChildrens的set方法
    void ChangeParent(KeyFrame* pKF);   // mpParent的set方法
    std::set<KeyFrame*> GetChilds();    // mspChildrens的get方法
    KeyFrame* GetParent();              // mpParent的get方法
    bool hasChild(KeyFrame* pKF);       // 判断mspChildrens是否为空

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);    // mspLoopEdge的set函数
    std::set<KeyFrame*> GetLoopEdges(); // mspLoopEdge的get函数

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();         // mbNotErase的set方法
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();          // 真的执行删除
    bool isBad();               // mbBad的get方法

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the fog estimation
    long unsigned int mnFogForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
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
    std::vector<cv::Mat> mvImagePyramid;        // 图像金字塔每层的图像
    std::vector<cv::Mat> mvImagePyramidColour;  // 图像金字塔每层的图像(彩色图)
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    bool mbIsInputImageColour;       // 是否为彩色图像

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;        // 当前关键帧观测到的地图点列表

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;          // 当前关键帧的共视关键帧及权重
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;        // 所有共视关键帧,按权重从大到小排序
    std::vector<int> mvOrderedWeights;                          // 所有共视权重,按从大到小排序

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;             // 当前关键帧是否还未加入到生成树,构造函数中初始化为true,加入生成树后置为false
    KeyFrame* mpParent;                 // 当前关键帧在生成树中的父节点
    std::set<KeyFrame*> mspChildrens;   // 当前关键帧在生成树中的子节点列表
    std::set<KeyFrame*> mspLoopEdges;   // 和当前帧形成回环的关键帧集合

    // Bad flags
    bool mbNotErase;                    // 当前关键帧是否具有不被删除的特权,初值为false
    bool mbToBeErased;                  // 当前关键帧是否曾被豁免过删除,初值为false
    bool mbBad;                         // 标记是坏帧,初值为false

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H