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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "FogEstimation.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;
class FogEstimation;

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);           // mWorldPos的set方法
    cv::Mat GetWorldPos();                          // mWorldPos的get方法

    cv::Mat GetNormal();                            // mNormalVector的get方法
    KeyFrame* GetReferenceKeyFrame();               // mpRefKF的get方法

    std::map<KeyFrame*,size_t> GetObservations();   // mObservations的get方法
    int Observations();                             // nObs的get方法

    void AddObservation(KeyFrame* pKF,size_t idx);  // 添加当前地图点对某KeyFrame的观测
    void EraseObservation(KeyFrame* pKF);           // 删除当前地图点对某KeyFrame的观测

    int GetIndexInKeyFrame(KeyFrame* pKF);          // 查询当前地图点在某KeyFrame中的索引
    bool IsInKeyFrame(KeyFrame* pKF);               // 查询当前地图点是否在某KeyFrame中

    void SetBadFlag();                              // 删除当前地图点
    bool isBad();                                   // 查询当前地图点是否被删除(本质上就是查询mbBad)

    void Replace(MapPoint* pMP);                    // 使用地图点pMP替换当前地图点
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);                  // mnVisible加1
    void IncreaseFound(int n=1);                    // mnFound加1
    float GetFoundRatio();                          // 召回率(实际观测到该地图点的帧数 / 理论上应当观测到该地图点的帧数)
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();           // 计算mDescriptor

    cv::Mat GetDescriptor();                        // mDescriptor的get方法

    void UpdateNormalAndDepth();                    // 更新平均观测距离和方向

    float GetMinDistanceInvariance();               // mfMinDistance的get方法
    float GetMaxDistanceInvariance();               // mfMaxDistance的get方法
    int PredictScale(const float &currentDist, KeyFrame*pKF);   // 估计当前地图点在某Frame中对应特征点的金字塔层级
    int PredictScale(const float &currentDist, Frame* pF);      // 估计当前地图点在某Frame中对应特征点的金字塔层级

    float GetJCurrentEstimate() const;
    float GetJCurrentEstimate(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate2(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate3(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate4(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate5(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate6(FogEstimation::eIntensityMode intensity_mode) const;
    float GetJCurrentEstimate7(FogEstimation::eIntensityMode intensity_mode) const;
    cv::Vec4f GetJCurrentEstimateAllChannels(FogEstimation::eIntensityMode intensity_mode) const;
    cv::Vec4f GetJCurrentEstimateAllChannelsWoGc(FogEstimation::eIntensityMode intensity_mode) const;
    void UpdateJEstimates(KeyFrame* pKF, float J_hat);
    void UpdateJEstimates(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates2(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates3(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates4(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates5(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates6(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimates7(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    void UpdateJEstimatesAllChannels(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode, int nChannel);
    void UpdateJEstimatesAllChannelsWoGc(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode, int nChannel);

    // partial groundtruth
    float GetJCurrentEstimateBeta(FogEstimation::eIntensityMode intensity_mode) const;
    void UpdateJEstimatesBeta(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);
    float GetJCurrentEstimateA(FogEstimation::eIntensityMode intensity_mode) const;
    void UpdateJEstimatesA(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;                           // (应设为protected) 记录当前地图点被多少相机观测到, 单目帧每次观测加1,双目帧每次观测加2

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by the fog estimation
    long unsigned int mnFogForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:

    // Position in absolute coordinates
    cv::Mat mWorldPos;     // 地图点的世界坐标

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;      // 当前地图点在某KeyFrame中的索引。key值为某个关键帧的指针，value值为当前地图点在该关键帧中的索引（即在该关键帧成员变量std::vector<MapPoint*> mvpMapPoints中的索引）

    // Mean viewing direction
    cv::Mat mNormalVector;                         // 平均观测方向

    // Best descriptor to fast matching
    cv::Mat mDescriptor;                           // 当前关键点的特征描述子(所有描述子的中位数)

     // Reference KeyFrame
    KeyFrame* mpRefKF;                             // 当前地图点的参考关键帧

    // Tracking counters
    int mnVisible;                                 // 理论上应当观测到该地图点的帧数
    int mnFound;                                   // 实际观测到该地图点的帧数

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;                                    // 坏点标记
    MapPoint* mpReplaced;                          // 用来替换当前地图点的新地图点

    // Scale invariance distances
    float mfMinDistance;                           // 平均观测距离的下限
    float mfMaxDistance;                           // 平均观测距离的上限

    Map* mpMap;

    float mfJCurrentEstimate;
    std::map<KeyFrame*, float> mJEstimates;

    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate2;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates2;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate3;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates3;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate4;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates4;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate5;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates5;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate6;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates6;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimate7;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimates7;
    std::map<FogEstimation::eIntensityMode, cv::Vec4f> mAllModesJCurrentEstimateAllChannels;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, cv::Vec4f>> mAllModesJEstimatesAllChannels;
    std::map<FogEstimation::eIntensityMode, cv::Vec4f> mAllModesJCurrentEstimateAllChannelsWoGc;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, cv::Vec4f>> mAllModesJEstimatesAllChannelsWoGc;

    // partial groundtruth
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimateBeta;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimatesBeta;
    std::map<FogEstimation::eIntensityMode, float> mAllModesJCurrentEstimateA;
    std::map<FogEstimation::eIntensityMode, std::map<KeyFrame*, float>> mAllModesJEstimatesA;

    std::mutex mMutexPos;                          // mWorldPos的锁
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H