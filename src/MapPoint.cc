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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnFogForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mfJCurrentEstimate(0.0f)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;

    for (int intensity_mode_int=FogEstimation::RAW; intensity_mode_int!=FogEstimation::REFD_REF_PRG; intensity_mode_int++)
    {
        FogEstimation::eIntensityMode intensity_mode = static_cast<FogEstimation::eIntensityMode>(intensity_mode_int);
        mAllModesJCurrentEstimate.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate2.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate3.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate4.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate5.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate6.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate7.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimateAllChannels.insert(make_pair(intensity_mode, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f)));
        mAllModesJCurrentEstimateAllChannelsWoGc.insert(make_pair(intensity_mode, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f)));
        mAllModesJCurrentEstimateBeta.insert(make_pair(intensity_mode, -1.0f));
    }
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0), mnFogForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mfJCurrentEstimate(0.0f)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    // pFrame是当前MapPoint的参考帧
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;

    for (int intensity_mode_int=FogEstimation::RAW; intensity_mode_int!=FogEstimation::REFD_REF_PRG; intensity_mode_int++)
    {
        FogEstimation::eIntensityMode intensity_mode = static_cast<FogEstimation::eIntensityMode>(intensity_mode_int);
        mAllModesJCurrentEstimate.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate2.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate3.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate4.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate5.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate6.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimate7.insert(make_pair(intensity_mode, -1.0f));
        mAllModesJCurrentEstimateAllChannels.insert(make_pair(intensity_mode, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f)));
        mAllModesJCurrentEstimateAllChannelsWoGc.insert(make_pair(intensity_mode, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f)));
        mAllModesJCurrentEstimateBeta.insert(make_pair(intensity_mode, -1.0f));
    }
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

// 向参考帧pKF中添加对本地图点的观测,本地图点在pKF中的编号为idx
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    // 如果已经添加过观测，返回
    // 注意：map不允许插入重复key的元素（而multimap可以），所以count值非0即1
    if(mObservations.count(pKF))
        return;
    // 如果没有添加过观测，记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
    // 等同于 mObservations.insert(make_pair(pKF, idx))
    mObservations[pKF]=idx;

    // 根据观测形式是单目（右图像为0）还是双目（右图像为非0）更新观测计数变量nObs
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

// 从参考帧pKF中移除本地图点
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 查找这个要删除的观测,根据单目和双目类型的不同从其中删除当前地图点的被观测次数
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)       // 右目特征点的横坐标非-1，是双目或RGBD点
                nObs-=2;
            else                            // 右目特征点的横坐标为-1，是单目点
                nObs--;

            mObservations.erase(pKF);       // 删除对该地图点的观测

            // 如果该keyFrame是参考帧，该Frame被删除后重新指定RefFrame为第一个观测到当前地图点的关键帧
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;     // ????参考帧指定得这么草率真的好么?

            // If only 2 observations or less, discard point
            // 当观测到该点的相机数目少于2时，丢弃该点(至少需要两个观测才能三角化)
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        // 告知可以观测到该MapPoint的Frame，该MapPoint已被删除
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;                                 // 标记mbBad,逻辑上删除当前地图点
        obs = mObservations;
        mObservations.clear();
    }
    // 删除关键帧对当前地图点的观测
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    // 在地图类上注册删除当前地图点,这里会发生内存泄漏
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    // 如果是同一地图点则跳过
    if(pMP->mnId==this->mnId)
        return;

    // step1. 逻辑上删除当前地图点
    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    // step2. 将当地图点的数据叠加到新地图点上
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    // step3. 删除当前地图点
    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    // step1. 获取地图点相关信息
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    // step2. 根据观测到该地图点的关键帧先求和再取平均计算平均观测方向
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    // step3. 根据参考关键帧计算平均观测距离
    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

float MapPoint::GetJCurrentEstimate() const
{
    return mfJCurrentEstimate;
}

float MapPoint::GetJCurrentEstimate(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate2(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate2.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate2.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate3(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate3.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate3.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate4(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate4.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate4.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate5(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate5.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate5.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate6(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate6.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate6.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimate7(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimate7.find(intensity_mode);
    if (it != mAllModesJCurrentEstimate7.end())
        return it->second;
    else
        return -1.0f;
}
cv::Vec4f MapPoint::GetJCurrentEstimateAllChannels(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimateAllChannels.find(intensity_mode);
    if (it != mAllModesJCurrentEstimateAllChannels.end())
        return it->second;
    else
        return cv::Vec4f(-1.0f, -1.0f, -1.0f,-1.0f);
}
cv::Vec4f MapPoint::GetJCurrentEstimateAllChannelsWoGc(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimateAllChannelsWoGc.find(intensity_mode);
    if (it != mAllModesJCurrentEstimateAllChannelsWoGc.end())
        return it->second;
    else
        return cv::Vec4f(-1.0f, -1.0f, -1.0f,-1.0f);
}
float MapPoint::GetJCurrentEstimateBeta(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimateBeta.find(intensity_mode);
    if (it != mAllModesJCurrentEstimateBeta.end())
        return it->second;
    else
        return -1.0f;
}
float MapPoint::GetJCurrentEstimateA(FogEstimation::eIntensityMode intensity_mode) const
{
    auto it = mAllModesJCurrentEstimateA.find(intensity_mode);
    if (it != mAllModesJCurrentEstimateA.end())
        return it->second;
    else
        return -1.0f;
}

void MapPoint::UpdateJEstimates(KeyFrame* pKF, float J_hat)
{
    mfJCurrentEstimate = J_hat;
    mJEstimates.insert(std::make_pair(pKF, J_hat));
}

void MapPoint::UpdateJEstimates(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate[intensity_mode] = J_hat;
    mAllModesJEstimates[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates2(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate2[intensity_mode] = J_hat;
    mAllModesJEstimates2[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates3(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate3[intensity_mode] = J_hat;
    mAllModesJEstimates3[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates4(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate4[intensity_mode] = J_hat;
    mAllModesJEstimates4[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates5(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate5[intensity_mode] = J_hat;
    mAllModesJEstimates5[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates6(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate6[intensity_mode] = J_hat;
    mAllModesJEstimates6[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimates7(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimate7[intensity_mode] = J_hat;
    mAllModesJEstimates7[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimatesAllChannels(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode, int nChannel)
{
    mAllModesJCurrentEstimateAllChannels[intensity_mode][nChannel] = J_hat;
    mAllModesJEstimatesAllChannels[intensity_mode][pKF][nChannel] = J_hat;
}
void MapPoint::UpdateJEstimatesAllChannelsWoGc(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode, int nChannel)
{
    mAllModesJCurrentEstimateAllChannelsWoGc[intensity_mode][nChannel] = J_hat;
    mAllModesJEstimatesAllChannelsWoGc[intensity_mode][pKF][nChannel] = J_hat;
}
void MapPoint::UpdateJEstimatesBeta(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimateBeta[intensity_mode] = J_hat;
    mAllModesJEstimatesBeta[intensity_mode].insert(std::make_pair(pKF, J_hat));
}
void MapPoint::UpdateJEstimatesA(KeyFrame* pKF, float J_hat, FogEstimation::eIntensityMode intensity_mode)
{
    mAllModesJCurrentEstimateA[intensity_mode] = J_hat;
    mAllModesJEstimatesA[intensity_mode].insert(std::make_pair(pKF, J_hat));
}

} //namespace ORB_SLAM