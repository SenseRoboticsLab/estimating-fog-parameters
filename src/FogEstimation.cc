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
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "PatchAligner.h"
#include "SingleImageDehazerHe.h"

#include<mutex>
#include<thread>

#include <unistd.h>

namespace ORB_SLAM2
{

FoggyObservation::FoggyObservation(MapPoint* pMP, KeyFrame* pKF, float dist, float intensity, int octave, float gradient_squared):
m_pMP(pMP), m_pKF(pKF), m_dist(dist), m_octave(octave), m_gradient_squared(gradient_squared)
{
    m_intensity = intensity;
    m_intensities = cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f);
}

FoggyObservation::FoggyObservation(MapPoint* pMP, KeyFrame* pKF, float dist, const cv::Vec4f& intensities, int octave, float gradient_squared):
m_pMP(pMP), m_pKF(pKF), m_dist(dist), m_octave(octave), m_gradient_squared(gradient_squared)
{
    m_intensity = -1.0f;
    m_intensities = intensities;
}

bool operator< (const struct FoggyObservation & a, const struct FoggyObservation & b)
{
    return a.m_dist < b.m_dist;
}

bool operator> (const struct FoggyObservation & a, const struct FoggyObservation & b)
{
    return a.m_dist > b.m_dist;
}

BetaAtmos::BetaAtmos(float beta, float atmos): m_beta(beta), m_atmos(atmos)
{
}

const int FogEstimation::MIN_NUM_OBS = 4;                   // set to 0 to ignore
//const float FogEstimation::MIN_INTENSITY_RANGE = 0.0f;      // set to 0 to ignore
const float FogEstimation::MIN_DIST_BETWEEN_KFS = 5.0f;                    // set to 0 to ignore
const int FogEstimation::MIN_NUM_MPS = 15;                   // set to 1 to ignore
const float FogEstimation::TH_HUBER2 = 25.0f;

const float FogEstimation::MU = 25.0f;

const float FogEstimation::TH_SLOPE = 2.0f;

const float FogEstimation::BETA_LOW = 0.001f;  // vis ~= 3000 m
const float FogEstimation::BETA_HIGH = 0.2f;   // vis ~= 15 m
const float FogEstimation::ATMOS_LOW = 0.0f;
const float FogEstimation::ATMOS_HIGH = 255.0f;
const float FogEstimation::J_LOW = 0.0f;
const float FogEstimation::J_HIGH = 255.0f;

const float FogEstimation::HIST_BIN_WIDTH = 0.001f;
const float FogEstimation::HIST_RANGE_LOW = -0.100f - FogEstimation::HIST_BIN_WIDTH/2;
const float FogEstimation::HIST_RANGE_HIGH = FogEstimation::BETA_HIGH + FogEstimation::HIST_BIN_WIDTH/2;
const float FogEstimation::HIST_RANGE_LOW_POSITIVE = FogEstimation::BETA_LOW - FogEstimation::HIST_BIN_WIDTH/2;
const std::map<std::string, std::array<std::array<float ,3>, 4>> FogEstimation::PHOTOMETRIC_PARAMS_ALL =
        {
            {"G0E1",
            {0.3093f, 1.321f, -23.872f,
                0.2006f, 1.467f, 7.043f,
                0.07877f, 1.701f, -25.999f,
                0.1198f, 1.58f, -2.477f}
            },
            {"G0E2",
            {0.2637f, 1.218f, -15.370f,
                0.09402f, 1.494f, 4.053f,
                0.9125f, 1.044f, -36.017f,
                0.1344f, 1.407f, -4.047f}
            },
            {"G0E3",
            {0.02224f, 1.618f, -1.360f,
                0.1142f, 1.364f, 0.7401f,
                0.1794f, 1.278f, -14.429f,
                0.101f, 1.38f, -3.239f}
            },
            {"G1E1",
            {0.3553f, 1.284f, -24.601f,
                0.1742f, 1.493f, 7.142f,
                0.1026f, 1.633f, -27.295f,
                0.1131f, 1.584f, -2.267f}
            },
            {"G5E5",
            {0.0005032f, 2.199f, 5.288f,
                0.008037f, 1.751f, 4.909f,
                0.002001f, 2.005f, 3.299f,
                0.003984f, 1.874f, 4.408f}
            },
            {"G5E10",
            {0.0008394f, 1.975f, 1.318f,
                0.01082f, 1.557f, 1.134f,
                0.00312f, 1.788f, 0.04581f,
                0.005737f, 1.669f, 0.7971f}
            },
            {"G10E5",
            {0.0005052f, 2.166f, 4.471f,
                0.007655f, 1.727f, 4.213f,
                0.001706f, 2.001f, 3.075f,
                0.003715f, 1.854f, 3.831f}
            },
            {"G20E15",
            {0.0001306f, 2.160f, 1.021f,
                0.001448f, 1.777f, 1.187f,
                0.0003849f, 2.018f, 0.7238f,
                0.0007607f, 1.889f, 1.013f}
            },
            {"NA",
             {1.0f, 1.0f, 0.0f,
                 1.0f, 1.0f, 0.0f,
                 1.0f, 1.0f, 0.0f,
                 1.0f, 1.0f, 0.0f}
            }
        };

const bool FogEstimation::APPLY_GAMMA_CORRECTION = true;
float FogEstimation::GammaExpansion(float intensity)
{
    return mPhotometricParams[3][0]*pow(intensity, mPhotometricParams[3][1]) + mPhotometricParams[3][2];
}
float FogEstimation::GammaExpansion(float intensity, int c)
{
    return mPhotometricParams[c][0]*pow(intensity, mPhotometricParams[c][1]) + mPhotometricParams[c][2];
}
cv::Vec3f FogEstimation::GammaExpansion(cv::Vec3f intensities)
{
    cv::Vec3f radiances;
    for (int c = 0; c < 3; c++)
        radiances[c] = GammaExpansion(intensities[c], c);
    return radiances;
}
cv::Vec4f FogEstimation::GammaExpansion(cv::Vec4f intensities)
{
    cv::Vec4f radiances;
    for (int c = 0; c < 4; c++)
        radiances[c] = GammaExpansion(intensities[c], c);
    return radiances;
}
float FogEstimation::GammaCompression(float radiance)
{
    return pow((radiance - mPhotometricParams[3][2]) / mPhotometricParams[3][0], 1/mPhotometricParams[3][1]);
}
float FogEstimation::GammaCompression(float radiance, int c)
{
    return pow((radiance - mPhotometricParams[c][2]) / mPhotometricParams[c][0], 1/mPhotometricParams[c][1]);
}
cv::Vec3f FogEstimation::GammaCompression(cv::Vec3f radiances)
{
    cv::Vec3f intensities;
    for (int c = 0; c < 3; c++)
        intensities[c] = GammaCompression(radiances[c], c);
    return intensities;
}
cv::Vec4f FogEstimation::GammaCompression(cv::Vec4f radiances)
{
    cv::Vec4f intensities;
    for (int c = 0; c < 4; c++)
        intensities[c] = GammaCompression(radiances[c], c);
    return intensities;
}

FogEstimation::FogEstimation(Map *pMap, const string &strPhotometricSettings): mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap), mLastFogKFid(0), mbBusy(false),
                                         mCurrentEstimate(BetaAtmos(0.02f, 255.0f)), mState(NOT_READY), mbHasFogBeenEstimated(false), mPhotometricParams(FogEstimation::PHOTOMETRIC_PARAMS_ALL.at(strPhotometricSettings))
{
    for (int intensity_mode_int=RAW; intensity_mode_int!=REFD_REF_PRG; intensity_mode_int++)
    {
        eIntensityMode intensity_mode = static_cast<eIntensityMode>(intensity_mode_int);
        mAllModesCurrentEstimate.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate2.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState2.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate3.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState3.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate4.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState4.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate5.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState5.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate6.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState6.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentEstimate7.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesState7.insert(make_pair(intensity_mode, NOT_READY));
        array<BetaAtmos, 4> BetaAtmosAllChannels = {BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f)};
        mAllModesCurrentEstimateAllChannels.insert(make_pair(intensity_mode, BetaAtmosAllChannels));
        array<eFogEstimationState, 4> StateAllChannels = {NOT_READY, NOT_READY, NOT_READY, NOT_READY};
        mAllModesStateAllChannels.insert(make_pair(intensity_mode, StateAllChannels));
        array<BetaAtmos, 4> BetaAtmosAllChannelsWoGc = {BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f), BetaAtmos(0.02f, 200.0f)};
        mAllModesCurrentEstimateAllChannelsWoGc.insert(make_pair(intensity_mode, BetaAtmosAllChannelsWoGc));
        array<eFogEstimationState, 4> StateAllChannelsWoGc = {NOT_READY, NOT_READY, NOT_READY, NOT_READY};
        mAllModesStateAllChannelsWoGc.insert(make_pair(intensity_mode, StateAllChannelsWoGc));

#if TEST_PARTIAL_GT
        mAllModesCurrentBetaEstimate.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesStateBeta.insert(make_pair(intensity_mode, NOT_READY));
        mAllModesCurrentAEstimate.insert(make_pair(intensity_mode, BetaAtmos(0.02f, 200.0f)));
        mAllModesStateA.insert(make_pair(intensity_mode, NOT_READY));
#endif

//        map<pair<MapPoint*, KeyFrame*>, bool> empty_quality;
//        MapPoint* null_pMP = static_cast<MapPoint*>(nullptr);
//        KeyFrame* null_pKF = static_cast<KeyFrame*>(nullptr);
//        empty_quality.insert(make_pair(make_pair(null_pMP, null_pKF), false));
//        mAllModesQuality.insert(make_pair(intensity_mode, empty_quality));
//
//        map<pair<MapPoint*, KeyFrame*>, int> empty_num_inlier_times;
//        empty_num_inlier_times.insert(make_pair(make_pair(null_pMP, null_pKF), 0));
//        mAllModesNumInlierTimes.insert(make_pair(intensity_mode, empty_num_inlier_times));
    }
}

void FogEstimation::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void FogEstimation::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void FogEstimation::Run()
{
    mbFinished =false;

    while(1)
    {
        // set the busy flag
        SetBusy(true);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // whether the fog parameters needs to be updated
            if(NeedUpdateFog())
            {
                mbIsEstimatePerColourChannel = mpCurrentKF->mbIsInputImageColour;

//                // update fog parameters
//                UpdateFog(FogEstimation::REFD_MED_NONPRG);

                // collect LKFs and LMPs
                list<MapPoint *> lLocalMapPoints;
                CollectLMPs(lLocalMapPoints);

                // Generate distance-intensity pairs
                std::vector<std::vector<FoggyObservation>> vvFoggyObservation;
                std::vector<std::vector<FoggyObservation>> vvFoggyObservationLi;
                std::vector<std::vector<FoggyObservation>> vvFoggyObservationWoGc;
                GenerateFoggyObservationsRawProj(lLocalMapPoints, vvFoggyObservation, vvFoggyObservationLi, vvFoggyObservationWoGc, RAW, true, true);

                /* Li's method */
                // Li's interested
                if (mpCurrentKF->mbIsInputImageColour)
                {
                    std::thread threadLiMax(&FogEstimation::EstimateFogLiAllChannels, this, SingleImageDehazerHe::MAX_INTENSITY, vvFoggyObservationLi, false, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f));      // Li's original
                    std::thread threadLiMedian(&FogEstimation::EstimateFogLiAllChannels, this, SingleImageDehazerHe::MEDIAN_INTENSITY, vvFoggyObservationLi, true, cv::Vec4f(-1.0f, -1.0f, -1.0f, -1.0f));      // Li's modified
                    threadLiMax.join();
                    threadLiMedian.join();
                }
                else
                {
                    std::thread threadLiMax(&FogEstimation::EstimateFogLi, this, SingleImageDehazerHe::MAX_INTENSITY, vvFoggyObservationLi, false, -1.0f);      // Li's original
                    std::thread threadLiMedian(&FogEstimation::EstimateFogLi, this, SingleImageDehazerHe::MEDIAN_INTENSITY, vvFoggyObservationLi, true, -1.0f);      // Li's modified
                    threadLiMax.join();
                    threadLiMedian.join();
                }

#if TEST_PARTIAL_GT
                if (!mpCurrentKF->mbIsInputImageColour) {
                    std::thread threadLiMaxGT(&FogEstimation::EstimateFogLi, this, SingleImageDehazerHe::MAX_INTENSITY, vvFoggyObservationLi, false, GROUNDTRUTH_A);      // Li's original
                    std::thread threadLiMedianGT(&FogEstimation::EstimateFogLi, this, SingleImageDehazerHe::MEDIAN_INTENSITY, vvFoggyObservationLi, true, GROUNDTRUTH_A);      // Li's modified
                    threadLiMaxGT.join();
                    threadLiMedianGT.join();
                }
#endif

                /* ours method */
#if RUN_SINGLE_OURS
//                // Generate distance-intensity pairs
//                std::vector<std::vector<FoggyObservation>> vvFoggyObservation;
//                switch (OURS_SINGLE_MODE)
//                {
//                    case RAW:
//                        GenerateFoggyObservationsRawProj(lLocalMapPoints, vvFoggyObservation, RAW);
//                        break;
//                    case PROJ:
//                        GenerateFoggyObservationsRawProj(lLocalMapPoints, vvFoggyObservation, PROJ);
//                        break;
//                    case REFD_REF_NONPRG:
//                        GenerateFoggyObservationsRefdRefNonPrg(lLocalMapPoints, vvFoggyObservation);
//                        break;
//                    case REFD_MED_NONPRG:
//                        GenerateFoggyObservationsRefdMedNonPrg(lLocalMapPoints, vvFoggyObservation);
//                        break;
//                    default:
//                        break;
//                }

#if SAVE_FOGGY_OBSERVATIONS
                std::ofstream ofs;
                ofs.open("vvFoggyObservation.txt", ios::trunc);
                for (auto &vFoggyObservation : vvFoggyObservation)
                {
                    for (auto &foggyObservation : vFoggyObservation)
                    {
                        ofs << " " << std::setprecision(16) << foggyObservation.m_dist;
                        if (mpCurrentKF->mbIsInputImageColour)
                        {
                            ofs << " " << std::setprecision(16) << foggyObservation.m_intensities[0]
                                << " " << std::setprecision(16) << foggyObservation.m_intensities[1]
                                << " " << std::setprecision(16) << foggyObservation.m_intensities[2];
                        }
                        else
                            ofs << " " << std::setprecision(16) << foggyObservation.m_intensity;
                        ofs << std::endl;
                    }
                    ofs << std::endl;
                }
                ofs.close();
#endif

                // non-linear optimisation then update values
                if (vvFoggyObservation.size() >= MIN_NUM_MPS)
                {
                    if (vvFoggyObservation[0][0].m_intensity != -1) {
                        /* estimate gray channel only, e.g., all synthetic datasets */
                        std::thread threadOptimiseFogCeresTight(&FogEstimation::OptimiseFogCeresTight, this, vvFoggyObservation, OURS_SINGLE_MODE);

                        threadOptimiseFogCeresTight.join();
                    }
                    else {
                        /* estimate all channels (RGB+gray), and the case without gamma correction e.g., SDIRF */
                        std::thread threadOptimiseFogCeresTight(&FogEstimation::OptimiseFogCeresTightAllChannels, this, vvFoggyObservation, OURS_SINGLE_MODE);
                        std::thread threadOptimiseFogCeresTightWoGc(&FogEstimation::OptimiseFogCeresTightAllChannelsWoGc, this, vvFoggyObservationWoGc, OURS_SINGLE_MODE);

                        threadOptimiseFogCeresTight.join();
                        threadOptimiseFogCeresTightWoGc.join();
                    }

#if TEST_ABLATION
                    std::thread threadOptimiseFogCeresLoose(&FogEstimation::OptimiseFogCeresLoose, this, vvFoggyObservation, OURS_SINGLE_MODE);
//                    std::thread threadOptimiseFogG2o(&FogEstimation::OptimiseFogG2o, this, vvFoggyObservation, OURS_SINGLE_MODE);
//                    std::thread threadOptimiseFogCeresUnbounded(&FogEstimation::OptimiseFogCeresUnbounded, this, vvFoggyObservation, OURS_SINGLE_MODE);
//                    std::thread threadOptimiseFogCeresTightProductWeight(&FogEstimation::OptimiseFogCeresTightProductWeight, this, vvFoggyObservation, OURS_SINGLE_MODE);
                    std::thread threadOptimiseFogCeresTightUniformWeight(&FogEstimation::OptimiseFogCeresTightUniformWeight, this, vvFoggyObservation, OURS_SINGLE_MODE);
                    std::thread threadOptimiseFogCeresTightOneStage(&FogEstimation::OptimiseFogCeresTightOneStage, this, vvFoggyObservation, OURS_SINGLE_MODE);
#endif

#if TEST_PARTIAL_GT
                    std::thread threadOptimiseBeta(&FogEstimation::OptimiseBeta, this, vvFoggyObservation, OURS_SINGLE_MODE, false);
                    std::thread threadOptimiseA(&FogEstimation::OptimiseA, this, vvFoggyObservation, OURS_SINGLE_MODE, false);
#endif

#if TEST_ABLATION
                    threadOptimiseFogCeresLoose.join();
//                    threadOptimiseFogG2o.join();
//                    threadOptimiseFogCeresUnbounded.join();
//                    threadOptimiseFogCeresTightProductWeight.join();
                    threadOptimiseFogCeresTightUniformWeight.join();
                    threadOptimiseFogCeresTightOneStage.join();
#endif

#if TEST_PARTIAL_GT
                    threadOptimiseBeta.join();
                    threadOptimiseA.join();
#endif
                }

#else
                // Generate distance-intensity pairs
                std::map<eIntensityMode, std::vector<std::vector<FoggyObservation>>> FoggyObservationsAllModes;
                GenerateFoggyObservationsAllModes(lLocalMapPoints, FoggyObservationsAllModes);

#if SAVE_FOGGY_OBSERVATIONS
                std::ofstream ofs;
                ofs.open("vvFoggyObservationRaw.txt", ios::trunc);
                for (auto &vFoggyObservation : FoggyObservationsAllModes[RAW])
                {
                    for (auto &foggyObservation : vFoggyObservation)
                    {
                        ofs << " " << foggyObservation.m_dist
                            << " " << foggyObservation.m_intensity
                            << std::endl;
                    }
                    ofs << std::endl;
                }
                ofs.close();
#endif

                // non-linear optimisation then update values
                for (std::map<eIntensityMode, std::vector<std::vector<FoggyObservation>>>::iterator vit=FoggyObservationsAllModes.begin(); vit!=FoggyObservationsAllModes.end(); vit++)
                {
                    std::vector<std::vector<FoggyObservation>> vvFoggyObservation = vit->second;
                    if (vvFoggyObservation.size() >= MIN_NUM_MPS)
                    {
                        std::thread threadOptimiseFogCeresTight(&FogEstimation::OptimiseFogCeresTight, this, vvFoggyObservation, vit->first);

#if TEST_ABLATION
                        std::thread threadOptimiseFogCeresLoose(&FogEstimation::OptimiseFogCeresLoose, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseFogG2o(&FogEstimation::OptimiseFogG2o, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseFogCeresUnbounded(&FogEstimation::OptimiseFogCeresUnbounded, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseFogCeresTightProductWeight(&FogEstimation::OptimiseFogCeresTightProductWeight, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseFogCeresTightUniformWeight(&FogEstimation::OptimiseFogCeresTightUniformWeight, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseFogCeresTightOneStage(&FogEstimation::OptimiseFogCeresTightOneStage, this, vvFoggyObservation, vit->first);
#endif

#if TEST_PARTIAL_GT
                        std::thread threadOptimiseBeta(&FogEstimation::OptimiseBeta, this, vvFoggyObservation, vit->first);
                        std::thread threadOptimiseA(&FogEstimation::OptimiseA, this, vvFoggyObservation, vit->first);
#endif

                        threadOptimiseFogCeresTight.join();
#if TEST_ABLATION
                        threadOptimiseFogCeresLoose.join();
                        threadOptimiseFogG2o.join();
                        threadOptimiseFogCeresUnbounded.join();
                        threadOptimiseFogCeresTightProductWeight.join();
                        threadOptimiseFogCeresTightUniformWeight.join();
                        threadOptimiseFogCeresTightOneStage.join();
#endif

#if TEST_PARTIAL_GT
                        threadOptimiseBeta.join();
                        threadOptimiseA.join();
#endif
                    }
                }

#endif
            }
        }

        ResetIfRequested();

        // reset the busy flag
        SetBusy(false);

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void FogEstimation::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFogQueue);
    if(pKF->mnId!=0)
        mlpFogKeyFrameQueue.push_back(pKF);
}

bool FogEstimation::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexFogQueue);
    return(!mlpFogKeyFrameQueue.empty());
}

bool FogEstimation::NeedUpdateFog()
{
    {
        unique_lock<mutex> lock(mMutexFogQueue);
        mpCurrentKF = mlpFogKeyFrameQueue.front();
        mlpFogKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    float dist_between_KFs;
    if (mbHasFogBeenEstimated)
    {
        dist_between_KFs = cv::norm(mpCurrentKF->GetCameraCenter() - mPreviousFogEstimateKFPos);
    }
    else
    {
        dist_between_KFs = cv::norm(mpCurrentKF->GetCameraCenter());
    }

    // If at least MIN_NUM_OBS KFs have been created and at least 1 KFs have passed from the last fog estimation
//    if(mpCurrentKF->mnId >= MIN_NUM_OBS && mpCurrentKF->mnId >= mLastFogKFid+1)
//    if(mpCurrentKF->mnId >= MIN_NUM_OBS)
    if(mpCurrentKF->mnId >= MIN_NUM_OBS && dist_between_KFs >= MIN_DIST_BETWEEN_KFS)
    {
        for (int intensity_mode_int=RAW; intensity_mode_int!=REFD_REF_PRG; intensity_mode_int++)
        {
            eIntensityMode intensity_mode = static_cast<eIntensityMode>(intensity_mode_int);

            if (mAllModesState[intensity_mode] == NOT_READY)
                mAllModesState[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState2[intensity_mode] == NOT_READY)
                mAllModesState2[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState3[intensity_mode] == NOT_READY)
                mAllModesState3[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState4[intensity_mode] == NOT_READY)
                mAllModesState4[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState5[intensity_mode] == NOT_READY)
                mAllModesState5[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState6[intensity_mode] == NOT_READY)
                mAllModesState6[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesState7[intensity_mode] == NOT_READY)
                mAllModesState7[intensity_mode] = NOT_INITIALIZED;
#if TEST_PARTIAL_GT
            if (mAllModesStateBeta[intensity_mode] == NOT_READY)
                mAllModesStateBeta[intensity_mode] = NOT_INITIALIZED;
            if (mAllModesStateA[intensity_mode] == NOT_READY)
                mAllModesStateA[intensity_mode] = NOT_INITIALIZED;
#endif
        }

        mbHasFogBeenEstimated = true;
        mPreviousFogEstimateKFPos = mpCurrentKF->GetCameraCenter();

        return true;
    }
    else
    {
        //        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }
}

bool FogEstimation::GetBusy()
{
    unique_lock<mutex> lock(mMutexBusy);
    return mbBusy;
}

void FogEstimation::SetBusy(bool flag)
{
    unique_lock<mutex> lock(mMutexBusy);
    mbBusy=flag;
}

void FogEstimation::UpdateFog()
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(mpCurrentKF);
    mpCurrentKF->mnFogForKF = mpCurrentKF->mnId;

    const vector<KeyFrame *> vNeighKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnFogForKF = mpCurrentKF->mnId;
        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnFogForKF != mpCurrentKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnFogForKF = mpCurrentKF->mnId;
                    }
        }
    }

//    MyLog(lLocalMapPoints, "After", true, false, true, false);
//    MyLog(lLocalMapPoints, "After", true, false, true, true);

    /* get distance-intensity pairs; might need to add locks*/
    // container to store foggy observations
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation;
    vvFoggyObservation.reserve(lLocalMapPoints.size());
    // vector to store MPs that are used for estimation
    std::vector<MapPoint*> vpMPs;

    // loop through all local MPs
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint* pMP = *lit;

        // do not process MPs that have been erased or set bad
        if (!pMP || pMP->isBad())
        {
            continue;
        }

        map<KeyFrame *, size_t> observations = pMP->GetObservations();     // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)

        // do not process MPs that have too few observations (because they are not suitable for estimating fog parameters)
        if (observations.size() < MIN_NUM_OBS)
        {
            continue;
        }

        std::vector<FoggyObservation> vFoggyObservation;

        // the reference KF
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // find its ref KF
        PatchAligner pa = PatchAligner(pMP, pRefKF);     // instantiate a patch aligner object
        float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
        float ref_intensity = pa.GetRefRawIntensity();
        int ref_octave = pa.GetRefOctave();
        vFoggyObservation.push_back(FoggyObservation(pMP, pRefKF, ref_distance, ref_intensity, ref_octave));

        // record the min and max intensity
        float min_intensity = ref_intensity, max_intensity = ref_intensity;

        /* Non-Progressive alignment */
        // loop through all KFs (other than the reference one) that can see this MP
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pQurKF = mit->first;
            if (pQurKF->mnId == pRefKF->mnId)
            {
                continue;       // do not need to warp the ref KF to itself
            }

//            // use raw or projected intensity
//            pa.AddQueryKF(pQurKF);
//            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
//            float qur_intensity = pa.GetQurProjIntensity();
//            int qur_octave = pa.GetQurOctave();
//            vFoggyObservation.push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave));
//
//            if (qur_intensity < min_intensity)
//                min_intensity = qur_intensity;
//            if (qur_intensity > max_intensity)
//                max_intensity = qur_intensity;

            // use refined intensity
            pa.Align(pQurKF);
            if (pa.GetIsConverged())
            {
                float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
                float qur_intensity = pa.GetQurRefdIntensity();
                int qur_octave = pa.GetQurOctave();
                vFoggyObservation.push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave));

                if (qur_intensity < min_intensity)
                    min_intensity = qur_intensity;
                if (qur_intensity > max_intensity)
                    max_intensity = qur_intensity;
            }
        }

        /* Progressive alignment */
//        // sort observations by distance in ascending order
//        vector<pair<float, KeyFrame *>> vOrderedDistKFPairs;
//        vOrderedDistKFPairs.reserve(observations.size());
//        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
//        {
//            KeyFrame *pKF = mit->first;
//            float dist = FogEstimation::CalcDistMP2KF(pMP, pKF);
//            vOrderedDistKFPairs.push_back(make_pair(dist, pKF));
//        }
//        sort(vOrderedDistKFPairs.begin(), vOrderedDistKFPairs.end());      // in ascending order
//
//        // locate where the RefKF is
//        vector<pair<float, KeyFrame *>>::iterator itRefKF;
//        for (vector<pair<float, KeyFrame *>>::iterator mit = vOrderedDistKFPairs.begin(), mend = vOrderedDistKFPairs.end(); mit != mend; mit++)
//        {
//            if (mit->second->mnId == pRefKF->mnId) {
//                itRefKF = mit;
//                break;
//            }
//        }
//
//        // align patches in frames further than the reference KF
//        pa = PatchAligner(pMP, pRefKF);
//        KeyFrame *pBestRefKF = pRefKF;
//        for (vector<pair<float, KeyFrame *>>::iterator mit = itRefKF + 1; mit != vOrderedDistKFPairs.end(); mit++)
//        {
//            KeyFrame *pCurrentQurKF = mit->second;
//            pa.Align(pCurrentQurKF);
//
//            if (pa.GetIsConverged()) {
//                float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
//                float qur_intensity = pa.GetQurRefdIntensity();
//                int qur_octave = pa.GetQurOctave();
//                vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));
//
//                // the current query KF can be used as a reference KF to align other unaligned KFs
//                pBestRefKF = pCurrentQurKF;
//                vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
//                pa = PatchAligner(pMP, pBestRefKF);
//                pa.SetRefImg(vAlignedQurImagePyramid);
//            }
//        }
//
//        // align patches in frames closer than the reference KF
//        pa = PatchAligner(pMP, pRefKF);
//        pBestRefKF = pRefKF;
//        for (vector<pair<float, KeyFrame *>>::iterator mit = itRefKF; mit != vOrderedDistKFPairs.begin();)
//        {
//            mit--;
//
//            KeyFrame *pCurrentQurKF = mit->second;
//            pa.Align(pCurrentQurKF);
//
//            if (pa.GetIsConverged()) {
//                float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
//                float qur_intensity = pa.GetQurRefdIntensity();
//                int qur_octave = pa.GetQurOctave();
//                vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));
//
//                // the current query KF can be used as a reference KF to align other unaligned KFs
//                pBestRefKF = pCurrentQurKF;
//                vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
//                pa = PatchAligner(pMP, pBestRefKF);
//                pa.SetRefImg(vAlignedQurImagePyramid);
//            }
//        }


        if (vFoggyObservation.size() < MIN_NUM_OBS)
        {
            continue;
        }
        else
        {
            vvFoggyObservation.push_back(vFoggyObservation);
            vpMPs.push_back(pMP);
        }
    }

    if (vvFoggyObservation.size() >= MIN_NUM_MPS)
    {
        /* ready to estimate fog parameters */
        // set g2o and optimizer
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 1>> Block;  // variable to optimise is of dimension max{1, 2}, error is of dimension 1
        Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block *solver_ptr = new Block(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(true);

        const size_t n_MPs = vpMPs.size();       // number of MPs used for estimation

        // add clear intensity vertices, each of which corresponds to a map point
        for (int i = 0; i < n_MPs; i++) {
            ClearVertex* vtxClear = new ClearVertex();
            float J_init = vpMPs[i]->GetJCurrentEstimate();
            if (J_init == 0.0f)
                vtxClear->setEstimate(128.0f);  // this MP's J has never been estimated, use 128 to initialise
            else
                vtxClear->setEstimate(
                        J_init);  // this MP's J has been estimated before, use the current estimate to initialise
            vtxClear->setId(i);
            vtxClear->setMarginalized(true);          // need to set to true!
            optimizer.addVertex(vtxClear);
        }

        // add a (the only one) fog parameter vertex containing the scattering coefficient Beta and the atmospheric light A
        BetaAtmosVertex* vtxBetaAtmos = new BetaAtmosVertex();
        vtxBetaAtmos->setEstimate(Eigen::Vector2f(mCurrentEstimate.m_beta,
                                                  mCurrentEstimate.m_atmos));    // use the current estimates to initialise
        vtxBetaAtmos->setId(n_MPs);
        optimizer.addVertex(vtxBetaAtmos);

        // containers to store edges, MPs and KFs
        const int nExpectedSize = n_MPs * lLocalMapPoints.size();
        std::vector<FoggyEdge *> vpEdges;
        vpEdges.reserve(nExpectedSize);
        std::vector<MapPoint *> vpEdgeMPs;
        vpEdgeMPs.reserve(nExpectedSize);
        std::vector<KeyFrame *> vpEdgeKFs;
        vpEdgeKFs.reserve(nExpectedSize);

        // add edges (each edge corresponds to an observation of a map point)
        int JVertex_count = 0;
        for (std::vector<std::vector<FoggyObservation>>::iterator vvit = vvFoggyObservation.begin();
             vvit != vvFoggyObservation.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation = *vvit;
            int observation_count = 0;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                 vit != vFoggyObservation.end(); vit++) {
                FoggyEdge *edgeFoggy = new FoggyEdge(vit->m_dist);
                edgeFoggy->setId(100 * JVertex_count + observation_count);  // to assure unique edgeFoggy ID
                edgeFoggy->setVertex(0, dynamic_cast<ClearVertex *>(optimizer.vertex(
                        JVertex_count))); // 1st vertex is ClearVertex
                edgeFoggy->setVertex(1, vtxBetaAtmos);         // 2nd vertex is the BetaAtmosVertex
                edgeFoggy->setMeasurement(vit->m_intensity);
                edgeFoggy->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                          mpCurrentKF->mvLevelSigma2[vit->m_octave]);      // TODO

                // Huber loss
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                edgeFoggy->setRobustKernel(rk);
                rk->setDelta(sqrt(TH_HUBER2));     // TODO

                optimizer.addEdge(edgeFoggy);
                observation_count++;

                vpEdges.push_back(edgeFoggy);
                vpEdgeMPs.push_back(vit->m_pMP);
                vpEdgeKFs.push_back(vit->m_pKF);
            }
            JVertex_count++;
        }

        // optimise 1st round
        optimizer.initializeOptimization();
        int n_Iters1 = optimizer.optimize(20);

        // print for debug
        std::cout << mpCurrentKF->mnFrameId
                  << " " << mpCurrentKF->mnId
                  << " " << vtxBetaAtmos->estimate()[0]
                  << " " << vtxBetaAtmos->estimate()[1]
                  << " " << optimizer.activeVertices().size() - 1
                  << " " << optimizer.activeEdges().size()
                  << " " << n_Iters1
                  << std::endl;

//    // find outliers and do not optimise them in the second round
//    for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
//    {
//        FoggyEdge* e = vpEdges[i];
//        MapPoint* pMP = vpEdgeMPs[i];
//        ClearVertex* vJ = dynamic_cast<ClearVertex*>(e->vertex(0));
//
//        if(!pMP || pMP->isBad())
//            continue;
//
//        // if error is too big or the estimated J is not within [0, 255]
//        if(e->chi2()>TH_HUBER2 || vJ->estimate()<0 || vJ->estimate()>255)     // TODO
//        {
//            // do not optimise in the next round
//            e->setLevel(1);
//        }
//        // do not use robust loss in the second round
//        e->setRobustKernel(0);
//    }
//
//    // optimise 2nd round
//    optimizer.initializeOptimization(0);
//    int n_Iters2 = optimizer.optimize(10);
//
//    // print for debug
//    std::cout << "FID: " << mpCurrentKF->mnFrameId
//              << "\t\tKFID: " << mpCurrentKF->mnId
//              << "\t\tBeta: " << vtxBetaAtmos->estimate()[0]
//              << "\t\tAtmos: " << vtxBetaAtmos->estimate()[1]
//              << "\t\tnumMPs: " << optimizer.activeVertices().size() - 1
//              << "\t\tnumObs: " << optimizer.activeEdges().size()
//              << "\t\tnumIters: " << n_Iters2
//              << std::endl
//              << std::endl;

        // retrieve estimated values
        // Js
        for (size_t i = 0; i < vpMPs.size(); i++) {
            ClearVertex *currentJVertex = dynamic_cast<ClearVertex *>(optimizer.vertex(i));
            vpMPs[i]->UpdateJEstimates(mpCurrentKF, currentJVertex->estimate());
        }
        // Beta and A
        Eigen::Vector2f BetaAtmos_hat = vtxBetaAtmos->estimate();
        mCurrentEstimate = BetaAtmos(BetaAtmos_hat[0], BetaAtmos_hat[1]);
        mEstimates.insert(std::make_pair(mpCurrentKF, mCurrentEstimate));

        mLastFogKFid = mpCurrentKF->mnId;
    }
}

    void FogEstimation::UpdateFog(eIntensityMode intensity_mode)
    {
        /* STEP1: COLLECT LKFS AND LMPS */
        // might need to add locks
        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        lLocalKeyFrames.push_back(mpCurrentKF);
        mpCurrentKF->mnFogForKF = mpCurrentKF->mnId;

        const vector<KeyFrame *> vNeighKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnFogForKF = mpCurrentKF->mnId;
            if (!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        list<MapPoint *> lLocalMapPoints;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnFogForKF != mpCurrentKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnFogForKF = mpCurrentKF->mnId;
                        }
            }
        }

        /* STEP2: GET DISTANCE-INTENSITY PAIRS */
        // might need to add locks
        // container to store foggy observations
        std::vector<std::vector<FoggyObservation>> vvFoggyObservation;
        vvFoggyObservation.reserve(lLocalMapPoints.size());
        // vector to store MPs that are used for estimation
        std::vector<MapPoint*> vpMPs;

        // loop through all local MPs
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint* pMP = *lit;

            // do not process MPs that have been erased or set bad
            if (!pMP || pMP->isBad())
            {
                continue;
            }

            map<KeyFrame *, size_t> observations = pMP->GetObservations();     // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)

            // do not process MPs that have too few observations (because they are not suitable for estimating fog parameters)
            if (observations.size() < MIN_NUM_OBS)
            {
                continue;
            }

            std::vector<FoggyObservation> vFoggyObservation;

            // record the min and max intensity
            float min_intensity, max_intensity;

            // if use raw or projected intensity, or use ORB-SLAM2's reference KF as alignment's reference frame
            if (intensity_mode == FogEstimation::RAW
             || intensity_mode == FogEstimation::PROJ
             || intensity_mode == FogEstimation::REFD_REF_NONPRG
             || intensity_mode == FogEstimation::REFD_REF_PRG
             )
            {
                // construct the alignment's reference frame
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // find its ref KF
                PatchAligner pa = PatchAligner(pMP, pRefKF);     // instantiate a patch aligner object
                float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
                float ref_intensity = pa.GetRefRawIntensity();
                int ref_octave = pa.GetRefOctave();
                vFoggyObservation.push_back(FoggyObservation(pMP, pRefKF, ref_distance, ref_intensity, ref_octave));

                min_intensity = ref_intensity;
                max_intensity = ref_intensity;

                // if use raw or projected intensity, or non-progressive alignment
                if (intensity_mode != FogEstimation::REFD_REF_PRG)
                {
                    // loop through all KFs (other than the reference one) that can see this MP
                    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
                        KeyFrame *pQurKF = mit->first;
                        if (pQurKF->mnId == pRefKF->mnId) {
                            continue;       // do not need to warp the ref KF to itself
                        }

                        // if use raw or projected intensity
                        if (intensity_mode == FogEstimation::RAW
                         || intensity_mode == FogEstimation::PROJ)
                        {
                            pa.AddQueryKF(pQurKF);
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
                            float qur_intensity;
                            if (intensity_mode == FogEstimation::RAW)
                                qur_intensity = pa.GetQurRawIntensity();
                            else
                                qur_intensity = pa.GetQurProjIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;
                        }
                        // else use non-progressive alignment and align the rest to the reference KF
                        else
                        {
                            pa.Align(pQurKF);
                            if (pa.GetIsConverged())
                            {
                                float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
                                float qur_intensity = pa.GetQurRefdIntensity();
                                int qur_octave = pa.GetQurOctave();
                                vFoggyObservation.push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave));

                                if (qur_intensity < min_intensity)
                                    min_intensity = qur_intensity;
                                if (qur_intensity > max_intensity)
                                    max_intensity = qur_intensity;
                            }
                        }

                    }
                }
                // else use progressive alignment and align the rest to the reference KF
                else
                {
                    // sort observations by distance in ascending order
                    vector<pair<float, KeyFrame *>> vOrderedDistKFPairs;
                    vOrderedDistKFPairs.reserve(observations.size());
                    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                    {
                        KeyFrame *pKF = mit->first;
                        float dist = FogEstimation::CalcDistMP2KF(pMP, pKF);
                        vOrderedDistKFPairs.push_back(make_pair(dist, pKF));
                    }
                    sort(vOrderedDistKFPairs.begin(), vOrderedDistKFPairs.end());      // in ascending order

                    // locate where the RefKF is
                    vector<pair<float, KeyFrame *>>::iterator itRefKF;
                    for (vector<pair<float, KeyFrame *>>::iterator mit = vOrderedDistKFPairs.begin(), mend = vOrderedDistKFPairs.end(); mit != mend; mit++)
                    {
                        if (mit->second->mnId == pRefKF->mnId) {
                            itRefKF = mit;
                            break;
                        }
                    }

                    // align patches in frames further than the reference KF
                    pa = PatchAligner(pMP, pRefKF);
                    KeyFrame *pBestRefKF = pRefKF;
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itRefKF + 1; mit != vOrderedDistKFPairs.end(); mit++)
                    {
                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged()) {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;

                            // the current query KF can be used as a reference KF to align other unaligned KFs
                            pBestRefKF = pCurrentQurKF;
                            vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                            pa = PatchAligner(pMP, pBestRefKF);
                            pa.SetRefImg(vAlignedQurImagePyramid);
                        }
                    }

                    // align patches in frames closer than the reference KF
                    pa = PatchAligner(pMP, pRefKF);
                    pBestRefKF = pRefKF;
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itRefKF; mit != vOrderedDistKFPairs.begin();)
                    {
                        mit--;

                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged()) {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;

                            // the current query KF can be used as a reference KF to align other unaligned KFs
                            pBestRefKF = pCurrentQurKF;
                            vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                            pa = PatchAligner(pMP, pBestRefKF);
                            pa.SetRefImg(vAlignedQurImagePyramid);
                        }
                    }
                }
            }
            // else use the middle-distance KF as alignment's reference frame
            else
            {
                // sort observations by distance in ascending order
                vector<pair<float, KeyFrame *>> vOrderedDistKFPairs;
                vOrderedDistKFPairs.reserve(observations.size());
                for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                {
                    KeyFrame *pKF = mit->first;
                    float dist = FogEstimation::CalcDistMP2KF(pMP, pKF);
                    vOrderedDistKFPairs.push_back(make_pair(dist, pKF));
                }
                sort(vOrderedDistKFPairs.begin(), vOrderedDistKFPairs.end());      // in ascending order

                // get the iterator of the middle-distance frame
                vector<pair<float, KeyFrame *>>::iterator itMedDistKF = vOrderedDistKFPairs.begin() + vOrderedDistKFPairs.size()/2;

                // construct the alignment's reference frame
                KeyFrame *pRefKF = itMedDistKF->second;
                PatchAligner pa = PatchAligner(pMP, pRefKF);     // instantiate a patch aligner object
                float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
                float ref_intensity = pa.GetRefRawIntensity();
                int ref_octave = pa.GetRefOctave();
                vFoggyObservation.push_back(FoggyObservation(pMP, pRefKF, ref_distance, ref_intensity, ref_octave));

                min_intensity = ref_intensity;
                max_intensity = ref_intensity;

                if (intensity_mode == FogEstimation::REFD_MED_NONPRG)
                {
                    // align patches in frames further than the reference KF
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itMedDistKF + 1; mit != vOrderedDistKFPairs.end(); mit++)
                    {
                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged())
                        {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;
                        }
                    }

                    // align patches in frames closer than the reference KF
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itMedDistKF; mit != vOrderedDistKFPairs.begin();)
                    {
                        mit--;

                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged())
                        {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;
                        }
                    }
                }
                else
                {
                    // align patches in frames further than the reference KF
                    pa = PatchAligner(pMP, pRefKF);
                    KeyFrame *pBestRefKF = pRefKF;
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itMedDistKF + 1; mit != vOrderedDistKFPairs.end(); mit++)
                    {
                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged()) {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;

                            // the current query KF can be used as a reference KF to align other unaligned KFs
                            pBestRefKF = pCurrentQurKF;
                            vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                            pa = PatchAligner(pMP, pBestRefKF);
                            pa.SetRefImg(vAlignedQurImagePyramid);
                        }
                    }

                    // align patches in frames closer than the reference KF
                    pa = PatchAligner(pMP, pRefKF);
                    pBestRefKF = pRefKF;
                    for (vector<pair<float, KeyFrame *>>::iterator mit = itMedDistKF; mit != vOrderedDistKFPairs.begin();)
                    {
                        mit--;

                        KeyFrame *pCurrentQurKF = mit->second;
                        pa.Align(pCurrentQurKF);

                        if (pa.GetIsConverged()) {
                            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pCurrentQurKF);
                            float qur_intensity = pa.GetQurRefdIntensity();
                            int qur_octave = pa.GetQurOctave();
                            vFoggyObservation.push_back(FoggyObservation(pMP, pCurrentQurKF, qur_distance, qur_intensity, qur_octave));

                            if (qur_intensity < min_intensity)
                                min_intensity = qur_intensity;
                            if (qur_intensity > max_intensity)
                                max_intensity = qur_intensity;

                            // the current query KF can be used as a reference KF to align other unaligned KFs
                            pBestRefKF = pCurrentQurKF;
                            vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                            pa = PatchAligner(pMP, pBestRefKF);
                            pa.SetRefImg(vAlignedQurImagePyramid);
                        }
                    }
                }
            }

            if (vFoggyObservation.size() < MIN_NUM_OBS)
            {
                continue;
            }
            else
            {
                vvFoggyObservation.push_back(vFoggyObservation);
                vpMPs.push_back(pMP);
            }
        }

        /* STEP3: NON-LINEAR OPTIMISATION */
        if (vvFoggyObservation.size() >= MIN_NUM_MPS)
        {
            /* ready to estimate fog parameters */
            // set g2o and optimizer
            typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 1>> Block;  // variable to optimise is of dimension max{1, 2}, error is of dimension 1
            Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
            Block *solver_ptr = new Block(linearSolver);
            g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(true);

            const size_t n_MPs = vpMPs.size();       // number of MPs used for estimation

            // add clear intensity vertices, each of which corresponds to a map point
            for (int i = 0; i < n_MPs; i++) {
                ClearVertex* vtxClear = new ClearVertex();
                float J_init = vpMPs[i]->GetJCurrentEstimate();
                if (J_init == 0.0f)
                    vtxClear->setEstimate(128.0f);  // this MP's J has never been estimated, use 128 to initialise
                else
                    vtxClear->setEstimate(
                            J_init);  // this MP's J has been estimated before, use the current estimate to initialise
                vtxClear->setId(i);
                vtxClear->setMarginalized(true);          // need to set to true!
                optimizer.addVertex(vtxClear);
            }

            // add a (the only one) fog parameter vertex containing the scattering coefficient Beta and the atmospheric light A
            BetaAtmosVertex* vtxBetaAtmos = new BetaAtmosVertex();
            vtxBetaAtmos->setEstimate(Eigen::Vector2f(mCurrentEstimate.m_beta,
                                                      mCurrentEstimate.m_atmos));    // use the current estimates to initialise
            vtxBetaAtmos->setId(n_MPs);
            optimizer.addVertex(vtxBetaAtmos);

            // containers to store edges, MPs and KFs
            const int nExpectedSize = n_MPs * lLocalMapPoints.size();
            std::vector<FoggyEdge *> vpEdges;
            vpEdges.reserve(nExpectedSize);
            std::vector<MapPoint *> vpEdgeMPs;
            vpEdgeMPs.reserve(nExpectedSize);
            std::vector<KeyFrame *> vpEdgeKFs;
            vpEdgeKFs.reserve(nExpectedSize);

            // add edges (each edge corresponds to an observation of a map point)
            int JVertex_count = 0;
            for (std::vector<std::vector<FoggyObservation>>::iterator vvit = vvFoggyObservation.begin();
                 vvit != vvFoggyObservation.end(); vvit++) {
                std::vector<FoggyObservation> vFoggyObservation = *vvit;
                int observation_count = 0;
                for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                     vit != vFoggyObservation.end(); vit++) {
                    FoggyEdge *edgeFoggy = new FoggyEdge(vit->m_dist);
                    edgeFoggy->setId(100 * JVertex_count + observation_count);  // to assure unique edgeFoggy ID
                    edgeFoggy->setVertex(0, dynamic_cast<ClearVertex *>(optimizer.vertex(
                            JVertex_count))); // 1st vertex is ClearVertex
                    edgeFoggy->setVertex(1, vtxBetaAtmos);         // 2nd vertex is the BetaAtmosVertex
                    edgeFoggy->setMeasurement(vit->m_intensity);
                    edgeFoggy->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                              mpCurrentKF->mvLevelSigma2[vit->m_octave]);      // TODO

                    // Huber loss
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    edgeFoggy->setRobustKernel(rk);
                    rk->setDelta(sqrt(TH_HUBER2));     // TODO

                    optimizer.addEdge(edgeFoggy);
                    observation_count++;

                    vpEdges.push_back(edgeFoggy);
                    vpEdgeMPs.push_back(vit->m_pMP);
                    vpEdgeKFs.push_back(vit->m_pKF);
                }
                JVertex_count++;
            }

            // optimise 1st round
            optimizer.initializeOptimization();
            int n_Iters1 = optimizer.optimize(20);

            // print for debug
            std::cout << mpCurrentKF->mnFrameId
                      << " " << mpCurrentKF->mnId
                      << " " << vtxBetaAtmos->estimate()[0]
                      << " " << vtxBetaAtmos->estimate()[1]
                      << " " << optimizer.activeVertices().size() - 1
                      << " " << optimizer.activeEdges().size()
                      << " " << n_Iters1
                      << std::endl;

//    // find outliers and do not optimise them in the second round
//    for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
//    {
//        FoggyEdge* e = vpEdges[i];
//        MapPoint* pMP = vpEdgeMPs[i];
//        ClearVertex* vJ = dynamic_cast<ClearVertex*>(e->vertex(0));
//
//        if(!pMP || pMP->isBad())
//            continue;
//
//        // if error is too big or the estimated J is not within [0, 255]
//        if(e->chi2()>TH_HUBER2 || vJ->estimate()<0 || vJ->estimate()>255)     // TODO
//        {
//            // do not optimise in the next round
//            e->setLevel(1);
//        }
//        // do not use robust loss in the second round
//        e->setRobustKernel(0);
//    }
//
//    // optimise 2nd round
//    optimizer.initializeOptimization(0);
//    int n_Iters2 = optimizer.optimize(10);
//
//    // print for debug
//    std::cout << "FID: " << mpCurrentKF->mnFrameId
//              << "\t\tKFID: " << mpCurrentKF->mnId
//              << "\t\tBeta: " << vtxBetaAtmos->estimate()[0]
//              << "\t\tAtmos: " << vtxBetaAtmos->estimate()[1]
//              << "\t\tnumMPs: " << optimizer.activeVertices().size() - 1
//              << "\t\tnumObs: " << optimizer.activeEdges().size()
//              << "\t\tnumIters: " << n_Iters2
//              << std::endl
//              << std::endl;

            // retrieve estimated values
            // Js
            for (size_t i = 0; i < vpMPs.size(); i++) {
                ClearVertex *currentJVertex = dynamic_cast<ClearVertex *>(optimizer.vertex(i));
                vpMPs[i]->UpdateJEstimates(mpCurrentKF, currentJVertex->estimate());
            }
            // Beta and A
            Eigen::Vector2f BetaAtmos_hat = vtxBetaAtmos->estimate();
            mCurrentEstimate = BetaAtmos(BetaAtmos_hat[0], BetaAtmos_hat[1]);
            mEstimates.insert(std::make_pair(mpCurrentKF, mCurrentEstimate));

            mLastFogKFid = mpCurrentKF->mnId;
        }
    }

void FogEstimation::CollectLMPs(list<MapPoint *> &lLocalMapPoints)
{
    /* might need to add locks */

    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(mpCurrentKF);
    mpCurrentKF->mnFogForKF = mpCurrentKF->mnId;

    const vector<KeyFrame *> vNeighKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnFogForKF = mpCurrentKF->mnId;
        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnFogForKF != mpCurrentKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnFogForKF = mpCurrentKF->mnId;
                    }
        }
    }
}

void FogEstimation::GenerateFoggyObservationsRawProj(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, std::vector<std::vector<FoggyObservation>> &vvFoggyObservationLi, std::vector<std::vector<FoggyObservation>> &vvFoggyObservationWoGc, eIntensityMode intensity_mode, bool also_li, bool also_wo_gc)
{
    assert((intensity_mode == RAW
           || intensity_mode == PROJ)
           && "The intensity mode has to be either RAW or PROJ!");

    vvFoggyObservation.reserve(lLocalMapPoints.size());
    if (also_li)
        vvFoggyObservationLi.reserve(lLocalMapPoints.size());
    if (also_wo_gc)
        vvFoggyObservationWoGc.reserve(lLocalMapPoints.size());

    // loop through all local MPs
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;

        // do not process MPs that have been erased or set bad
        if (!pMP || pMP->isBad()) {
            continue;
        }

        // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)
        map<KeyFrame *, size_t> observations(pMP->GetObservations());     // use the copy constructor because the map will be modified later and we don't want to affect the MP's member variable

        // do not process MPs that have too few observations
        if (observations.size() < MIN_NUM_OBS) {
            continue;
        }

        std::vector<FoggyObservation> vFoggyObservation;
        vFoggyObservation.reserve(observations.size());
        std::vector<FoggyObservation> vFoggyObservationWoGc;
        if (also_wo_gc)
        {
            vFoggyObservationWoGc.reserve(observations.size());
        }

        // the reference KF (does not matter here! just make use of the patch aligner code)
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // find its ref KF
        PatchAligner pa = PatchAligner(pMP, pRefKF);     // instantiate a patch aligner object
        float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
        if (mpCurrentKF->mbIsInputImageColour)
        {
            cv::Vec4f ref_intensities;
            if (intensity_mode == RAW)
            {
                ref_intensities = pa.GetRefRawIntensities();
            }
            else
            {
                ref_intensities = pa.GetRefProjIntensities();
            }
            int ref_octave = pa.GetRefOctave();
            if (ref_intensities[0] >= 0 && ref_intensities[1] >= 0 && ref_intensities[2] >= 0 && ref_intensities[3] >= 0)
            {
                vFoggyObservation.emplace_back(pMP, pRefKF, ref_distance, GammaExpansion(ref_intensities), ref_octave, -1.0f);
                if (also_wo_gc)
                    vFoggyObservationWoGc.emplace_back(pMP, pRefKF, ref_distance, ref_intensities, ref_octave, -1.0f);
            }
        }
        else
        {
            float ref_intensity, ref_gradient_squared;
            if (intensity_mode == RAW)
            {
                ref_intensity = pa.GetRefRawIntensity();
                ref_gradient_squared = pa.GetRefRawGradientSquared();
            }
            else
            {
                ref_intensity = pa.GetRefProjIntensity();
                ref_gradient_squared = pa.GetRefProjGradientSquared();
            }
            int ref_octave = pa.GetRefOctave();
            if (ref_intensity >= 0)
            {
                vFoggyObservation.emplace_back(pMP, pRefKF, ref_distance, GammaExpansion(ref_intensity), ref_octave, -1.0f);
                if (also_wo_gc)
                    vFoggyObservationWoGc.emplace_back(pMP, pRefKF, ref_distance, ref_intensity, ref_octave, ref_gradient_squared);
            }
        }
        observations.erase(pRefKF);     // erase the observation from the reference KF

        // loop through all KFs (other than the reference one because it has already been erased) that can see this MP
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            KeyFrame *pQurKF = mit->first;

            pa.AddQueryKF(pQurKF);
            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
            float qur_intensity, qur_gradient_squared;
            if (mpCurrentKF->mbIsInputImageColour)
            {
                cv::Vec4f qur_intensities;
                if (intensity_mode == RAW)
                {
                    qur_intensities = pa.GetQurRawIntensities();
                }
                else
                {
                    qur_intensities = pa.GetQurProjIntensities();
                }
                int qur_octave = pa.GetQurOctave();
                if (qur_intensities[0] >= 0 && qur_intensities[1] >= 0 && qur_intensities[2] >= 0 && qur_intensities[3] >= 0)
                {
                    vFoggyObservation.emplace_back(pMP, pQurKF, qur_distance, GammaExpansion(qur_intensities), qur_octave, -1.0f);
                    if (also_wo_gc)
                        vFoggyObservationWoGc.emplace_back(pMP, pQurKF, qur_distance, qur_intensities, qur_octave, -1.0f);
                }
            }
            else
            {
                if (intensity_mode == RAW)
                {
                    qur_intensity = pa.GetQurRawIntensity();
                    qur_gradient_squared = pa.GetQurRawGradientSquared();
                }
                else
                {
                    qur_intensity = pa.GetQurProjIntensity();
                    qur_gradient_squared = pa.GetQurProjGradientSquared();
                }
                int qur_octave = pa.GetQurOctave();
                if (qur_intensity >= 0)
                {
                    vFoggyObservation.emplace_back(pMP, pQurKF, qur_distance, GammaExpansion(qur_intensity), qur_octave, -1.0f);
                    if (also_wo_gc)
                        vFoggyObservationWoGc.emplace_back(pMP, pQurKF, qur_distance, qur_intensity, qur_octave, qur_gradient_squared);
                }
            }
        }

        if (vFoggyObservation.size() >= MIN_NUM_OBS)
        {
            vvFoggyObservation.push_back(vFoggyObservation);
            if (also_li)
                vvFoggyObservationLi.push_back(vFoggyObservation);
            if (also_wo_gc)
                vvFoggyObservationWoGc.push_back(vFoggyObservationWoGc);
        }
        else
        {
            if (also_li)
                vvFoggyObservationLi.push_back(vFoggyObservation);
        }
    }
}

static std::mutex s_vFoggyObservationMutex;

static void AlignRef(std::vector<FoggyObservation> *pvFoggyObservation, MapPoint* pMP, KeyFrame* pRefKF, KeyFrame* pQurKF)
{
    PatchAligner pa = PatchAligner(pMP, pRefKF, true);
    pa.Align(pQurKF);
    std::lock_guard<std::mutex> lock(s_vFoggyObservationMutex);
//    if (pa.GetIsConverged())
    if (true)   // no matter the alignment is converged or not, we still add the observation because in the latter case, the raw intensity will be used
    {
        float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
        float qur_intensity = pa.GetQurRefdIntensity();
        int qur_octave = pa.GetQurOctave();
        float qur_gradient_squared = pa.GetQurRefdGradientSquared();
        if (qur_intensity >= 0)
            pvFoggyObservation->push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave, qur_gradient_squared));
    }
}

static void AlignMed(std::vector<FoggyObservation> *pvFoggyObservation, MapPoint* pMP, KeyFrame* pRefKF, KeyFrame* pQurKF, map<KeyFrame *, float> *pKFDist)
{
    PatchAligner pa = PatchAligner(pMP, pRefKF, true);
    pa.Align(pQurKF);
    std::lock_guard<std::mutex> lock(s_vFoggyObservationMutex);
//    if (pa.GetIsConverged())
    if (true)   // no matter the alignment is converged or not, we still add the observation because in the latter case, the raw intensity will be used
    {
        float qur_distance = (*pKFDist)[pQurKF];
        float qur_intensity = pa.GetQurRefdIntensity();
        int qur_octave = pa.GetQurOctave();
        float qur_gradient_squared = pa.GetQurRefdGradientSquared();
        if (qur_intensity >= 0)
            pvFoggyObservation->push_back(FoggyObservation(pMP, pQurKF, qur_distance, qur_intensity, qur_octave, qur_gradient_squared));
    }
}

void FogEstimation::GenerateFoggyObservationsRefdRefNonPrg(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation)
{
    vvFoggyObservation.reserve(lLocalMapPoints.size());

    // loop through all local MPs
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;

        // do not process MPs that have been erased or set bad
        if (!pMP || pMP->isBad()) {
            continue;
        }

        // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)
        map<KeyFrame *, size_t> observations(pMP->GetObservations());     // use the copy constructor because the map will be modified later and we don't want to affect the MP's member variable

        // do not process MPs that have too few observations
        if (observations.size() < MIN_NUM_OBS) {
            continue;
        }

        std::vector<FoggyObservation> vFoggyObservation;
        vFoggyObservation.reserve(observations.size());

        // choose the reference KF
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // use ORB-SLAM2's reference KF
        PatchAligner pa = PatchAligner(pMP, pRefKF, true);     // instantiate a patch aligner object
        float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
        float ref_intensity = pa.GetRefRawIntensity();
        float ref_gradient_squared = pa.GetRefRawGradientSquared();
        if (ref_intensity < 0)
            continue;
        int ref_octave = pa.GetRefOctave();
        vFoggyObservation.emplace_back(pMP, pRefKF, ref_distance, ref_intensity, ref_octave, ref_gradient_squared);
        observations.erase(pRefKF);     // erase the observation from the reference KF

        // loop through all KFs (other than the reference one because it has already been erased) that can see this MP
#define ASYNC 1
#if ASYNC
        // use multi-threading
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            mFutures.push_back(std::async(std::launch::async, AlignRef, &vFoggyObservation, pMP, pRefKF, mit->first));
        }
        mFutures.clear();
#else
        // do not use multi-threading
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            KeyFrame *pQurKF = mit->first;

            // use refined intensity
            pa.Align(pQurKF);
            if (pa.GetIsConverged())
            {
                float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
                float qur_intensity = pa.GetQurRefdIntensity();
                int qur_octave = pa.GetQurOctave();
                if (qur_intensity >= 0)
                    vFoggyObservation.emplace_back(pMP, pQurKF, qur_distance, qur_intensity, qur_octave);
            }
        }
#endif

        if (vFoggyObservation.size() >= MIN_NUM_OBS)
            vvFoggyObservation.push_back(vFoggyObservation);
    }
}

void FogEstimation::GenerateFoggyObservationsRefdMedNonPrg(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation)
{
    vvFoggyObservation.reserve(lLocalMapPoints.size());

    // loop through all local MPs
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;

        // do not process MPs that have been erased or set bad
        if (!pMP || pMP->isBad()) {
            continue;
        }

        // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)
        map<KeyFrame *, size_t> observations(pMP->GetObservations());     // use the copy constructor because the map will be modified later and we don't want to affect the MP's member variable

        // do not process MPs that have too few observations
        if (observations.size() < MIN_NUM_OBS) {
            continue;
        }

        std::vector<FoggyObservation> vFoggyObservation;
        vFoggyObservation.reserve(observations.size());

        // get the iterator of the median distance
        vector<pair<float, KeyFrame *>> vDistKFPairs;
        map<KeyFrame *, float> KFDist;
        vDistKFPairs.reserve(observations.size());
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            float dist = FogEstimation::CalcDistMP2KF(pMP, pKF);
            vDistKFPairs.emplace_back(dist, pKF);
            KFDist.insert(make_pair(pKF, dist));
        }
        vector<pair<float, KeyFrame *>>::iterator itMedDistKF = vDistKFPairs.begin() + vDistKFPairs.size()/2;
        std::nth_element(vDistKFPairs.begin(), itMedDistKF , vDistKFPairs.end());

        // choose the reference KF
        KeyFrame *pRefKF = itMedDistKF->second;                  // use the KF with median distance as the reference KF
        PatchAligner pa = PatchAligner(pMP, pRefKF, true);     // instantiate a patch aligner object
        float ref_distance = KFDist[pRefKF];
        float ref_intensity = pa.GetRefRawIntensity();
        float ref_gradient_squared = pa.GetRefRawGradientSquared();
        if (ref_intensity < 0)
            continue;
        int ref_octave = pa.GetRefOctave();
        vFoggyObservation.emplace_back(pMP, pRefKF, ref_distance, ref_intensity, ref_octave, ref_gradient_squared);
        observations.erase(pRefKF);     // erase the observation from the reference KF

        // loop through all KFs (other than the reference one because it has already been erased) that can see this MP
#define ASYNC 1
#if ASYNC
        // use multi-threading
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            mFutures.push_back(std::async(std::launch::async, AlignMed, &vFoggyObservation, pMP, pRefKF, mit->first, &KFDist));
        }
        mFutures.clear();
#else
        // do not use multi-threading
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            KeyFrame *pQurKF = mit->first;

            // use refined intensity
            pa.Align(pQurKF);
            if (pa.GetIsConverged())
            {
                float qur_distance = KFDist[pQurKF];
                float qur_intensity = pa.GetQurRefdIntensity();
                int qur_octave = pa.GetQurOctave();
                if (qur_intensity >= 0)
                    vFoggyObservation.emplace_back(pMP, pQurKF, qur_distance, qur_intensity, qur_octave);
            }
        }
#endif

        if (vFoggyObservation.size() >= MIN_NUM_OBS)
            vvFoggyObservation.push_back(vFoggyObservation);
    }
}

void FogEstimation::GenerateFoggyObservationsAllModes(const list<MapPoint *> &lLocalMapPoints,
                                                      std::map<eIntensityMode, std::vector<std::vector<FoggyObservation>>> &FoggyObservationsAllModes)
{
    FoggyObservationsAllModes[RAW].reserve(lLocalMapPoints.size());
    FoggyObservationsAllModes[PROJ].reserve(lLocalMapPoints.size());
    FoggyObservationsAllModes[REFD_REF_NONPRG].reserve(lLocalMapPoints.size());
    FoggyObservationsAllModes[REFD_MED_NONPRG].reserve(lLocalMapPoints.size());

    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;

        // do not process MPs that have been erased or set bad
        if (!pMP || pMP->isBad()) {
            continue;
        }

        // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)
        map<KeyFrame *, size_t> observations(pMP->GetObservations());     // use the copy constructor because the map will be modified later and we don't want to affect the MP's member variable
        map<KeyFrame *, size_t> observations_med(observations);     // use the copy constructor because the map will be modified later and we don't want to affect the MP's member variable

        // do not process MPs that have too few observations
        if (observations.size() < MIN_NUM_OBS) {
            continue;
        }

        std::vector<FoggyObservation> vFoggyObservation_Raw;
        std::vector<FoggyObservation> vFoggyObservation_Proj;
        std::vector<FoggyObservation> vFoggyObservation_Refd_Ref;
        std::vector<FoggyObservation> vFoggyObservation_Refd_Med;
        vFoggyObservation_Raw.reserve(observations.size());
        vFoggyObservation_Proj.reserve(observations.size());
        vFoggyObservation_Refd_Ref.reserve(observations.size());
        vFoggyObservation_Refd_Med.reserve(observations.size());

        // the reference KF from ORB-SLAM2's reference KF
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // find its ref KF

        PatchAligner pa = PatchAligner(pMP, pRefKF);     // instantiate a patch aligner object for Raw and Proj
        float ref_distance = FogEstimation::CalcDistMP2KF(pMP, pRefKF);
        float ref_intensity_raw = pa.GetRefRawIntensity();
        float ref_intensity_proj = pa.GetRefProjIntensity();
        int ref_octave = pa.GetRefOctave();
        float ref_gradient_squared_raw = pa.GetRefRawGradientSquared();
        float ref_gradient_squared_proj = pa.GetRefProjGradientSquared();
        if (ref_intensity_raw >= 0)
        {
            vFoggyObservation_Raw.emplace_back(pMP, pRefKF, ref_distance, ref_intensity_raw, ref_octave, ref_gradient_squared_raw);
        }
        if (ref_intensity_proj >= 0)
        {
            vFoggyObservation_Proj.emplace_back(pMP, pRefKF, ref_distance, ref_intensity_proj, ref_octave, ref_gradient_squared_proj);
        }

        PatchAligner pa_ref = PatchAligner(pMP, pRefKF, true);     // instantiate a patch aligner object for Refd_Ref (cannot use the pa for Raw and Proj because one is level specific while the other one is not)
        float ref_intensity_raw_ref = pa_ref.GetRefRawIntensity();
        int ref_octave_ref = pa_ref.GetRefOctave();
        float ref_gradient_squared_raw_ref = pa_ref.GetRefRawGradientSquared();
        if (ref_intensity_raw_ref >= 0)
        {
            vFoggyObservation_Refd_Ref.emplace_back(pMP, pRefKF, ref_distance, ref_intensity_raw_ref, ref_octave_ref, ref_gradient_squared_raw_ref);
        }

        observations.erase(pRefKF);     // erase the observation from the reference KF

        // the reference KF from the median distance
        // get the iterator of the median distance
        vector<pair<float, KeyFrame *>> vDistKFPairs;
        map<KeyFrame *, float> KFDist;
        vDistKFPairs.reserve(observations_med.size());
        for (map<KeyFrame *, size_t>::iterator mit = observations_med.begin(), mend = observations_med.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            float dist = FogEstimation::CalcDistMP2KF(pMP, pKF);
            vDistKFPairs.emplace_back(dist, pKF);
            KFDist.insert(make_pair(pKF, dist));
        }
        vector<pair<float, KeyFrame *>>::iterator itMedDistKF = vDistKFPairs.begin() + vDistKFPairs.size()/2;
        std::nth_element(vDistKFPairs.begin(), itMedDistKF , vDistKFPairs.end());
        // choose the reference KF
        KeyFrame *pRefKF_med = itMedDistKF->second;                  // use the KF with median distance as the reference KF
        PatchAligner pa_med = PatchAligner(pMP, pRefKF_med, true);     // instantiate a patch aligner object
        float ref_distance_med = KFDist[pRefKF_med];
        float ref_intensity_med = pa_med.GetRefRawIntensity();
        int ref_octave_med = pa_med.GetRefOctave();
        float ref_gradient_squared_med = pa_med.GetRefRawGradientSquared();
        if (ref_intensity_med >= 0)
        {
            vFoggyObservation_Refd_Med.emplace_back(pMP, pRefKF_med, ref_distance_med, ref_intensity_med, ref_octave_med, ref_gradient_squared_med);
        }
        observations_med.erase(pRefKF_med);

        // loop through all KFs (other than the reference one because it has already been erased) that can see this MP for Raw and Proj
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            KeyFrame *pQurKF = mit->first;

            pa.AddQueryKF(pQurKF);
            float qur_distance = FogEstimation::CalcDistMP2KF(pMP, pQurKF);
            float qur_intensity_raw = pa.GetQurRawIntensity();
            float qur_intensity_proj = pa.GetQurProjIntensity();
            int qur_octave = pa.GetQurOctave();
            if (qur_intensity_raw >= 0 && !vFoggyObservation_Raw.empty())
                vFoggyObservation_Raw.emplace_back(pMP, pQurKF, qur_distance, qur_intensity_raw, qur_octave);
            if (qur_intensity_proj >= 0 && !vFoggyObservation_Proj.empty())
                vFoggyObservation_Proj.emplace_back(pMP, pQurKF, qur_distance, qur_intensity_proj, qur_octave);
        }

        // loop through all KFs (other than the reference one because it has already been erased) that can see this MP for Refd_Ref
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
            KeyFrame *pQurKF = mit->first;
            if (!vFoggyObservation_Refd_Ref.empty())
                mFutures.push_back(std::async(std::launch::async, AlignRef, &vFoggyObservation_Refd_Ref, pMP, pRefKF, pQurKF));
        }
        mFutures.clear();

        // loop through all KFs (other than the median distance one because it has already been erased) that can see this MP for Refd_Med
        for (map<KeyFrame *, size_t>::iterator mit = observations_med.begin(), mend = observations_med.end(); mit != mend; mit++) {
            if (!vFoggyObservation_Refd_Med.empty())
                mFutures.push_back(std::async(std::launch::async, AlignMed, &vFoggyObservation_Refd_Med, pMP, pRefKF_med, mit->first, &KFDist));
        }
        mFutures.clear();

        // calculate intensity range
        float raw_max_intensity = vFoggyObservation_Raw[0].m_intensity, raw_min_intensity = vFoggyObservation_Raw[0].m_intensity;
        for (auto & vit : vFoggyObservation_Raw)
        {
            if (vit.m_intensity > raw_max_intensity)
                raw_max_intensity = vit.m_intensity;
            if (vit.m_intensity < raw_min_intensity)
                raw_min_intensity = vit.m_intensity;
        }
        float proj_max_intensity = vFoggyObservation_Proj[0].m_intensity, proj_min_intensity = vFoggyObservation_Proj[0].m_intensity;
        for (auto & vit : vFoggyObservation_Proj)
        {
            if (vit.m_intensity > proj_max_intensity)
                proj_max_intensity = vit.m_intensity;
            if (vit.m_intensity < proj_min_intensity)
                proj_min_intensity = vit.m_intensity;
        }
        float refd_ref_max_intensity = vFoggyObservation_Refd_Ref[0].m_intensity, refd_ref_min_intensity = vFoggyObservation_Refd_Ref[0].m_intensity;
        for (auto & vit : vFoggyObservation_Refd_Ref)
        {
            if (vit.m_intensity > refd_ref_max_intensity)
                refd_ref_max_intensity = vit.m_intensity;
            if (vit.m_intensity < refd_ref_min_intensity)
                refd_ref_min_intensity = vit.m_intensity;
        }
        float refd_med_max_intensity = vFoggyObservation_Refd_Med[0].m_intensity, refd_med_min_intensity = vFoggyObservation_Refd_Med[0].m_intensity;
        for (auto & vit : vFoggyObservation_Refd_Med)
        {
            if (vit.m_intensity > refd_med_max_intensity)
                refd_med_max_intensity = vit.m_intensity;
            if (vit.m_intensity < refd_med_min_intensity)
                refd_med_min_intensity = vit.m_intensity;
        }

        // record observations
        if (vFoggyObservation_Raw.size() >= MIN_NUM_OBS)
            FoggyObservationsAllModes[RAW].push_back(vFoggyObservation_Raw);
        if (vFoggyObservation_Proj.size() >= MIN_NUM_OBS)
            FoggyObservationsAllModes[PROJ].push_back(vFoggyObservation_Proj);
        if (vFoggyObservation_Refd_Ref.size() >= MIN_NUM_OBS)
            FoggyObservationsAllModes[REFD_REF_NONPRG].push_back(vFoggyObservation_Refd_Ref);
        if (vFoggyObservation_Refd_Med.size() >= MIN_NUM_OBS)
            FoggyObservationsAllModes[REFD_MED_NONPRG].push_back(vFoggyObservation_Refd_Med);
    }
}

void FogEstimation::InitialiseEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate();
        if (J_current == -1.0f)
            estimates.push_back(128.0f);    // this MP's J has never been estimated, use 128 to initialise
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    estimates.push_back(mCurrentEstimate.m_beta);
    estimates.push_back(mCurrentEstimate.m_atmos);
}

void FogEstimation::InitialiseEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    float intensity_max_dist; // to store the intensity of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate[intensity_mode].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!use_li && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(),vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensity_max_dist = vit_max_dist->m_intensity;
            }
        }
    }
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li)
        {
            // use the Li's modified method to initialise
            beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
            A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
        }
        else
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensity_max_dist;
            // initialise beta using the calculated value by the fog model evaluated at the median distance
//                beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates2(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate2(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate2[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate2[intensity_mode].m_atmos;
    if (beta_current == 0.02f && A_current == 200.0f)
    {
        // if never been estimated before, use the Li's modified method to initialise
        beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
        A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates3(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate3(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate3[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate3[intensity_mode].m_atmos;
    if (beta_current == 0.02f && A_current == 200.0f)
    {
        // if never been estimated before, use the Li's modified method to initialise
        beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
        A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates4(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    float intensity_max_dist; // to store the intensity of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate4(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate4[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate4[intensity_mode].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!use_li && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(),vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensity_max_dist = vit_max_dist->m_intensity;
            }
        }
    }
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li)
        {
            // use the Li's modified method to initialise
            beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
            A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
        }
        else
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensity_max_dist;
            // initialise beta using the calculated value by the fog model evaluated at the median distance
//                beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates5(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate5(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate5[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate5[intensity_mode].m_atmos;
    if (beta_current == 0.02f && A_current == 200.0f)
    {
        // if never been estimated before, use the Li's modified method to initialise
        beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
        A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates6(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    float intensity_max_dist; // to store the intensity of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate6(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate6[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate6[intensity_mode].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!use_li && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(),vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensity_max_dist = vit_max_dist->m_intensity;
            }
        }
    }
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li)
        {
            // use the Li's modified method to initialise
            beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
            A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
        }
        else
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensity_max_dist;
            // initialise beta using the calculated value by the fog model evaluated at the median distance
//                beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimates7(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li)
{
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    float intensity_max_dist; // to store the intensity of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate7(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentEstimate7[intensity_mode].m_beta;
    float A_current = mAllModesCurrentEstimate7[intensity_mode].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!use_li && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(),vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensity_max_dist = vit_max_dist->m_intensity;
            }
        }
    }
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li)
        {
            // use the Li's modified method to initialise
            beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
            A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
        }
        else
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensity_max_dist;
            // initialise beta using the calculated value by the fog model evaluated at the median distance
//                beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);
}
void FogEstimation::InitialiseEstimatesAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, array<vector<float>, 4> &estimates, eIntensityMode intensity_mode, bool use_li)
{
    // Js followed by beta then A
    for (int c = 0; c < 4; c++)
    {
        estimates[c].reserve(vvFoggyObservation.size()+2);
    }

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    cv::Vec4f intensities_max_dist; // to store the intensity values of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++) {
        cv::Vec4f J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimateAllChannels(intensity_mode);
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());   // the iterator with min distance
        for (int c = 0; c < 4; c++)
        {
            if (J_current[c] == -1.0f)  // this MP's this channel's J has never been estimated, use its foggy observation from the shortest distance to initialise
                estimates[c].push_back(vit_min_dist->m_intensities[c]);
            else                        // this MP's J has been estimated before, use the current estimate to initialise
                estimates[c].push_back(J_current[c]);
        }
    }

    // Beta and A
//    float median_dist, median_Js[3];
//    cv::Vec4f median_observed_intensities;

    float beta_current = mAllModesCurrentEstimateAllChannels[intensity_mode][0].m_beta;
    float A_current = mAllModesCurrentEstimateAllChannels[intensity_mode][0].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!use_li && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensities_max_dist = vit_max_dist->m_intensities;
                // keep track of the median distance and the corresponding intensity values which will be used to initialise beta
//                auto vit_med_dist = vFoggyObservation.begin() + vFoggyObservation.size()/2;
//                std::nth_element(vFoggyObservation.begin(), vit_med_dist, vFoggyObservation.end());
//                median_dist = vit_med_dist->m_dist;
//                median_observed_intensities = vit_med_dist->m_intensities;
//                for (int c = 0; c < 3; c++)
//                {
//                    median_Js[c] = estimates[c][i];
//                }
            }
        }
    }
    for (int c = 0; c < 4; c++)
    {
        beta_current = mAllModesCurrentEstimateAllChannels[intensity_mode][c].m_beta;
        A_current = mAllModesCurrentEstimateAllChannels[intensity_mode][c].m_atmos;
        if (!has_estimated_beta_A)      // if never been estimated before
        {
            if (use_li)
            {
                // use the Li's modified method to initialise
                beta_current = mAllModesCurrentEstimateLiMedianAllChannels[true][c].m_beta;
                A_current = mAllModesCurrentEstimateLiMedianAllChannels[true][c].m_atmos;
            }
            else
            {
                // use the intensity observed at the largest distance to initialise A
                A_current = intensities_max_dist[c];
                // initialise beta using the calculated value by the fog model evaluated at the median distance
//                beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
                beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
            }
        }
        estimates[c].push_back(beta_current);
        estimates[c].push_back(A_current);
    }
}
void FogEstimation::InitialiseEstimatesAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, array<vector<float>, 4> &estimates, eIntensityMode intensity_mode)
{
    // Js followed by beta then A
    for (int c = 0; c < 4; c++)
    {
        estimates[c].reserve(vvFoggyObservation.size()+2);
    }

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    cv::Vec4f intensities_max_dist; // to store the intensity values of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++) {
        cv::Vec4f J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimateAllChannelsWoGc(intensity_mode);
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());   // the iterator with min distance
        for (int c = 0; c < 4; c++)
        {
            if (J_current[c] == -1.0f)  // this MP's this channel's J has never been estimated, use its foggy observation from the shortest distance to initialise
                estimates[c].push_back(vit_min_dist->m_intensities[c]);
            else                        // this MP's J has been estimated before, use the current estimate to initialise
                estimates[c].push_back(J_current[c]);
        }
    }

    // Beta and A
//    float median_dist, median_Js[3];
//    cv::Vec4f median_observed_intensities;

    float beta_current = mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode][0].m_beta;
    float A_current = mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode][0].m_atmos;
    bool has_estimated_beta_A = (beta_current != 0.02f && A_current != 200.0f);
    if (!has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensities_max_dist = vit_max_dist->m_intensities;
                // keep track of the median distance and the corresponding intensity values which will be used to initialise beta
//                auto vit_med_dist = vFoggyObservation.begin() + vFoggyObservation.size()/2;
//                std::nth_element(vFoggyObservation.begin(), vit_med_dist, vFoggyObservation.end());
//                median_dist = vit_med_dist->m_dist;
//                median_observed_intensities = vit_med_dist->m_intensities;
//                for (int c = 0; c < 3; c++)
//                {
//                    median_Js[c] = estimates[c][i];
//                }
            }
        }
    }
    for (int c = 0; c < 4; c++)
    {
        beta_current = mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode][c].m_beta;
        A_current = mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode][c].m_atmos;
        if (!has_estimated_beta_A)      // if never been estimated before
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensities_max_dist[c];
            // initialise beta using the calculated value by the fog model evaluated at the median distance
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
        estimates[c].push_back(beta_current);
        estimates[c].push_back(A_current);
    }
}

void FogEstimation::OptimiseFog0(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation)
{
    // set g2o and optimizer
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 1>> Block;  // variable to optimise is of dimension max{1, 2}, error is of dimension 1
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(true);

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // add clear intensity vertices, each of which corresponds to a map point
    for (int i = 0; i < n_MPs; i++) {
        ClearVertex* vtxClear = new ClearVertex();
        float J_init = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimate();
        if (J_init == 0.0f)
            vtxClear->setEstimate(128.0f);  // this MP's J has never been estimated, use 128 to initialise
        else
            vtxClear->setEstimate(
                    J_init);  // this MP's J has been estimated before, use the current estimate to initialise
        vtxClear->setId(i);
        vtxClear->setMarginalized(true);          // need to set to true!
        optimizer.addVertex(vtxClear);
    }

    // add a (the only one) fog parameter vertex containing the scattering coefficient Beta and the atmospheric light A
    BetaAtmosVertex* vtxBetaAtmos = new BetaAtmosVertex();
    vtxBetaAtmos->setEstimate(Eigen::Vector2f(mCurrentEstimate.m_beta,
                                              mCurrentEstimate.m_atmos));    // use the current estimates to initialise
    vtxBetaAtmos->setId(n_MPs);
    optimizer.addVertex(vtxBetaAtmos);

    // add edges (each edge corresponds to an observation of a map point)
    int JVertex_count = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        int observation_count = 0;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            FoggyEdge *edgeFoggy = new FoggyEdge(vit->m_dist);
            edgeFoggy->setId(100 * JVertex_count + observation_count);  // to assure unique edgeFoggy ID
            edgeFoggy->setVertex(0, dynamic_cast<ClearVertex *>(optimizer.vertex(
                    JVertex_count))); // 1st vertex is ClearVertex
            edgeFoggy->setVertex(1, vtxBetaAtmos);         // 2nd vertex is the BetaAtmosVertex
            edgeFoggy->setMeasurement(vit->m_intensity);
            edgeFoggy->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                      mpCurrentKF->mvLevelSigma2[vit->m_octave]);      // TODO

            // Huber loss
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            edgeFoggy->setRobustKernel(rk);
            rk->setDelta(sqrt(TH_HUBER2));     // TODO

            optimizer.addEdge(edgeFoggy);
            observation_count++;
        }
        JVertex_count++;
    }

    // optimise 1st round
    optimizer.initializeOptimization();
    int n_Iters1 = optimizer.optimize(20);

    // print for debug
    std::cout << mpCurrentKF->mnFrameId
              << " " << mpCurrentKF->mnId
              << " " << vtxBetaAtmos->estimate()[0]
              << " " << vtxBetaAtmos->estimate()[1]
              << " " << optimizer.activeVertices().size() - 1
              << " " << optimizer.activeEdges().size()
              << " " << n_Iters1
              << std::endl;

    // retrieve estimated values
    // Js
    for (size_t i = 0; i < n_MPs; i++) {
        ClearVertex *currentJVertex = dynamic_cast<ClearVertex *>(optimizer.vertex(i));
        vvFoggyObservation[i][0].m_pMP->UpdateJEstimates(mpCurrentKF, currentJVertex->estimate());
    }
    // Beta and A
    Eigen::Vector2f BetaAtmos_hat = vtxBetaAtmos->estimate();
    mCurrentEstimate = BetaAtmos(BetaAtmos_hat[0], BetaAtmos_hat[1]);
    mEstimates.insert(std::make_pair(mpCurrentKF, mCurrentEstimate));

    mLastFogKFid = mpCurrentKF->mnId;
}

void FogEstimation::OptimiseFogCeresLoose(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes[intensity_mode].end())
            {
                mAllModesNumInlierTimes[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight));
#else
            // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // set bounds
    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
    for (int i = 0; i < Js.size(); ++i)
    {
        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
    }

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimes[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimes[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), 1.0));
#else
                // Analytic
                ceres::CostFunction* cost_function =
                        new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), 1.0);
#endif
                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
        problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
        for (int i = 0; i < Js2.size(); ++i)
        {
            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
        }

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = A;

        // update estimates
        UpdateEstimates(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

        mAllModesState[intensity_mode] = OK;
    }
    else {
        if (mAllModesState[intensity_mode] == OK)
            mAllModesState[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesState[intensity_mode] == OK
        || mAllModesState[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_loose";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_loose";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_loose";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_loose";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogG2o(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates2(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // set g2o and optimizer
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 1>> Block;  // variable to optimise is of dimension max{1, 2}, error is of dimension 1
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(true);

    // add clear intensity vertices, each of which corresponds to a map point
    for (int i = 0; i < n_MPs; i++) {
        ClearVertex* vtxClear = new ClearVertex();
        vtxClear->setEstimate(estimates[i]);
        vtxClear->setId(i);
        vtxClear->setMarginalized(true);          // need to set to true!
        optimizer.addVertex(vtxClear);
    }

    // add a (the only one) fog parameter vertex containing the scattering coefficient Beta and the atmospheric light A
    BetaAtmosVertex* vtxBetaAtmos = new BetaAtmosVertex();
    vtxBetaAtmos->setEstimate(Eigen::Vector2f(estimates.end()[-2], estimates.end()[-1]));
    vtxBetaAtmos->setId(n_MPs);
    optimizer.addVertex(vtxBetaAtmos);

    // containers to store edges, MPs and KFs
    const int nExpectedSize = n_MPs * 20;
    std::vector<FoggyEdge *> vpEdges;
    vpEdges.reserve(nExpectedSize);
    std::vector<MapPoint *> vpEdgeMPs;
    vpEdgeMPs.reserve(nExpectedSize);
    std::vector<KeyFrame *> vpEdgeKFs;
    vpEdgeKFs.reserve(nExpectedSize);
    std::vector<float> vEdgeWeights;
    vEdgeWeights.reserve(nExpectedSize);

    // compute the mean of all J vertices that have already been estimated before
//    int J_count = 0;
//    float J_sum = 0;
//    for (int n=0; n<estimates.size()-2; n++)
//    {
//        float J_current = estimates[n];
//        if (J_current != 128.0f)
//        {
//            J_sum += J_current;
//            J_count++;
//        }
//    }
//    float J_mean;
//    if (J_count != 0)
//        J_mean = J_sum / J_count;
//    else
//        J_mean = 128.0f;

    // add edges (each edge corresponds to an observation of a map point)
    float A_est = vtxBetaAtmos->estimate()[1];
    int JVertex_count = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        ClearVertex* current_vtxJ = dynamic_cast<ClearVertex*>(optimizer.vertex(JVertex_count));
        float current_J_est = current_vtxJ->estimate();
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        int observation_count = 0;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            FoggyEdge *edgeFoggy = new FoggyEdge(vit->m_dist);
            edgeFoggy->setId(100 * JVertex_count + observation_count);  // to assure unique edgeFoggy ID
            edgeFoggy->setVertex(0, current_vtxJ); // 1st vertex is ClearVertex
            edgeFoggy->setVertex(1, vtxBetaAtmos);         // 2nd vertex is the BetaAtmosVertex
            edgeFoggy->setMeasurement(vit->m_intensity);

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes2[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes2[intensity_mode].end())
            {
                mAllModesNumInlierTimes2[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes2[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(current_J_est - A_est);
//            float weight_J = std::fabs(current_J_est - A_est) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

            edgeFoggy->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * weight);      // TODO

            // Huber loss
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            edgeFoggy->setRobustKernel(rk);
            rk->setDelta(sqrt(TH_HUBER2));     // TODO

            optimizer.addEdge(edgeFoggy);
            observation_count++;

            vpEdges.push_back(edgeFoggy);
            vpEdgeMPs.push_back(current_pMP);
            vpEdgeKFs.push_back(pKF);
            vEdgeWeights.push_back(weight);
        }
        JVertex_count++;
    }

    // optimise 1st round
    optimizer.initializeOptimization();
    int n_Iters1 = optimizer.optimize(50);

    // check validity
    float beta_est = vtxBetaAtmos->estimate()[0];
    A_est = vtxBetaAtmos->estimate()[1];
    bool is_first_optimisation_valid = A_est >= ATMOS_LOW
                                       && A_est <= ATMOS_HIGH
                                       && beta_est >= BETA_LOW
                                       && beta_est <= BETA_HIGH;

    int num_inlier_MPs = 0, num_inlier_obs = 0;
    int n_Iters2 = 0;

    if (is_first_optimisation_valid) {
        // find outliers and do not optimise them in the second round
        for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
        {
            FoggyEdge* e = vpEdges[i];
            MapPoint* pMP = vpEdgeMPs[i];
            KeyFrame* pKF = vpEdgeKFs[i];
            float weight = vEdgeWeights[i];
            ClearVertex* vJ = dynamic_cast<ClearVertex*>(e->vertex(0));

            if(!pMP || pMP->isBad()) {
                e->setLevel(1);
                mAllModesNumInlierTimes2[intensity_mode][make_pair(pMP, pKF)] = 0;
                continue;
            }

            // if error is too big
            if(e->chi2()/weight > TH_HUBER2)     // TODO
            {
                // do not optimise in the next round
                e->setLevel(1);
            }
            else
            {
                // update mAllModesNumInlierTimes2[intensity_mode]
                mAllModesNumInlierTimes2[intensity_mode][make_pair(pMP, pKF)]++;
            }

            float weight_inlier_times = mAllModesNumInlierTimes2[intensity_mode][make_pair(pMP, pKF)] + 1;

            //
            float J_est = vJ->estimate();
            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());

            // do not use robust loss in the second round
            e->setRobustKernel(0);

            //        // update mAllModesQuality[intensity_mode]
            //        if(e->chi2()>TH_HUBER2)
            //            mAllModesQuality[intensity_mode][make_pair(pMP, pKF)] = false;
            //        else
            //            mAllModesQuality[intensity_mode][make_pair(pMP, pKF)] = true;
        }

        // check number of inlier observations of each MP; if too few inliers, do not optimise any observation of that MP in the second round
        std::vector<bool> vIsMPActive;
        vIsMPActive.reserve(n_MPs);
        for (int i = 0; i < n_MPs; i++)
        {
            ClearVertex* vJ = dynamic_cast<ClearVertex*>(optimizer.vertex(i));

            int num_inlier_obs_current_MP = 0;
            auto set_edges = vJ->edges();
            for (auto sit=set_edges.begin(); sit!=set_edges.end(); sit++)
            {
                FoggyEdge* foggy_edge = dynamic_cast<FoggyEdge*>(*sit);
                if (foggy_edge->level() == 0)
                    num_inlier_obs_current_MP++;
            }

            if (num_inlier_obs_current_MP < MIN_NUM_OBS)
            {
                for (auto sit=set_edges.begin(); sit!=set_edges.end(); sit++)
                {
                    FoggyEdge* foggy_edge = dynamic_cast<FoggyEdge*>(*sit);
                    foggy_edge->setLevel(1);
                }
                vIsMPActive[i] = false;
            }
            else
            {
                vIsMPActive[i] = true;
                num_inlier_MPs++;
                num_inlier_obs += num_inlier_obs_current_MP;
            }
        }

        // optimise 2nd round if there are enough inlier MPs
        if (num_inlier_MPs >= MIN_NUM_MPS) {
            optimizer.initializeOptimization(0);
            n_Iters2 = optimizer.optimize(50);

            // check validity
            beta_est = vtxBetaAtmos->estimate()[0];
            A_est = vtxBetaAtmos->estimate()[1];
            bool is_second_optimisation_valid = A_est >= ATMOS_LOW
                                                && A_est <= ATMOS_HIGH
                                                && beta_est >= BETA_LOW
                                                && beta_est <= BETA_HIGH;

            if (is_second_optimisation_valid) {
                // retrieve estimated values
                // Js
                for (size_t i = 0; i < n_MPs; i++) {
                    ClearVertex *currentJVertex = dynamic_cast<ClearVertex *>(optimizer.vertex(i));
                    estimates[i] = currentJVertex->estimate();
                }
                // Beta and A
                estimates.end()[-2] = beta_est;
                estimates.end()[-1] = A_est;

                // update estimates
                UpdateEstimates2(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

                mAllModesState2[intensity_mode] = OK;
            }
            else {
                if (mAllModesState2[intensity_mode] == OK)
                    mAllModesState2[intensity_mode] = NOT_UP_TO_DATE;
            }
        } else {
            if (mAllModesState2[intensity_mode] == OK)
                mAllModesState2[intensity_mode] = NOT_UP_TO_DATE;
        }
    }
    else {
        if (mAllModesState2[intensity_mode] == OK)
            mAllModesState2[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = num_inlier_MPs;
    n_obs = num_inlier_obs;
    n_iters = n_Iters2;

    if (mAllModesState2[intensity_mode] == OK
        || mAllModesState2[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserg2o";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserg2o";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserg2o";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserg2o";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogCeresUnbounded(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates3(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes3[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes3[intensity_mode].end())
            {
                mAllModesNumInlierTimes3[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes3[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight));
#else
            // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimes3[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimes3[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), 1.0));
#else
                // Analytic
                ceres::CostFunction* cost_function =
                        new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), 1.0);
#endif
                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = A;

        // update estimates
        UpdateEstimates3(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

        mAllModesState3[intensity_mode] = OK;
    }
    else {
        if (mAllModesState3[intensity_mode] == OK)
            mAllModesState3[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesState3[intensity_mode] == OK
        || mAllModesState3[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_unbounded";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_unbounded";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_unbounded";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_unbounded";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogCeresTight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
// #if SAVE_FOGGY_OBSERVATIONS_OUR_METHOD
//     std::ofstream ofs;
//     ofs.open("results/vvFoggyObservationRaw_all.txt", ios::trunc);
//     for (auto &vFoggyObservation : vvFoggyObservation)
//     {
//         ofs << " " << vFoggyObservation[0].m_pMP->mnId
//             << std::endl;
//         for (auto &foggyObservation : vFoggyObservation)
//         {
//             ofs << " " << foggyObservation.m_dist
//                 << " " << foggyObservation.m_intensity
//                 << std::endl;
//         }
//         ofs << std::endl;
//     }
//     ofs.close();
// #endif

    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations
    float A_lower_bound = -1.0f, A_higher_bound = -1.0f, A_lower_bound2 = -1.0f, A_higher_bound2 = -1.0f;

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates4(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    std::vector<float> vec_huber_th;
    vec_huber_th.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes4[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes4[intensity_mode].end())
            {
                mAllModesNumInlierTimes4[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes4[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight));
#else
            // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            float huber_th;
            if (APPLY_GAMMA_CORRECTION)
            {
                huber_th = sqrt(TH_HUBER2) * mPhotometricParams[3][0]*mPhotometricParams[3][1]
                           *pow(GammaCompression(vit->m_intensity), mPhotometricParams[3][1]-1.0f);
            }
            else
            {
                huber_th = sqrt(TH_HUBER2);
            }
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(huber_th), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
            vec_huber_th.push_back(huber_th);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    float j_low_gamma_corrected;
    float j_high_gamma_corrected;
    float atmos_low_gamma_corrected;
    float atmos_high_gamma_corrected;
    if (APPLY_GAMMA_CORRECTION)
    {
        j_low_gamma_corrected = GammaExpansion(J_LOW);
        j_high_gamma_corrected = GammaExpansion(J_HIGH);
        atmos_low_gamma_corrected = GammaExpansion(ATMOS_LOW);
        atmos_high_gamma_corrected = GammaExpansion(ATMOS_HIGH);
    }
    else
    {
        j_low_gamma_corrected = J_LOW;
        j_high_gamma_corrected = J_HIGH;
        atmos_low_gamma_corrected = ATMOS_LOW;
        atmos_high_gamma_corrected = ATMOS_HIGH;
    }

    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    std::vector<float> A_lower_bounds;
    std::vector<float> A_higher_bounds;
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope;
        if (APPLY_GAMMA_CORRECTION)
            slope = (FogEstimation::GammaCompression(I_max_dist) - FogEstimation::GammaCompression(I_min_dist)) / (max_dist - min_dist);
        else
            slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
            A_lower_bounds.push_back(I_max_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
            A_higher_bounds.push_back(I_max_dist);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
            problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
        }
    }

    if (!A_lower_bounds.empty())
    {
        auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
        std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
        A_lower_bound = *itMedianElement_lower_bounds;

//        A_lower_bound = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
    }
    else
    {
        A_lower_bound = atmos_low_gamma_corrected;
    }
    A_higher_bound = atmos_high_gamma_corrected;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound = *itMedianElement_higher_bounds;
//
//        A_higher_bound = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound = ATMOS_HIGH;
//    }
    problem.SetParameterLowerBound(&A, 0, A_lower_bound);
    problem.SetParameterUpperBound(&A, 0, A_higher_bound);
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimes4[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= vec_huber_th[count_obs])
                {
                    mAllModesNumInlierTimes4[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }


// #if SAVE_FOGGY_OBSERVATIONS_OUR_METHOD
//     ofs.open("results/vvFoggyObservationRaw_inliers.txt", ios::trunc);
//     for (auto &vFoggyObservation2 : vvFoggyObservation2)
//     {
//         ofs << " " << vFoggyObservation2[0].m_pMP->mnId
//             << std::endl;
//         for (auto &foggyObservation2 : vFoggyObservation2)
//         {
//             ofs << " " << foggyObservation2.m_dist
//                 << " " << foggyObservation2.m_intensity
//                 << std::endl;
//         }
//         ofs << std::endl;
//     }
//     ofs.close();
// #endif

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), 1.0));
#else
                // Analytic
                ceres::CostFunction* cost_function =
                        new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), 1.0);
#endif
                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
//        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js2.size(); ++i)
//        {
//            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        std::vector<float> A_lower_bounds2;
        std::vector<float> A_higher_bounds2;
        for (int i = 0; i < vvFoggyObservation2.size(); ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
            auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

            float slope;
            if (APPLY_GAMMA_CORRECTION)
                slope = (FogEstimation::GammaCompression(I_max_dist) - FogEstimation::GammaCompression(I_min_dist)) / (max_dist - min_dist);
            else
                slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                A_lower_bounds2.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
                A_higher_bounds2.push_back(I_max_dist);
            }
            else
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
            }
        }
        if (!A_lower_bounds2.empty())
        {
            auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
            std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
            A_lower_bound2 = *itMedianElement_lower_bounds2;

//            A_lower_bound2 = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
        }
        else
        {
            A_lower_bound2 = A_lower_bound;
        }
        A_higher_bound2 = atmos_high_gamma_corrected;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2 = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2 = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2 = A_higher_bound;
//        }
        problem2.SetParameterLowerBound(&A, 0, A_lower_bound2);
        problem2.SetParameterUpperBound(&A, 0, A_higher_bound2);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = A;

        // update estimates
        UpdateEstimates4(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

        mAllModesState4[intensity_mode] = OK;
    }
    else {
        if (mAllModesState4[intensity_mode] == OK)
            mAllModesState4[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesState4[intensity_mode] == OK
        || mAllModesState4[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();

        // // write A bounds to csv file
        // std::string csv_file_name;
        // switch (intensity_mode)
        // {
        //     case RAW:
        //         csv_file_name = "ABounds_IntensityModeraw";
        //         break;
        //     case PROJ:
        //         csv_file_name = "ABounds_IntensityModeproj";
        //         break;
        //     case REFD_REF_NONPRG:
        //         csv_file_name = "ABounds_IntensityModerefd_ref";
        //         break;
        //     case REFD_MED_NONPRG:
        //         csv_file_name = "ABounds_IntensityModerefd_med";
        //         break;
        //     default:
        //         break;
        // }
        // std::ofstream ofs_ABounds;
        // ofs_ABounds.open("results/"+csv_file_name+".txt", ios::app);
        // ofs_ABounds << mpCurrentKF->mnFrameId
        //             << " " << mpCurrentKF->mnId
        //             << " " << A_lower_bound
        //             << " " << A_higher_bound
        //             << " " << A_lower_bound2
        //             << " " << A_higher_bound2
        //             << std::endl;
        // ofs_ABounds.close();
        //
        // // also save the Js
        // std::ofstream ofs_Js;
        // ofs_Js.open("results/"+file_name+"_Js.txt", ios::trunc);
        // for (size_t i = 0; i < estimates.size()-2; i++) {
        //     if (vIsMPActive[i])
        //     {
        //         ofs_Js << vvFoggyObservation[i][0].m_pMP->mnId
        //                << " " << estimates[i]
        //                << std::endl;
        //     }
        // }
        // ofs_Js.close();
    }
}

void FogEstimation::OptimiseFogCeresTightProductWeight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates5(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes5[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes5[intensity_mode].end())
            {
                mAllModesNumInlierTimes5[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes5[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight));
#else
            // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    std::vector<float> A_lower_bounds;
    std::vector<float> A_higher_bounds;
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
            A_lower_bounds.push_back(I_max_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
            A_higher_bounds.push_back(I_max_dist);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
    }
    float A_lower_bound;
    if (!A_lower_bounds.empty())
    {
        auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
        std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
        A_lower_bound = *itMedianElement_lower_bounds;

//        A_lower_bound = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
    }
    else
    {
        A_lower_bound = ATMOS_LOW;
    }
    float A_higher_bound = ATMOS_HIGH;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound = *itMedianElement_higher_bounds;
//
//        A_higher_bound = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound = ATMOS_HIGH;
//    }
    problem.SetParameterLowerBound(&A, 0, A_lower_bound);
    problem.SetParameterUpperBound(&A, 0, A_higher_bound);
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimes5[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimes5[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    float A_lower_bound2 = ATMOS_LOW;
    float A_higher_bound2 = ATMOS_HIGH;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {

                float weight_J = std::fabs(Js2[count_MP] - A);
//            float weight_J = std::fabs(Js2[count_MP] - A) / 255.0f;
                float inlier_times_old = mAllModesNumInlierTimes5[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] - 1;
                float weight_inlier_times_old = 1.0f - 1.0f/(std::exp(inlier_times_old) + 1.0f);
                float inlier_times_new = mAllModesNumInlierTimes5[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)];
                float weight_inlier_times_new = inlier_times_new + 1.0f;
//            float weight_inlier_times_new = 1.0f - 1.0f/(std::exp(inlier_times_new) + 1.0f);
                float weight_new = weight_J * weight_inlier_times_new;

#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight_new));
#else
                // Analytic
                ceres::CostFunction* cost_function =
                        new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), 1.0);
#endif
                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
//        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js2.size(); ++i)
//        {
//            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        std::vector<float> A_lower_bounds2;
        std::vector<float> A_higher_bounds2;
        for (int i = 0; i < vvFoggyObservation2.size(); ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
            auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

            float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                A_lower_bounds2.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
                A_higher_bounds2.push_back(I_max_dist);
            }
            else
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
            }
        }
        if (!A_lower_bounds2.empty())
        {
            auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
            std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
            A_lower_bound2 = *itMedianElement_lower_bounds2;

//            A_lower_bound2 = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
        }
        else
        {
            A_lower_bound2 = A_lower_bound;
        }
        A_higher_bound2 = ATMOS_HIGH;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2 = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2 = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2 = A_higher_bound;
//        }
        problem2.SetParameterLowerBound(&A, 0, A_lower_bound2);
        problem2.SetParameterUpperBound(&A, 0, A_higher_bound2);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = A;

        // update estimates
        UpdateEstimates5(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

        mAllModesState5[intensity_mode] = OK;
    }
    else {
        if (mAllModesState5[intensity_mode] == OK)
            mAllModesState5[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesState5[intensity_mode] == OK
        || mAllModesState5[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproduct-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproduct-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproduct-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproduct-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogCeresTightUniformWeight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates6(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), 1.0));
#else
            // Analytic
        ceres::CostFunction* cost_function =
                new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    std::vector<float> A_lower_bounds;
    std::vector<float> A_higher_bounds;
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
            A_lower_bounds.push_back(I_max_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
            A_higher_bounds.push_back(I_max_dist);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
    }
    float A_lower_bound;
    if (!A_lower_bounds.empty())
    {
        auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
        std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
        A_lower_bound = *itMedianElement_lower_bounds;

//        A_lower_bound = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
    }
    else
    {
        A_lower_bound = ATMOS_LOW;
    }
    float A_higher_bound = ATMOS_HIGH;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound = *itMedianElement_higher_bounds;
//
//        A_higher_bound = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound = ATMOS_HIGH;
//    }
    problem.SetParameterLowerBound(&A, 0, A_lower_bound);
    problem.SetParameterUpperBound(&A, 0, A_higher_bound);
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimes6[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimes6[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    float A_lower_bound2 = ATMOS_LOW;
    float A_higher_bound2 = ATMOS_HIGH;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), 1.0));
#else
                // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), 1.0);
#endif
                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
//        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js2.size(); ++i)
//        {
//            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        std::vector<float> A_lower_bounds2;
        std::vector<float> A_higher_bounds2;
        for (int i = 0; i < vvFoggyObservation2.size(); ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
            auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

            float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                A_lower_bounds2.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
                A_higher_bounds2.push_back(I_max_dist);
            }
            else
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
            }
        }
        if (!A_lower_bounds2.empty())
        {
            auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
            std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
            A_lower_bound2 = *itMedianElement_lower_bounds2;

//            A_lower_bound2 = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
        }
        else
        {
            A_lower_bound2 = A_lower_bound;
        }
        A_higher_bound2 = ATMOS_HIGH;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2 = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2 = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2 = A_higher_bound;
//        }
        problem2.SetParameterLowerBound(&A, 0, A_lower_bound2);
        problem2.SetParameterUpperBound(&A, 0, A_higher_bound2);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = A;

        // update estimates
        UpdateEstimates6(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

        mAllModesState6[intensity_mode] = OK;
    }
    else {
        if (mAllModesState6[intensity_mode] == OK)
            mAllModesState6[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesState6[intensity_mode] == OK
        || mAllModesState6[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightuniform-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightuniform-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightuniform-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightuniform-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogCeresTightOneStage(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    InitialiseEstimates7(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimes7[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimes7[intensity_mode].end())
            {
                mAllModesNumInlierTimes7[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimes7[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                            new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensity), weight));
#else
            // Analytic
        ceres::CostFunction* cost_function =
                new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensity), weight);
#endif
            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta, &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    std::vector<float> A_lower_bounds;
    std::vector<float> A_higher_bounds;
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
            A_lower_bounds.push_back(I_max_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
            A_higher_bounds.push_back(I_max_dist);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
    }
    float A_lower_bound;
    if (!A_lower_bounds.empty())
    {
        auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
        std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
        A_lower_bound = *itMedianElement_lower_bounds;

//        A_lower_bound = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
    }
    else
    {
        A_lower_bound = ATMOS_LOW;
    }
    float A_higher_bound = ATMOS_HIGH;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound = *itMedianElement_higher_bounds;
//
//        A_higher_bound = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound = ATMOS_HIGH;
//    }
    problem.SetParameterLowerBound(&A, 0, A_lower_bound);
    problem.SetParameterUpperBound(&A, 0, A_higher_bound);
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    std::vector<bool> vIsMPActive(n_MPs, true);    // all MPs are active in this case because of one-stage
    int count_obs = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || (vit->m_pMP)->isBad())
            {
                mAllModesNumInlierTimes7[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                    mAllModesNumInlierTimes7[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
            }

            count_obs++;
        }
    }

    // retrieve estimated values
    for (size_t i = 0; i < n_MPs; i++) {
        estimates[i] = Js[i];
    }
    // Beta and A
    estimates.end()[-2] = beta;
    estimates.end()[-1] = A;

    // update estimates
    UpdateEstimates7(vvFoggyObservation, estimates, vIsMPActive, intensity_mode);

    mAllModesState7[intensity_mode] = OK;

    // for printing/writing to txt file
    n_Js = n_MPs;
    n_obs = vec_residual_block_id.size();
    n_iters = summary.iterations.size();

    if (mAllModesState7[intensity_mode] == OK
        || mAllModesState7[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage1-Weightproduct-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage1-Weightproduct-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage1-Weightproduct-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage1-Weightproduct-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseFogCeresTightAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    array<int, 4> n_Js = {-1, -1, -1, -1}, n_obs = {-1, -1, -1, -1}, n_iters = {-1, -1, -1, -1};       // number of MPs, number of observations, number of iterations
    array<float, 4> A_lower_bound = {-1, -1, -1, -1}, A_higher_bound = {-1, -1, -1, -1}, A_lower_bound2 = {-1, -1, -1, -1}, A_higher_bound2 = {-1, -1, -1, -1};

    array<vector<float>, 4> estimates;

    // initialise estimates
    InitialiseEstimatesAllChannels(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    for (int c = 0; c < 4; c++)
    {
        std::vector<double> Js(estimates[c].begin(), estimates[c].end()-2);
        double beta = estimates[c].end()[-2];
        double A = estimates[c].end()[-1];

        ceres::Problem problem;
        std::vector<ceres::ResidualBlockId> vec_residual_block_id;
        vec_residual_block_id.reserve(20 * n_MPs);
        std::vector<float> vec_weight;
        vec_weight.reserve(20 * n_MPs);
        std::vector<float> vec_huber_th;
        vec_huber_th.reserve(20 * n_MPs);
        int count_MP = 0;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
             vvit != vvFoggyObservation.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation = *vvit;
            MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                 vit != vFoggyObservation.end(); vit++) {

                // check the number of inlier times
                KeyFrame* pKF = vit->m_pKF;
                auto mit = mAllModesNumInlierTimesAllChannels[intensity_mode].find(make_pair(current_pMP, pKF));
                if (mit == mAllModesNumInlierTimesAllChannels[intensity_mode].end())
                {
                    mAllModesNumInlierTimesAllChannels[intensity_mode][make_pair(current_pMP, pKF)][c] = 0;   // this is a new observation so set to 0
                }
                float inlier_times = mAllModesNumInlierTimesAllChannels[intensity_mode][make_pair(current_pMP, pKF)][c];

                // calculate the overall weight
                float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
                float weight_gradient = MU / (MU + vit->m_gradient_squared);
                float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
                float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensities[c]), weight));
#else
                // Analytic
        ceres::CostFunction* cost_function =
                new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensities[c]), weight);
#endif
                float huber_th;
                if (APPLY_GAMMA_CORRECTION)
                {
                    huber_th = sqrt(TH_HUBER2) * mPhotometricParams[c][0]*mPhotometricParams[c][1]
                               *pow(GammaCompression(vit->m_intensities[c], c), mPhotometricParams[c][1]-1.0f);
                }
                else
                {
                    huber_th = sqrt(TH_HUBER2);
                }
                ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(huber_th), &Js[count_MP], &beta, &A);
                vec_residual_block_id.push_back(residual_block_id);

                vec_weight.push_back(weight);
                vec_huber_th.push_back(huber_th);
            }
            count_MP++;
        }

        // set bounds
//        problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js.size(); ++i)
//        {
//            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        float j_low_gamma_corrected;
        float j_high_gamma_corrected;
        float atmos_low_gamma_corrected;
        float atmos_high_gamma_corrected;
        if (APPLY_GAMMA_CORRECTION)
        {
            j_low_gamma_corrected = GammaExpansion(J_LOW, c);
            j_high_gamma_corrected = GammaExpansion(J_HIGH, c);
            atmos_low_gamma_corrected = GammaExpansion(ATMOS_LOW, c);
            atmos_high_gamma_corrected = GammaExpansion(ATMOS_HIGH, c);
        }
        else
        {
            j_low_gamma_corrected = J_LOW;
            j_high_gamma_corrected = J_HIGH;
            atmos_low_gamma_corrected = ATMOS_LOW;
            atmos_high_gamma_corrected = ATMOS_HIGH;
        }


        problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        std::vector<float> A_lower_bounds;
        std::vector<float> A_higher_bounds;
        for (int i = 0; i < n_MPs; ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensities[c];    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensities[c];    // I at the max distance

            float slope;
            if (APPLY_GAMMA_CORRECTION)
                slope = (FogEstimation::GammaCompression(I_max_dist, c) - FogEstimation::GammaCompression(I_min_dist, c)) / (max_dist - min_dist);
            else
                slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
                problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
                A_lower_bounds.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
                problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
                A_higher_bounds.push_back(I_max_dist);
            }
            else
            {
                problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
                problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
            }
        }
        if (!A_lower_bounds.empty())
        {
            auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
            std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
            A_lower_bound[c] = *itMedianElement_lower_bounds;

//        A_lower_bound[c] = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
        }
        else
        {
            A_lower_bound[c] = atmos_low_gamma_corrected;
        }
        A_higher_bound[c] = atmos_high_gamma_corrected;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound[c] = *itMedianElement_higher_bounds;
//
//        A_higher_bound[c] = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound[c] = ATMOS_HIGH;
//    }
        problem.SetParameterLowerBound(&A, 0, A_lower_bound[c]);
        problem.SetParameterUpperBound(&A, 0, A_higher_bound[c]);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options;
        options.max_num_iterations = 50;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

        // evaluate to get residuals
        ceres::Problem::EvaluateOptions evaluate_options;
        evaluate_options.residual_blocks = vec_residual_block_id;
        evaluate_options.apply_loss_function = false;       // the default is true
        std::vector<double> residuals;
        problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

        // check residuals and update inlier count
        count_MP = 0;
        int count_obs = 0;
        std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
        vvFoggyObservation2.reserve(n_MPs);
        std::vector<double> Js2;
        Js2.reserve(n_MPs);
        std::vector<bool> vIsMPActive(n_MPs, false);
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
             vvit != vvFoggyObservation.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2;

            std::vector<FoggyObservation> vFoggyObservation = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                 vit != vFoggyObservation.end(); vit++) {
                if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
                {
                    mAllModesNumInlierTimesAllChannels[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)][c] = 0;
                }
                else
                {
                    if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= vec_huber_th[count_obs])
                    {
                        mAllModesNumInlierTimesAllChannels[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)][c]++;
                        vFoggyObservation2.push_back(*vit);
                    }
                }
                count_obs++;
            }

            if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
                vvFoggyObservation2.push_back(vFoggyObservation2);
                Js2.push_back(Js[count_MP]);
                vIsMPActive[count_MP] = true;
            }

            count_MP++;
        }

        count_MP = 0;
        int num_inlier_obs = 0;
        int num_iterations = 0;
        if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
        {
            // 2nd round
            ceres::Problem problem2;
            for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
                 vvit != vvFoggyObservation2.end(); vvit++) {
                std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
                for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                     vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                    // Auto
                    ceres::CostFunction* cost_function =
                            new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                    new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensities[c]), 1.0));
#else
                    // Analytic
            ceres::CostFunction* cost_function =
                    new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensities[c]), 1.0);
#endif
                    ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                    num_inlier_obs++;
                }
                count_MP++;
            }

            // set bounds
//            problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//            problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//            problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//            problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//            for (int i = 0; i < Js2.size(); ++i)
//            {
//                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//            }

            /* ---> set tighter bounds */
            problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
            problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
            std::vector<float> A_lower_bounds2;
            std::vector<float> A_higher_bounds2;
            for (int i = 0; i < vvFoggyObservation2.size(); ++i)
            {
                // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
                std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
                auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
                float min_dist = vit_min_dist->m_dist;           // min distance
                float I_min_dist = vit_min_dist->m_intensities[c];    // I at the min distance
                auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
                float max_dist = vit_max_dist->m_dist;           // max distance
                float I_max_dist = vit_max_dist->m_intensities[c];    // I at the max distance

                float slope;
                if (APPLY_GAMMA_CORRECTION)
                    slope = (FogEstimation::GammaCompression(I_max_dist, c) - FogEstimation::GammaCompression(I_min_dist, c)) / (max_dist - min_dist);
                else
                    slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

                if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                    problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                    A_lower_bounds2.push_back(I_max_dist);
                }
                else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                    problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
                    A_higher_bounds2.push_back(I_max_dist);
                }
                else
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                    problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
                }
            }
            if (!A_lower_bounds2.empty())
            {
                auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
                std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
                A_lower_bound2[c] = *itMedianElement_lower_bounds2;

//            A_lower_bound2[c] = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
            }
            else
            {
                A_lower_bound2[c] = A_lower_bound[c];
            }
            A_higher_bound2[c] = atmos_high_gamma_corrected;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2[c] = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2[c] = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2[c] = A_higher_bound[c];
//        }
            problem2.SetParameterLowerBound(&A, 0, A_lower_bound2[c]);
            problem2.SetParameterUpperBound(&A, 0, A_higher_bound2[c]);
            /* ---< set tighter bounds */

            // optimise
            ceres::Solver::Options options2;
            options2.max_num_iterations = 50;
            options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary2;
            ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

            num_iterations = summary2.iterations.size();

            // retrieve estimated values
            int valid_count = 0;
            for (int i=0; i<estimates[c].size()-2; i++)
            {
                if (vIsMPActive[i])
                {
                    estimates[c][i] = Js2[valid_count];
                    valid_count++;
                }
            }
            estimates[c].end()[-2] = beta;
            estimates[c].end()[-1] = A;

            // update estimates
            UpdateEstimatesAllChannels(vvFoggyObservation, estimates[c], vIsMPActive, intensity_mode, c);

            mAllModesStateAllChannels[intensity_mode][c] = OK;
        }
        else {
            if (mAllModesStateAllChannels[intensity_mode][c] == OK)
                mAllModesStateAllChannels[intensity_mode][c] = NOT_UP_TO_DATE;
        }

        // for printing/writing to txt file
        n_Js[c] = vvFoggyObservation2.size();
        n_obs[c] = num_inlier_obs;
        n_iters[c] = num_iterations;
    }

    // End timer
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate duration
    std::chrono::duration<double> duration = end - start;

    // write results to text file
    std::string file_name;
    switch (intensity_mode)
    {
        case RAW:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight";
            break;
        case PROJ:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_tight";
            break;
        case REFD_REF_NONPRG:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_tight";
            break;
        case REFD_MED_NONPRG:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_tight";
            break;
        default:
            break;
    }
    std::ofstream ofs;
    ofs.open("results/"+file_name+".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId
        // first channel B
        << " " << estimates[0].end()[-2]
        << " " << estimates[0].end()[-1]
        << " " << n_Js[0]
        << " " << n_obs[0]
        << " " << n_iters[0]
        // second channel G
        << " " << estimates[1].end()[-2]
        << " " << estimates[1].end()[-1]
        << " " << n_Js[1]
        << " " << n_obs[1]
        << " " << n_iters[1]
        // third channel R
        << " " << estimates[2].end()[-2]
        << " " << estimates[2].end()[-1]
        << " " << n_Js[2]
        << " " << n_obs[2]
        << " " << n_iters[2]
        // last channel grey
        << " " << estimates[3].end()[-2]
        << " " << estimates[3].end()[-1]
        << " " << n_Js[3]
        << " " << n_obs[3]
        << " " << n_iters[3]
        // optimisation time
        // << " " << duration.count()
        << std::endl;
    ofs.close();

    // write A bounds to csv file
    // std::string csv_file_name;
    // switch (intensity_mode)
    // {
    //     case RAW:
    //         csv_file_name = "ABounds_IntensityModeraw";
    //         break;
    //     case PROJ:
    //         csv_file_name = "ABounds_IntensityModeproj";
    //         break;
    //     case REFD_REF_NONPRG:
    //         csv_file_name = "ABounds_IntensityModerefd_ref";
    //         break;
    //     case REFD_MED_NONPRG:
    //         csv_file_name = "ABounds_IntensityModerefd_med";
    //         break;
    //     default:
    //         break;
    // }
    // std::ofstream ofs_ABounds;
    // ofs_ABounds.open("results/"+csv_file_name+".txt", ios::app);
    // ofs_ABounds << mpCurrentKF->mnFrameId
    //             << " " << mpCurrentKF->mnId
    //             // first channel
    //             << " " << A_lower_bound[0]
    //             << " " << A_higher_bound[0]
    //             << " " << A_lower_bound2[0]
    //             << " " << A_higher_bound2[0]
    //             // second channel
    //             << " " << A_lower_bound[1]
    //             << " " << A_higher_bound[1]
    //             << " " << A_lower_bound2[1]
    //             << " " << A_higher_bound2[1]
    //             // third channel
    //             << " " << A_lower_bound[2]
    //             << " " << A_higher_bound[2]
    //             << " " << A_lower_bound2[2]
    //             << " " << A_higher_bound2[2]
    //             // last channel grey
    //             << " " << A_lower_bound[3]
    //             << " " << A_higher_bound[3]
    //             << " " << A_lower_bound2[3]
    //             << " " << A_higher_bound2[3]
    //             << std::endl;
    // ofs_ABounds.close();
}

void FogEstimation::OptimiseFogCeresTightAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode)
{
    // for printing/writing to txt file
    array<int, 4> n_Js = {-1, -1, -1, -1}, n_obs = {-1, -1, -1, -1}, n_iters = {-1, -1, -1, -1};       // number of MPs, number of observations, number of iterations
    array<float, 4> A_lower_bound = {-1, -1, -1, -1}, A_higher_bound = {-1, -1, -1, -1}, A_lower_bound2 = {-1, -1, -1, -1}, A_higher_bound2 = {-1, -1, -1, -1};

    array<vector<float>, 4> estimates;

    // initialise estimates
    InitialiseEstimatesAllChannelsWoGc(vvFoggyObservation, estimates, intensity_mode);
    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    for (int c = 0; c < 4; c++)
    {
        std::vector<double> Js(estimates[c].begin(), estimates[c].end()-2);
        double beta = estimates[c].end()[-2];
        double A = estimates[c].end()[-1];

        ceres::Problem problem;
        std::vector<ceres::ResidualBlockId> vec_residual_block_id;
        vec_residual_block_id.reserve(20 * n_MPs);
        std::vector<float> vec_weight;
        vec_weight.reserve(20 * n_MPs);
        std::vector<float> vec_huber_th;
        vec_huber_th.reserve(20 * n_MPs);
        int count_MP = 0;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
             vvit != vvFoggyObservation.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation = *vvit;
            MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                 vit != vFoggyObservation.end(); vit++) {

                // check the number of inlier times
                KeyFrame* pKF = vit->m_pKF;
                auto mit = mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode].find(make_pair(current_pMP, pKF));
                if (mit == mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode].end())
                {
                    mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode][make_pair(current_pMP, pKF)][c] = 0;   // this is a new observation so set to 0
                }
                float inlier_times = mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode][make_pair(current_pMP, pKF)][c];

                // calculate the overall weight
                float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
                float weight_gradient = MU / (MU + vit->m_gradient_squared);
                float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
                float weight = weight_J * weight_inlier_times;

#if CERES_USE_AUTO_DIFF
                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensities[c]), weight));
#else
                // Analytic
    ceres::CostFunction* cost_function =
            new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensities[c]), weight);
#endif
                float huber_th;
                huber_th = sqrt(TH_HUBER2);
                ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(huber_th), &Js[count_MP], &beta, &A);
                vec_residual_block_id.push_back(residual_block_id);

                vec_weight.push_back(weight);
                vec_huber_th.push_back(huber_th);
            }
            count_MP++;
        }

        // set bounds
//        problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js.size(); ++i)
//        {
//            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        float j_low_gamma_corrected;
        float j_high_gamma_corrected;
        float atmos_low_gamma_corrected;
        float atmos_high_gamma_corrected;
        j_low_gamma_corrected = J_LOW;
        j_high_gamma_corrected = J_HIGH;
        atmos_low_gamma_corrected = ATMOS_LOW;
        atmos_high_gamma_corrected = ATMOS_HIGH;


        problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        std::vector<float> A_lower_bounds;
        std::vector<float> A_higher_bounds;
        for (int i = 0; i < n_MPs; ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensities[c];    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensities[c];    // I at the max distance

            float slope;
            slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
                problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
                A_lower_bounds.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
                problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
                A_higher_bounds.push_back(I_max_dist);
            }
            else
            {
                problem.SetParameterLowerBound(&Js[i], 0, j_low_gamma_corrected);
                problem.SetParameterUpperBound(&Js[i], 0, j_high_gamma_corrected);
            }
        }
        if (!A_lower_bounds.empty())
        {
            auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
            std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
            A_lower_bound[c] = *itMedianElement_lower_bounds;

//        A_lower_bound[c] = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
        }
        else
        {
            A_lower_bound[c] = atmos_low_gamma_corrected;
        }
        A_higher_bound[c] = atmos_high_gamma_corrected;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound[c] = *itMedianElement_higher_bounds;
//
//        A_higher_bound[c] = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound[c] = ATMOS_HIGH;
//    }
        problem.SetParameterLowerBound(&A, 0, A_lower_bound[c]);
        problem.SetParameterUpperBound(&A, 0, A_higher_bound[c]);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options;
        options.max_num_iterations = 50;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

        // evaluate to get residuals
        ceres::Problem::EvaluateOptions evaluate_options;
        evaluate_options.residual_blocks = vec_residual_block_id;
        evaluate_options.apply_loss_function = false;       // the default is true
        std::vector<double> residuals;
        problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

        // check residuals and update inlier count
        count_MP = 0;
        int count_obs = 0;
        std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
        vvFoggyObservation2.reserve(n_MPs);
        std::vector<double> Js2;
        Js2.reserve(n_MPs);
        std::vector<bool> vIsMPActive(n_MPs, false);
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
             vvit != vvFoggyObservation.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2;

            std::vector<FoggyObservation> vFoggyObservation = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
                 vit != vFoggyObservation.end(); vit++) {
                if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
                {
                    mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)][c] = 0;
                }
                else
                {
                    if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= vec_huber_th[count_obs])
                    {
                        mAllModesNumInlierTimesAllChannelsWoGc[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)][c]++;
                        vFoggyObservation2.push_back(*vit);
                    }
                }
                count_obs++;
            }

            if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
                vvFoggyObservation2.push_back(vFoggyObservation2);
                Js2.push_back(Js[count_MP]);
                vIsMPActive[count_MP] = true;
            }

            count_MP++;
        }

        count_MP = 0;
        int num_inlier_obs = 0;
        int num_iterations = 0;
        if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
        {
            // 2nd round
            ceres::Problem problem2;
            for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
                 vvit != vvFoggyObservation2.end(); vvit++) {
                std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
                for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                     vit != vFoggyObservation2.end(); vit++) {
#if CERES_USE_AUTO_DIFF
                    // Auto
                    ceres::CostFunction* cost_function =
                            new ceres::AutoDiffCostFunction<FoggyResidualAuto, 1, 1, 1, 1>(
                                    new FoggyResidualAuto(double(vit->m_dist), double(vit->m_intensities[c]), 1.0));
#else
                    // Analytic
        ceres::CostFunction* cost_function =
                new FoggyResidualAnalytic(double(vit->m_dist), double(vit->m_intensities[c]), 1.0);
#endif
                    ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta, &A);

                    num_inlier_obs++;
                }
                count_MP++;
            }

            // set bounds
//            problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//            problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//            problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//            problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//            for (int i = 0; i < Js2.size(); ++i)
//            {
//                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//            }

            /* ---> set tighter bounds */
            problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
            problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
            std::vector<float> A_lower_bounds2;
            std::vector<float> A_higher_bounds2;
            for (int i = 0; i < vvFoggyObservation2.size(); ++i)
            {
                // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
                std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
                auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
                float min_dist = vit_min_dist->m_dist;           // min distance
                float I_min_dist = vit_min_dist->m_intensities[c];    // I at the min distance
                auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
                float max_dist = vit_max_dist->m_dist;           // max distance
                float I_max_dist = vit_max_dist->m_intensities[c];    // I at the max distance

                float slope;
                slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

                if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                    problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                    A_lower_bounds2.push_back(I_max_dist);
                }
                else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                    problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
                    A_higher_bounds2.push_back(I_max_dist);
                }
                else
                {
                    problem2.SetParameterLowerBound(&Js2[i], 0, j_low_gamma_corrected);
                    problem2.SetParameterUpperBound(&Js2[i], 0, j_high_gamma_corrected);
                }
            }
            if (!A_lower_bounds2.empty())
            {
                auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
                std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
                A_lower_bound2[c] = *itMedianElement_lower_bounds2;

//            A_lower_bound2[c] = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
            }
            else
            {
                A_lower_bound2[c] = A_lower_bound[c];
            }
            A_higher_bound2[c] = atmos_high_gamma_corrected;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2[c] = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2[c] = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2[c] = A_higher_bound[c];
//        }
            problem2.SetParameterLowerBound(&A, 0, A_lower_bound2[c]);
            problem2.SetParameterUpperBound(&A, 0, A_higher_bound2[c]);
            /* ---< set tighter bounds */

            // optimise
            ceres::Solver::Options options2;
            options2.max_num_iterations = 50;
            options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary2;
            ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

            num_iterations = summary2.iterations.size();

            // retrieve estimated values
            int valid_count = 0;
            for (int i=0; i<estimates[c].size()-2; i++)
            {
                if (vIsMPActive[i])
                {
                    estimates[c][i] = Js2[valid_count];
                    valid_count++;
                }
            }
            estimates[c].end()[-2] = beta;
            estimates[c].end()[-1] = A;

            // update estimates
            UpdateEstimatesAllChannelsWoGc(vvFoggyObservation, estimates[c], vIsMPActive, intensity_mode, c);

            mAllModesStateAllChannelsWoGc[intensity_mode][c] = OK;
        }
        else {
            if (mAllModesStateAllChannelsWoGc[intensity_mode][c] == OK)
                mAllModesStateAllChannelsWoGc[intensity_mode][c] = NOT_UP_TO_DATE;
        }

        // for printing/writing to txt file
        n_Js[c] = vvFoggyObservation2.size();
        n_obs[c] = num_inlier_obs;
        n_iters[c] = num_iterations;
    }

    // write results to text file
    std::string file_name;
    switch (intensity_mode)
    {
        case RAW:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight-WoGc";
            break;
        case PROJ:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_tight-WoGc";
            break;
        case REFD_REF_NONPRG:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_tight-WoGc";
            break;
        case REFD_MED_NONPRG:
            file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_tight-WoGc";
            break;
        default:
            break;
    }
    std::ofstream ofs;
    ofs.open("results/"+file_name+".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId
        // first channel B
        << " " << estimates[0].end()[-2]
        << " " << estimates[0].end()[-1]
        << " " << n_Js[0]
        << " " << n_obs[0]
        << " " << n_iters[0]
        // second channel G
        << " " << estimates[1].end()[-2]
        << " " << estimates[1].end()[-1]
        << " " << n_Js[1]
        << " " << n_obs[1]
        << " " << n_iters[1]
        // third channel R
        << " " << estimates[2].end()[-2]
        << " " << estimates[2].end()[-1]
        << " " << n_Js[2]
        << " " << n_obs[2]
        << " " << n_iters[2]
        // last channel grey
        << " " << estimates[3].end()[-2]
        << " " << estimates[3].end()[-1]
        << " " << n_Js[3]
        << " " << n_obs[3]
        << " " << n_iters[3]
        << std::endl;
    ofs.close();

    // write A bounds to csv file
    // std::string csv_file_name;
    // switch (intensity_mode)
    // {
    //     case RAW:
    //         csv_file_name = "ABounds_IntensityModeraw_WoGc";
    //         break;
    //     case PROJ:
    //         csv_file_name = "ABounds_IntensityModeproj_WoGc";
    //         break;
    //     case REFD_REF_NONPRG:
    //         csv_file_name = "ABounds_IntensityModerefd_ref_WoGc";
    //         break;
    //     case REFD_MED_NONPRG:
    //         csv_file_name = "ABounds_IntensityModerefd_med_WoGc";
    //         break;
    //     default:
    //         break;
    // }
    // std::ofstream ofs_ABounds;
    // ofs_ABounds.open("results/"+csv_file_name+".txt", ios::app);
    // ofs_ABounds << mpCurrentKF->mnFrameId
    //             << " " << mpCurrentKF->mnId
    //             // first channel
    //             << " " << A_lower_bound[0]
    //             << " " << A_higher_bound[0]
    //             << " " << A_lower_bound2[0]
    //             << " " << A_higher_bound2[0]
    //             // second channel
    //             << " " << A_lower_bound[1]
    //             << " " << A_higher_bound[1]
    //             << " " << A_lower_bound2[1]
    //             << " " << A_higher_bound2[1]
    //             // third channel
    //             << " " << A_lower_bound[2]
    //             << " " << A_higher_bound[2]
    //             << " " << A_lower_bound2[2]
    //             << " " << A_higher_bound2[2]
    //             // last channel grey
    //             << " " << A_lower_bound[3]
    //             << " " << A_higher_bound[3]
    //             << " " << A_lower_bound2[3]
    //             << " " << A_higher_bound2[3]
    //             << std::endl;
    // ofs_ABounds.close();
}

#if TEST_PARTIAL_GT
void FogEstimation::OptimiseBeta(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode, bool use_li_to_init)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimateBeta(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = mAllModesCurrentBetaEstimate[intensity_mode].m_beta;
    float A_current = GROUNDTRUTH_A;
    bool has_estimated_beta_A = (beta_current != 0.02f);
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li_to_init)
        {
            // use the Li's modified method to initialise
            beta_current = mAllModesCurrentEstimateLiMedian[true].m_beta;
        }
        else
        {
//            beta_current = -log((median_observed_intensities[c] - A_current) / (median_Js[c] - A_current)) / median_dist;
            beta_current = 0.014f;      // the geometric mean of the lower and upper bounds of beta (0.001 and 0.2)
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);


    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimesBeta[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimesBeta[intensity_mode].end())
            {
                mAllModesNumInlierTimesBeta[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimesBeta[intensity_mode][make_pair(current_pMP, pKF)] + 1;

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAutoGTA, 1, 1, 1>(
                            new FoggyResidualAutoGTA(double(vit->m_dist), double(vit->m_intensity), weight, GROUNDTRUTH_A));

            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &beta);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
//    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    problem.SetParameterLowerBound(&beta, 0, BETA_LOW);
    problem.SetParameterUpperBound(&beta, 0, BETA_HIGH);
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
    }
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimesBeta[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimesBeta[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {

                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAutoGTA, 1, 1, 1>(
                                new FoggyResidualAutoGTA(double(vit->m_dist), double(vit->m_intensity), 1.0, GROUNDTRUTH_A));

                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &beta);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
//        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
//        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
//        for (int i = 0; i < Js2.size(); ++i)
//        {
//            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        problem2.SetParameterLowerBound(&beta, 0, BETA_LOW);
        problem2.SetParameterUpperBound(&beta, 0, BETA_HIGH);
        for (int i = 0; i < vvFoggyObservation2.size(); ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
            auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

            float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
            }
            else
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
            }
        }
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = beta;
        estimates.end()[-1] = GROUNDTRUTH_A;

        // update estimates
        // Js
        for (size_t i = 0; i < estimates.size()-2; i++) {
            if (vIsMPActive[i])
            {
                vvFoggyObservation[i][0].m_pMP->UpdateJEstimatesBeta(mpCurrentKF, estimates[i], intensity_mode);
            }
        }
        // Beta and A
        mAllModesCurrentBetaEstimate[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
        mAllModesBetaEstimates[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentBetaEstimate[intensity_mode]));

        mAllModesStateBeta[intensity_mode] = OK;
    }
    else {
        if (mAllModesStateBeta[intensity_mode] == OK)
            mAllModesStateBeta[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesStateBeta[intensity_mode] == OK
        || mAllModesStateBeta[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+"-GroundtruthA"+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}

void FogEstimation::OptimiseA(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode, bool use_li_to_init)
{
    // for printing/writing to txt file
    size_t n_Js, n_obs, n_iters;       // number of MPs, number of observations, number of iterations

    vector<float> estimates;

    // initialise estimates
    estimates.reserve(vvFoggyObservation.size()+2);       // Js followed by beta then A

    const size_t n_MPs = vvFoggyObservation.size();       // number of MPs used for estimation

    float overall_max_dist = 0.0f;  // to store the maximum distance among all MPs
    float intensity_max_dist; // to store the intensity of the MP observed at the overall maximum distance

    // Js
    for (int i = 0; i < n_MPs; i++)
    {
        float J_current = vvFoggyObservation[i][0].m_pMP->GetJCurrentEstimateA(intensity_mode);
        if (J_current == -1.0f)
        {
            // this MP's J has never been estimated, use its foggy observation from the shortest distance to initialise
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
            estimates.push_back(vit->m_intensity);
        }
        else
            estimates.push_back(J_current);    // this MP's J has been estimated before, use the current estimate to initialise
    }

    // Beta and A
    float beta_current = GROUNDTRUTH_BETA;
    float A_current = mAllModesCurrentAEstimate[intensity_mode].m_atmos;
    bool has_estimated_beta_A = (A_current != 200.0f);
    if (!use_li_to_init && !has_estimated_beta_A)
    {
        for (int i = 0; i < n_MPs; i++)
        {
            // keep track of the intensity values at maximum distance which will be used to initialise A
            std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
            auto vit_max_dist = std::max_element(vFoggyObservation.begin(),vFoggyObservation.end());   // the iterator with max distance
            float current_max_dist = vit_max_dist->m_dist;
            if (current_max_dist > overall_max_dist && current_max_dist <= 50)
            {
                overall_max_dist = current_max_dist;
                intensity_max_dist = vit_max_dist->m_intensity;
            }
        }
    }
    if (!has_estimated_beta_A)      // if never been estimated before
    {
        if (use_li_to_init)
        {
            // use the Li's modified method to initialise
            A_current = mAllModesCurrentEstimateLiMedian[true].m_atmos;
        }
        else
        {
            // use the intensity observed at the largest distance to initialise A
            A_current = intensity_max_dist;
        }
    }
    estimates.push_back(beta_current);
    estimates.push_back(A_current);


    std::vector<double> Js(estimates.begin(), estimates.end()-2);
    double beta = estimates.end()[-2];
    double A = estimates.end()[-1];

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> vec_residual_block_id;
    vec_residual_block_id.reserve(20 * n_MPs);
    std::vector<float> vec_weight;
    vec_weight.reserve(20 * n_MPs);
    int count_MP = 0;
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        MapPoint* current_pMP = vFoggyObservation[0].m_pMP;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {

            // check the number of inlier times
            KeyFrame* pKF = vit->m_pKF;
            map<pair<MapPoint*, KeyFrame*>, int>::iterator mit = mAllModesNumInlierTimesA[intensity_mode].find(make_pair(current_pMP, pKF));
            if (mit == mAllModesNumInlierTimesA[intensity_mode].end())
            {
                mAllModesNumInlierTimesA[intensity_mode][make_pair(current_pMP, pKF)] = 0;   // this is a new observation so set to 0
            }
            float inlier_times = mAllModesNumInlierTimesA[intensity_mode][make_pair(current_pMP, pKF)];

            // calculate the overall weight
            float weight_J = std::fabs(Js[count_MP] - A);
//            float weight_J = std::fabs(Js[count_MP] - A) / 255.0f;
            float weight_gradient = MU / (MU + vit->m_gradient_squared);
            float weight_inlier_times = inlier_times + 1.0f;
//            float weight_inlier_times = 1.0f - 1.0f/(std::exp(inlier_times) + 1.0f);
            float weight = weight_J * weight_inlier_times;

            // Auto
            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<FoggyResidualAutoGTBeta, 1, 1, 1>(
                            new FoggyResidualAutoGTBeta(double(vit->m_dist), double(vit->m_intensity), weight, GROUNDTRUTH_BETA));

            ceres::ResidualBlockId residual_block_id = problem.AddResidualBlock(cost_function, new ceres::HuberLoss(sqrt(TH_HUBER2)), &Js[count_MP], &A);
            vec_residual_block_id.push_back(residual_block_id);

            vec_weight.push_back(weight);
        }
        count_MP++;
    }

    // set bounds
//    problem.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//    problem.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//    for (int i = 0; i < Js.size(); ++i)
//    {
//        problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
//        problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
//    }

    /* ---> set tighter bounds */
    std::vector<float> A_lower_bounds;
    std::vector<float> A_higher_bounds;
    for (int i = 0; i < n_MPs; ++i)
    {
        // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
        std::vector<FoggyObservation> vFoggyObservation = vvFoggyObservation[i];
        auto vit_min_dist = std::min_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with min distance
        float min_dist = vit_min_dist->m_dist;           // min distance
        float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
        auto vit_max_dist = std::max_element(vFoggyObservation.begin(), vFoggyObservation.end());    // return the iterator with max distance
        float max_dist = vit_max_dist->m_dist;           // max distance
        float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

        float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

        if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, I_min_dist);
            A_lower_bounds.push_back(I_max_dist);
        }
        else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
        {
            problem.SetParameterLowerBound(&Js[i], 0, I_min_dist);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
            A_higher_bounds.push_back(I_max_dist);
        }
        else
        {
            problem.SetParameterLowerBound(&Js[i], 0, J_LOW);
            problem.SetParameterUpperBound(&Js[i], 0, J_HIGH);
        }
    }
    float A_lower_bound;
    if (!A_lower_bounds.empty())
    {
        auto itMedianElement_lower_bounds = A_lower_bounds.begin() + A_lower_bounds.size()/2;
        std::nth_element(A_lower_bounds.begin(), itMedianElement_lower_bounds, A_lower_bounds.end());
        A_lower_bound = *itMedianElement_lower_bounds;

//        A_lower_bound = *std::min_element(A_lower_bounds.begin(), A_lower_bounds.end());
    }
    else
    {
        A_lower_bound = ATMOS_LOW;
    }
    float A_higher_bound = ATMOS_HIGH;
//    if (!A_higher_bounds.empty())
//    {
////        auto itMedianElement_higher_bounds = A_higher_bounds.begin() + A_higher_bounds.size()/2;
////        std::nth_element(A_higher_bounds.begin(), itMedianElement_higher_bounds, A_higher_bounds.end());
////        A_higher_bound = *itMedianElement_higher_bounds;
//
//        A_higher_bound = *std::max_element(A_higher_bounds.begin(), A_higher_bounds.end());
//    }
//    else
//    {
//        A_higher_bound = ATMOS_HIGH;
//    }
    problem.SetParameterLowerBound(&A, 0, A_lower_bound);
    problem.SetParameterUpperBound(&A, 0, A_higher_bound);
    /* ---< set tighter bounds */

    // optimise
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";

    // evaluate to get residuals
    ceres::Problem::EvaluateOptions evaluate_options;
    evaluate_options.residual_blocks = vec_residual_block_id;
    evaluate_options.apply_loss_function = false;       // the default is true
    std::vector<double> residuals;
    problem.Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);

    // check residuals and update inlier count
    count_MP = 0;
    int count_obs = 0;
    std::vector<std::vector<FoggyObservation>> vvFoggyObservation2;
    vvFoggyObservation2.reserve(n_MPs);
    std::vector<double> Js2;
    Js2.reserve(n_MPs);
    std::vector<bool> vIsMPActive(n_MPs, false);
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin();
         vvit != vvFoggyObservation.end(); vvit++) {
        std::vector<FoggyObservation> vFoggyObservation2;

        std::vector<FoggyObservation> vFoggyObservation = *vvit;
        for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation.begin();
             vit != vFoggyObservation.end(); vit++) {
            if (!vit->m_pMP || ((vit->m_pMP)->isBad()))
            {
                mAllModesNumInlierTimesA[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)] = 0;
            }
            else
            {
                if (residuals[count_obs]*residuals[count_obs]/vec_weight[count_obs] <= TH_HUBER2)
                {
                    mAllModesNumInlierTimesA[intensity_mode][make_pair(vit->m_pMP, vit->m_pKF)]++;
                    vFoggyObservation2.push_back(*vit);
                }
            }
            count_obs++;
        }

        if (vFoggyObservation2.size() >= MIN_NUM_OBS) {
            vvFoggyObservation2.push_back(vFoggyObservation2);
            Js2.push_back(Js[count_MP]);
            vIsMPActive[count_MP] = true;
        }

        count_MP++;
    }

    count_MP = 0;
    int num_inlier_obs = 0;
    int num_iterations = 0;
    if (vvFoggyObservation2.size() >= MIN_NUM_MPS)
    {
        // 2nd round
        ceres::Problem problem2;
        for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation2.begin();
             vvit != vvFoggyObservation2.end(); vvit++) {
            std::vector<FoggyObservation> vFoggyObservation2 = *vvit;
            for (std::vector<FoggyObservation>::iterator vit = vFoggyObservation2.begin();
                 vit != vFoggyObservation2.end(); vit++) {

                // Auto
                ceres::CostFunction* cost_function =
                        new ceres::AutoDiffCostFunction<FoggyResidualAutoGTBeta, 1, 1, 1>(
                                new FoggyResidualAutoGTBeta(double(vit->m_dist), double(vit->m_intensity), 1.0, GROUNDTRUTH_BETA));

                ceres::ResidualBlockId residual_block_id = problem2.AddResidualBlock(cost_function, nullptr, &Js2[count_MP], &A);

                num_inlier_obs++;
            }
            count_MP++;
        }

        // set bounds
//        problem2.SetParameterLowerBound(&A, 0, ATMOS_LOW);
//        problem2.SetParameterUpperBound(&A, 0, ATMOS_HIGH);
//        for (int i = 0; i < Js2.size(); ++i)
//        {
//            problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
//            problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
//        }

        /* ---> set tighter bounds */
        std::vector<float> A_lower_bounds2;
        std::vector<float> A_higher_bounds2;
        for (int i = 0; i < vvFoggyObservation2.size(); ++i)
        {
            // determine if J<A or J>A for that MP by checking the slope of the line connecting the two min and max distance points
            std::vector<FoggyObservation> vFoggyObservation2 = vvFoggyObservation2[i];
            auto vit_min_dist = std::min_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with min distance
            float min_dist = vit_min_dist->m_dist;           // min distance
            float I_min_dist = vit_min_dist->m_intensity;    // I at the min distance
            auto vit_max_dist = std::max_element(vFoggyObservation2.begin(), vFoggyObservation2.end());    // return the iterator with max distance
            float max_dist = vit_max_dist->m_dist;           // max distance
            float I_max_dist = vit_max_dist->m_intensity;    // I at the max distance

            float slope = (I_max_dist - I_min_dist) / (max_dist - min_dist);

            if (slope > TH_SLOPE)        // if the slope is strongly positive, then J<A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, I_min_dist);
                A_lower_bounds2.push_back(I_max_dist);
            }
            else if (slope < -TH_SLOPE)  // if the slope is strongly negative, then J>A
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, I_min_dist);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
                A_higher_bounds2.push_back(I_max_dist);
            }
            else
            {
                problem2.SetParameterLowerBound(&Js2[i], 0, J_LOW);
                problem2.SetParameterUpperBound(&Js2[i], 0, J_HIGH);
            }
        }
        float A_lower_bound2;
        if (!A_lower_bounds2.empty())
        {
            auto itMedianElement_lower_bounds2 = A_lower_bounds2.begin() + A_lower_bounds2.size() / 2;
            std::nth_element(A_lower_bounds2.begin(), itMedianElement_lower_bounds2, A_lower_bounds2.end());
            A_lower_bound2 = *itMedianElement_lower_bounds2;

//            A_lower_bound2 = *std::min_element(A_lower_bounds2.begin(), A_lower_bounds2.end());
        }
        else
        {
            A_lower_bound2 = A_lower_bound;
        }
        float A_higher_bound2 = ATMOS_HIGH;
//        if (!A_higher_bounds2.empty())
//        {
////            auto itMedianElement_higher_bounds2 = A_higher_bounds2.begin() + A_higher_bounds2.size() / 2;
////            std::nth_element(A_higher_bounds2.begin(), itMedianElement_higher_bounds2, A_higher_bounds2.end());
////            A_higher_bound2 = *itMedianElement_higher_bounds2;
//
//            A_higher_bound2 = *std::max_element(A_higher_bounds2.begin(), A_higher_bounds2.end());
//        }
//        else
//        {
//            A_higher_bound2 = A_higher_bound;
//        }
        problem2.SetParameterLowerBound(&A, 0, A_lower_bound2);
        problem2.SetParameterUpperBound(&A, 0, A_higher_bound2);
        /* ---< set tighter bounds */

        // optimise
        ceres::Solver::Options options2;
        options2.max_num_iterations = 50;
        options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//        options2.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
//        std::cout << summary2.BriefReport() << "\n";

        num_iterations = summary2.iterations.size();

        // retrieve estimated values
        int valid_count = 0;
        for (int i=0; i<estimates.size()-2; i++)
        {
            if (vIsMPActive[i])
            {
                estimates[i] = Js2[valid_count];
                valid_count++;
            }
        }
        estimates.end()[-2] = GROUNDTRUTH_BETA;
        estimates.end()[-1] = A;

        // update estimates
        // Js
        for (size_t i = 0; i < estimates.size()-2; i++) {
            if (vIsMPActive[i])
            {
                vvFoggyObservation[i][0].m_pMP->UpdateJEstimatesA(mpCurrentKF, estimates[i], intensity_mode);
            }
        }
        // Beta and A
        mAllModesCurrentAEstimate[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
        mAllModesAEstimates[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentAEstimate[intensity_mode]));

        mAllModesStateA[intensity_mode] = OK;
    }
    else {
        if (mAllModesStateA[intensity_mode] == OK)
            mAllModesStateA[intensity_mode] = NOT_UP_TO_DATE;
    }

    // for printing/writing to txt file
    n_Js = vvFoggyObservation2.size();
    n_obs = num_inlier_obs;
    n_iters = num_iterations;

    if (mAllModesStateA[intensity_mode] == OK
        || mAllModesStateA[intensity_mode] == NOT_UP_TO_DATE)
    {
        // write results to text file
        std::string file_name;
        switch (intensity_mode)
        {
            case RAW:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeraw-Optimiserceres_tight";
                break;
            case PROJ:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModeproj-Optimiserceres_tight";
                break;
            case REFD_REF_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_ref-Optimiserceres_tight";
                break;
            case REFD_MED_NONPRG:
                file_name = "Ours-Stage2-Weightproductthenuniform-IntensityModerefd_med-Optimiserceres_tight";
                break;
            default:
                break;
        }
        std::ofstream ofs;
        ofs.open("results/"+file_name+"-GroundtruthBeta"+".txt", ios::app);
        ofs << mpCurrentKF->mnFrameId
            << " " << mpCurrentKF->mnId
            << " " << estimates.end()[-2]
            << " " << estimates.end()[-1]
            << " " << n_Js
            << " " << n_obs
            << " " << n_iters
            << std::endl;
        ofs.close();
    }
}
#endif

void FogEstimation::UpdateEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        vvFoggyObservation[i][0].m_pMP->UpdateJEstimates(mpCurrentKF, estimates[i]);
    }
    // Beta and A
    mCurrentEstimate = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mEstimates.insert(std::make_pair(mpCurrentKF, mCurrentEstimate));

    mLastFogKFid = mpCurrentKF->mnId;
}

void FogEstimation::UpdateEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates2(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates2(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate2[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates2[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate2[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates3(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates3(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate3[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates3[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate3[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates4(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates4(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate4[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates4[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate4[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates5(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates5(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate5[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates5[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate5[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates6(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates6(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate6[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates6[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate6[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimates7(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimates7(mpCurrentKF, estimates[i], intensity_mode);
        }
    }
    // Beta and A
    mAllModesCurrentEstimate7[intensity_mode] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimates7[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimate7[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimatesAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode, int nChannel)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimatesAllChannels(mpCurrentKF, estimates[i], intensity_mode, nChannel);
        }
    }
    // Beta and A
    mAllModesCurrentEstimateAllChannels[intensity_mode][nChannel] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimatesAllChannels[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimateAllChannels[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}
void FogEstimation::UpdateEstimatesAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode, int nChannel)
{
    // Js
    for (size_t i = 0; i < estimates.size()-2; i++) {
        if (vIsMPActive[i])
        {
            vvFoggyObservation[i][0].m_pMP->UpdateJEstimatesAllChannelsWoGc(mpCurrentKF, estimates[i], intensity_mode, nChannel);
        }
    }
    // Beta and A
    mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode][nChannel] = BetaAtmos(estimates.end()[-2], estimates.end()[-1]);
    mAllModesEstimatesAllChannelsWoGc[intensity_mode].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimateAllChannelsWoGc[intensity_mode]));

//    mLastFogKFid = mpCurrentKF->mnId;
}

void FogEstimation::EstimateBetaLi(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, int channel_index, bool preserve_positive_beta_only, float atmos_estimate, float* beta_estimate, std::vector<float>* vec_valid_beta_estimate, long* total_pairs_count)
{
    /* estimate beta */
    (*vec_valid_beta_estimate).reserve(50 * vvFoggyObservation.size());

    // accumulate beta estimates
    // iterate each vFoggyObservation in vvFoggyObservation
    long total_pairs_count_ = 0;     // to record the total number of pairs
    for (std::vector<std::vector<FoggyObservation>>::const_iterator vvit = vvFoggyObservation.begin(); vvit != vvFoggyObservation.end(); vvit++)
    {
        std::vector<FoggyObservation> vFoggyObservation = *vvit;

        total_pairs_count_ += long(vFoggyObservation.size() * (vFoggyObservation.size()-1) / 2);     // N-choose-2

        // iterate each pair in vFoggyObservation (see https://ideone.com/86h2wn)
        std::vector<bool> bitset(vFoggyObservation.size() - 2, false);
        bitset.resize(vFoggyObservation.size(), true);
        do
        {
            float intensity_pair[2], distance_pair[2];
            int element_count = 0;
            for (std::size_t i = 0; i != vFoggyObservation.size(); ++i)
            {
                if (bitset[i])
                {
                    if (channel_index == -1)
                        intensity_pair[element_count] = vFoggyObservation[i].m_intensity;
                    else
                        intensity_pair[element_count] = vFoggyObservation[i].m_intensities[channel_index];

                    distance_pair[element_count] = vFoggyObservation[i].m_dist;

                    element_count++;
                }
            }

            // Check 1: neither (I-A) is zero
            float I_A_difference_0 = intensity_pair[0] - atmos_estimate;
            float I_A_difference_1 = intensity_pair[1] - atmos_estimate;
            if (I_A_difference_0 == 0 || I_A_difference_1 == 0)
                continue;

            // Check 2: the (I-A) ratio has to be positive
            float I_minus_A_ratio = I_A_difference_0 / I_A_difference_1;
            if (I_minus_A_ratio <= 0)
                continue;

            // Check 3: the depth difference has to be non-zero
            float depth_difference = distance_pair[0] - distance_pair[1];
            if (depth_difference == 0)
                continue;

            // Check 4: the inverse depth difference has to be large enough
            float inverse_depth_difference = 1 / depth_difference;
            if (inverse_depth_difference < 0.001f)      // this threshold value is chosen according to the paper
                continue;

            // compute beta estimate and add to the container
            float current_beta_estimate = -log(I_minus_A_ratio) * inverse_depth_difference;
            if (preserve_positive_beta_only && current_beta_estimate <= 0)
                continue;
            (*vec_valid_beta_estimate).push_back(current_beta_estimate);
        } while (std::next_permutation(bitset.begin(), bitset.end()));
    }

    // build beta histogram and select the highest bin
    // convert beta estimate container from vector to Mat
    cv::Mat mat_beta_estimate = cv::Mat(1, (*vec_valid_beta_estimate).size(), CV_32F, (*vec_valid_beta_estimate).data());
    // histogram settings
    int num_bins;
    float range[2];

    if (preserve_positive_beta_only) {
        range[0] = HIST_RANGE_LOW_POSITIVE;
    }
    else {
        range[0] = HIST_RANGE_LOW;
    }
    range[1] = HIST_RANGE_HIGH;

    num_bins = int((range[1] - range[0]) / HIST_BIN_WIDTH);

    const float first_bin_centre = range[0] + HIST_BIN_WIDTH/2;

    const int channels[] = {0};
    const int hist_size[] = {num_bins};
    const float* ranges[] = {range};
    cv::Mat hist;
    calcHist(&mat_beta_estimate, 1, channels, cv::Mat(),
             hist, 1, hist_size, ranges,
             true, false);

    // locate the highest bin and use its centre as the beta estimate
    cv::Point highest_bin_point;
    cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &highest_bin_point);
    int highest_bin_ind = highest_bin_point.y;

    *beta_estimate = first_bin_centre + highest_bin_ind * HIST_BIN_WIDTH;
    *total_pairs_count = total_pairs_count_;
}

void FogEstimation::EstimateFogLi(SingleImageDehazerHe::eAtmosMode eAtmosIntensityMode, const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, bool preserve_positive_beta_only, float atmos_gt)
{
    /* check if groundtruth A is given */
    bool gt_A_given;
    if (atmos_gt == -1) {
        gt_A_given = false;
    }
    else {
        gt_A_given = true;
    }

    /* estimate A */
    float atmos_estimate;
    if (!gt_A_given) {
        SingleImageDehazerHe *dehazer = new SingleImageDehazerHe(mpCurrentKF->mvImagePyramid[0], eAtmosIntensityMode);
        dehazer->CalcDarkChannel();
        dehazer->CalcAtmosLight();
        cv::Vec3f atmos_light = dehazer->GetAtmosLight();
        atmos_estimate = atmos_light[0];
        delete dehazer;
    }
    else
        atmos_estimate = atmos_gt;
    if (APPLY_GAMMA_CORRECTION)
        atmos_estimate = GammaExpansion(atmos_estimate);

    /* estimate beta */
    float beta_estimate;
    std::vector<float> vec_valid_beta_estimate;   // the container to store all valid beta estimates
    long total_pairs_count;     // total number of pairs from summation of N-choose-2
    EstimateBetaLi(vvFoggyObservation, -1, preserve_positive_beta_only, atmos_estimate, &beta_estimate,
                   &vec_valid_beta_estimate, &total_pairs_count);

    /* update estimates */
    if (!gt_A_given) {
        switch (eAtmosIntensityMode)
        {
            case SingleImageDehazerHe::MAX_INTENSITY:
                mAllModesCurrentEstimateLiMax[preserve_positive_beta_only] = BetaAtmos(beta_estimate, atmos_estimate);
                mAllModesEstimatesLiMax[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMax[preserve_positive_beta_only]));
                break;
            case SingleImageDehazerHe::MEDIAN_INTENSITY:
                mAllModesCurrentEstimateLiMedian[preserve_positive_beta_only] = BetaAtmos(beta_estimate, atmos_estimate);
                mAllModesEstimatesLiMedian[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMedian[preserve_positive_beta_only]));
                break;
            case SingleImageDehazerHe::MEAN_INTENSITY:
                mAllModesCurrentEstimateLiMean[preserve_positive_beta_only] = BetaAtmos(beta_estimate, atmos_estimate);
                mAllModesEstimatesLiMean[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMean[preserve_positive_beta_only]));
                break;
        }
    }
#if TEST_PARTIAL_GT
    else
    {
        switch (eAtmosIntensityMode)
        {
            case SingleImageDehazerHe::MAX_INTENSITY:
                mAllModesCurrentBetaEstimateLiMax[preserve_positive_beta_only] = beta_estimate;
                mAllModesBetaEstimatesLiMax[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMax[preserve_positive_beta_only]));
                break;
            case SingleImageDehazerHe::MEDIAN_INTENSITY:
                mAllModesCurrentBetaEstimateLiMedian[preserve_positive_beta_only] = beta_estimate;
                mAllModesBetaEstimatesLiMedian[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMedian[preserve_positive_beta_only]));
                break;
            case SingleImageDehazerHe::MEAN_INTENSITY:
                mAllModesCurrentBetaEstimateLiMean[preserve_positive_beta_only] = beta_estimate;
                mAllModesBetaEstimatesLiMean[preserve_positive_beta_only].insert(
                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMean[preserve_positive_beta_only]));
                break;
        }
    }
#endif

    /* write results to text file */
    std::string str_atmos_mode;
    switch (eAtmosIntensityMode)
    {
        case SingleImageDehazerHe::MAX_INTENSITY:
            str_atmos_mode = "-AModeMax";
            break;
        case SingleImageDehazerHe::MEDIAN_INTENSITY:
            str_atmos_mode = "-AModeMedian";
            break;
        case SingleImageDehazerHe::MEAN_INTENSITY:
            str_atmos_mode = "-AModeMean";
            break;
    }

    std::string str_preserve_positive_beta_only;
    if (preserve_positive_beta_only)
        str_preserve_positive_beta_only = "-PreservePositiveBetaTrue";
    else
        str_preserve_positive_beta_only = "-PreservePositiveBetaFalse";

    std::string str_gt_A;
    if (gt_A_given)
        str_gt_A = "-GroundtruthA";
    else
        str_gt_A = "";

    std::ofstream ofs;
    ofs.open("results/OthersLi" + str_atmos_mode + str_preserve_positive_beta_only + str_gt_A + ".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId
        << " " << beta_estimate
        << " " << atmos_estimate
        << " " << vec_valid_beta_estimate.size()      // number of valid pairs (i.e. valid beta estimates)
        << " " << total_pairs_count             // total number of pairs from summation of N-choose-2
        << std::endl;
    ofs.close();

#if SAVE_BETA_ESTIMATES
    // write valid beta estimates to text file
    ofs.open("results/Betas-OthersLi" + str_atmos_mode + str_preserve_positive_beta_only + str_gt_A + ".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId;
    for (float & vit : vec_valid_beta_estimate)
    {
        ofs << " " << vit;
    }
    ofs << std::endl;
    ofs.close();
#endif
}

void FogEstimation::EstimateFogLiAllChannels(SingleImageDehazerHe::eAtmosMode eAtmosIntensityMode, const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, bool preserve_positive_beta_only, cv::Vec4f atmos_gt)
{
    /* check if groundtruth A is given */
    bool gt_A_given;
    if (atmos_gt[0] == -1 && atmos_gt[1] == -1 && atmos_gt[2] == -1 && atmos_gt[3] == -1) {
        gt_A_given = false;
    }
    else {
        gt_A_given = true;
    }

    /* estimate A */
    cv::Vec4f atmos_estimate;
    if (!gt_A_given) {
        // colour image
        SingleImageDehazerHe *dehazer = new SingleImageDehazerHe(mpCurrentKF->mvImagePyramidColour[0], eAtmosIntensityMode);
        dehazer->CalcDarkChannel();
        dehazer->CalcAtmosLight();
        cv::Vec3f atmos_light = dehazer->GetAtmosLight();
        atmos_estimate[0] = atmos_light[0];
        atmos_estimate[1] = atmos_light[1];
        atmos_estimate[2] = atmos_light[2];
        delete dehazer;
        // grey image
        SingleImageDehazerHe *dehazer_grey = new SingleImageDehazerHe(mpCurrentKF->mvImagePyramid[0], eAtmosIntensityMode);
        dehazer_grey->CalcDarkChannel();
        dehazer_grey->CalcAtmosLight();
        cv::Vec3f atmos_light_grey = dehazer_grey->GetAtmosLight();
        atmos_estimate[3] = atmos_light_grey[0];
        delete dehazer_grey;
    }
    else
        atmos_estimate = atmos_gt;
    if (APPLY_GAMMA_CORRECTION)
        atmos_estimate = GammaExpansion(atmos_estimate);

    /* estimate beta */
    cv::Vec4f beta_estimate;
    std::array<std::vector<float>, 4> vec_valid_beta_estimate_all_channels;   // the container to store all valid beta estimates
    std::array<long, 4> total_pairs_count_all_channels;     // total number of pairs from summation of N-choose-2

    std::thread threadEstimateBetaLiChannel0(&FogEstimation::EstimateBetaLi, this, vvFoggyObservation, 0, preserve_positive_beta_only, atmos_estimate[0], &beta_estimate[0], &vec_valid_beta_estimate_all_channels[0], &total_pairs_count_all_channels[0]);
    std::thread threadEstimateBetaLiChannel1(&FogEstimation::EstimateBetaLi, this, vvFoggyObservation, 1, preserve_positive_beta_only, atmos_estimate[1], &beta_estimate[1], &vec_valid_beta_estimate_all_channels[1], &total_pairs_count_all_channels[1]);
    std::thread threadEstimateBetaLiChannel2(&FogEstimation::EstimateBetaLi, this, vvFoggyObservation, 2, preserve_positive_beta_only, atmos_estimate[2], &beta_estimate[2], &vec_valid_beta_estimate_all_channels[2], &total_pairs_count_all_channels[2]);
    std::thread threadEstimateBetaLiChannel3(&FogEstimation::EstimateBetaLi, this, vvFoggyObservation, 3, preserve_positive_beta_only, atmos_estimate[3], &beta_estimate[3], &vec_valid_beta_estimate_all_channels[3], &total_pairs_count_all_channels[3]);
    threadEstimateBetaLiChannel0.join();
    threadEstimateBetaLiChannel1.join();
    threadEstimateBetaLiChannel2.join();
    threadEstimateBetaLiChannel3.join();

    /* update estimates */
    if (!gt_A_given) {
        switch (eAtmosIntensityMode)
        {
            case SingleImageDehazerHe::MAX_INTENSITY:
                for (int c= 0; c < 4; c++)
                {
                    mAllModesCurrentEstimateLiMaxAllChannels[preserve_positive_beta_only][c] = BetaAtmos(beta_estimate[c], atmos_estimate[c]);
                    mAllModesEstimatesLiMaxAllChannels[preserve_positive_beta_only].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMaxAllChannels[preserve_positive_beta_only]));
                }
                break;
            case SingleImageDehazerHe::MEDIAN_INTENSITY:
                for (int c= 0; c < 4; c++)
                {
                    mAllModesCurrentEstimateLiMedianAllChannels[preserve_positive_beta_only][c] = BetaAtmos(beta_estimate[c], atmos_estimate[c]);
                    mAllModesEstimatesLiMedianAllChannels[preserve_positive_beta_only].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMedianAllChannels[preserve_positive_beta_only]));
                }
                break;
            case SingleImageDehazerHe::MEAN_INTENSITY:
                for (int c= 0; c < 4; c++)
                {
                    mAllModesCurrentEstimateLiMeanAllChannels[preserve_positive_beta_only][c] = BetaAtmos(beta_estimate[c], atmos_estimate[c]);
                    mAllModesEstimatesLiMeanAllChannels[preserve_positive_beta_only].insert(make_pair(mpCurrentKF, mAllModesCurrentEstimateLiMeanAllChannels[preserve_positive_beta_only]));
                }
                break;
        }
    }
#if TEST_PARTIAL_GT
//    else
//    {
//        switch (eAtmosIntensityMode)
//        {
//            case SingleImageDehazerHe::MAX_INTENSITY:
//                mAllModesCurrentBetaEstimateLiMaxAllChannels[preserve_positive_beta_only] = beta_estimate;
//                mAllModesBetaEstimatesLiMaxAllChannels[preserve_positive_beta_only].insert(
//                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMaxAllChannels[preserve_positive_beta_only]));
//                break;
//            case SingleImageDehazerHe::MEDIAN_INTENSITY:
//                mAllModesCurrentBetaEstimateLiMedianAllChannels[preserve_positive_beta_only] = beta_estimate;
//                mAllModesBetaEstimatesLiMedianAllChannels[preserve_positive_beta_only].insert(
//                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMedianAllChannels[preserve_positive_beta_only]));
//                break;
//            case SingleImageDehazerHe::MEAN_INTENSITY:
//                mAllModesCurrentBetaEstimateLiMeanAllChannels[preserve_positive_beta_only] = beta_estimate;
//                mAllModesBetaEstimatesLiMeanAllChannels[preserve_positive_beta_only].insert(
//                        make_pair(mpCurrentKF, mAllModesCurrentBetaEstimateLiMeanAllChannels[preserve_positive_beta_only]));
//                break;
//        }
//    }
#endif

    /* write results to text file */
    std::string str_atmos_mode;
    switch (eAtmosIntensityMode)
    {
        case SingleImageDehazerHe::MAX_INTENSITY:
            str_atmos_mode = "-AModeMax";
            break;
        case SingleImageDehazerHe::MEDIAN_INTENSITY:
            str_atmos_mode = "-AModeMedian";
            break;
        case SingleImageDehazerHe::MEAN_INTENSITY:
            str_atmos_mode = "-AModeMean";
            break;
    }

    std::string str_preserve_positive_beta_only;
    if (preserve_positive_beta_only)
        str_preserve_positive_beta_only = "-PreservePositiveBetaTrue";
    else
        str_preserve_positive_beta_only = "-PreservePositiveBetaFalse";

    std::string str_gt_A;
    if (gt_A_given)
        str_gt_A = "-GroundtruthA";
    else
        str_gt_A = "";

    std::ofstream ofs;
    ofs.open("results/OthersLi" + str_atmos_mode + str_preserve_positive_beta_only + str_gt_A + ".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId
        // first channel
        << " " << beta_estimate[0]
        << " " << atmos_estimate[0]
        << " " << vec_valid_beta_estimate_all_channels[0].size()      // number of valid pairs (i.e. valid beta estimates)
        << " " << total_pairs_count_all_channels[0]             // total number of pairs from summation of N-choose-2
        // second channel
        << " " << beta_estimate[1]
        << " " << atmos_estimate[1]
        << " " << vec_valid_beta_estimate_all_channels[1].size()      // number of valid pairs (i.e. valid beta estimates)
        << " " << total_pairs_count_all_channels[1]             // total number of pairs from summation of N-choose-2
        // third channel
        << " " << beta_estimate[2]
        << " " << atmos_estimate[2]
        << " " << vec_valid_beta_estimate_all_channels[2].size()      // number of valid pairs (i.e. valid beta estimates)
        << " " << total_pairs_count_all_channels[2]             // total number of pairs from summation of N-choose-2
        // grey channel
        << " " << beta_estimate[3]
        << " " << atmos_estimate[3]
        << " " << vec_valid_beta_estimate_all_channels[3].size()      // number of valid pairs (i.e. valid beta estimates)
        << " " << total_pairs_count_all_channels[3]             // total number of pairs from summation of N-choose-2
        << std::endl;
    ofs.close();

#if SAVE_BETA_ESTIMATES
    // write valid beta estimates to text file
    ofs.open("results/Betas-OthersLi" + str_atmos_mode + str_preserve_positive_beta_only + str_gt_A + ".txt", ios::app);
    ofs << mpCurrentKF->mnFrameId
        << " " << mpCurrentKF->mnId;
    for (auto & vvit : vec_vec_valid_beta_estimate)
    {
        for (auto & vit : vvit)
            ofs << " " << vit;
        ofs << std::endl;
    }
    ofs << std::endl;
    ofs.close();
#endif
}

void FogEstimation::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void FogEstimation::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpFogKeyFrameQueue.clear();
        mLastFogKFid=0;
        mbResetRequested=false;
    }
}

void FogEstimation::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool FogEstimation::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void FogEstimation::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool FogEstimation::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

float FogEstimation::CalcDistMP2KF(MapPoint* pMP, KeyFrame* pKF)
{
    return (float)cv::norm(pKF->GetCameraCenter() - pMP->GetWorldPos());
}

} //namespace ORB_SLAM