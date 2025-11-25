//
// Created by yd2007 on 27/02/23.
//

#include "MyLog.h"

namespace ORB_SLAM2
{
    class MyCompare
    {
    public:
        bool operator()(float dist1, float dist2)
        {
            return dist1 > dist2;
        }
    };

    void MyLog(list<MapPoint *> &localMapPoints, const string& beforeOrAfterLBA, bool isLevelSpecific, bool hasMeanDiff, bool isPatchNormalised, bool isProgressive)
    {
        assert((beforeOrAfterLBA == "Before" || beforeOrAfterLBA == "After")
            && "This log can only happen either 'Before' or 'After' Local BA");

        const string file_name = "MPs_"
                           + beforeOrAfterLBA + "LBA_"
                           + "LS" + to_string(isLevelSpecific) + "_"
                           + "MD" + to_string(hasMeanDiff) + "_"
                           + "PN" + to_string(isPatchNormalised) + "_"
                           + "Prg" + to_string(isProgressive)
                           + ".txt";

        ofstream ofs;
        ofs.open(file_name, ios::out);
        // write titles
        ofs
            << std::left << setw(SPACE_WIDTH) << "MPID "
            << std::left << setw(SPACE_WIDTH) << "KFID "
            << std::left << setw(SPACE_WIDTH) << "WarpKFID "
            << std::left << setw(SPACE_WIDTH) << "FID "
            << std::left << setw(SPACE_WIDTH) << "Distance "
            << std::left << setw(SPACE_WIDTH) << "RawBilinearI "
            << std::left << setw(SPACE_WIDTH) << "ProjBilinearI "
            << std::left << setw(SPACE_WIDTH) << "RefdBilinearI "
            << std::left << setw(SPACE_WIDTH) << "RawNearestI "
            << std::left << setw(SPACE_WIDTH) << "ProjNearestI "
            << std::left << setw(SPACE_WIDTH) << "RefdNearestI "
            << std::left << setw(SPACE_WIDTH) << "RawU "
            << std::left << setw(SPACE_WIDTH) << "RawV "
            << std::left << setw(SPACE_WIDTH) << "ProjU "
            << std::left << setw(SPACE_WIDTH) << "ProjV "
            << std::left << setw(SPACE_WIDTH) << "RefdU "
            << std::left << setw(SPACE_WIDTH) << "RefdV "
            << std::left << setw(SPACE_WIDTH) << "Convergence "
            << std::left << setw(SPACE_WIDTH) << "QurBestLevel "
            << std::left << setw(SPACE_WIDTH) << "KPOctave "
            << std::left << setw(SPACE_WIDTH) << "KPAngle "
            << std::left << setw(SPACE_WIDTH) << "KPSize "
            << std::left << setw(SPACE_WIDTH) << "KPResponse "
            << std::left << setw(SPACE_WIDTH) << "Arq00 "
            << std::left << setw(SPACE_WIDTH) << "Arq01 "
            << std::left << setw(SPACE_WIDTH) << "Arq10 "
            << std::left << setw(SPACE_WIDTH) << "Arq11 "
            << std::left << setw(SPACE_WIDTH) << "debug1 "
            << std::left << setw(SPACE_WIDTH) << "debug2 "
            << std::left << setw(SPACE_WIDTH) << "debug3 "
            << endl;
        ofs << endl;
        // loop through all local MPs
        for (list<MapPoint *>::iterator lit = localMapPoints.begin(), lend = localMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;

            if (pMP->isBad())       // TODO: also check if pMP is nullptr
            {
                continue;
            }

            map<KeyFrame *, size_t> observations = pMP->GetObservations();     // find all KFs that can see this MP (they are in either lLocalKeyFrames or lFixedCameras)

            // write MP info observed by the reference KF
            KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();                  // find its ref KF
            PatchAligner pa = PatchAligner(pMP, pRefKF, isLevelSpecific, PatchAligner::CENTRAL_DIFFERENCE, hasMeanDiff, isPatchNormalised);                   // create a patch aligner object
            int ind_ref = pa.GetIndexInRefKF();
            ofs
                << std::left << setw(SPACE_WIDTH) << pMP->mnId << " "
                << std::left << setw(SPACE_WIDTH) << pRefKF->mnId << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << pRefKF->mnFrameId << " "
                << std::left << setw(SPACE_WIDTH) << to_string((float)cv::norm(pRefKF->GetCameraCenter() - pMP->GetWorldPos())) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawIntensity()) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefProjIntensity()) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawIntensity()) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawIntensity(PatchAligner::NEAREST)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefProjIntensity(PatchAligner::NEAREST)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawIntensity(PatchAligner::NEAREST)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawLocation().at<float>(0)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefRawLocation().at<float>(1)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefProjLocation().at<float>(0)) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pa.GetRefProjLocation().at<float>(1)) << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << pa.GetRefOctave() << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pRefKF->mvKeys[ind_ref].angle) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pRefKF->mvKeys[ind_ref].size) << " "
                << std::left << setw(SPACE_WIDTH) << to_string(pRefKF->mvKeys[ind_ref].response) << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << std::left << setw(SPACE_WIDTH) << "NaN" << " "
                << endl;

            if (observations.size() == 1)
            {
                ofs << endl;
                continue;
            }

            // loop through all KFs that can see this MP
            if (!isProgressive)
            {
                for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                {
                    KeyFrame *pQurKF = mit->first;
                    if (pQurKF->mnId == pRefKF->mnId) {
                        continue;           // do not need to warp the ref KF to itself
                    }
                    pa.Align(pQurKF);

                    // write MP info observed by the query KF
                    int ind_qur = pa.GetIndexInQurKF();
                    ofs
                        << std::left << setw(SPACE_WIDTH) << pMP->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pQurKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pRefKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pQurKF->mnFrameId << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((float)cv::norm(pQurKF->GetCameraCenter() - pMP->GetWorldPos())) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(0))<< " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(1))<< " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << (int) pa.GetIsConverged() << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurBestSearchLevel() << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurOctave() << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((pQurKF->mvKeys[ind_qur].angle)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((pQurKF->mvKeys[ind_qur].size)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((pQurKF->mvKeys[ind_qur].response)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug1) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug2) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug3) << " "
                        << endl;
                }
                ofs << endl;
            }
            else    // progressive alignment
            {
                // sort observations by distance in ascending order
                vector<pair<float, KeyFrame*>> vOrderedDistKFPairs;
                vOrderedDistKFPairs.reserve(observations.size());
                for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
                {
                    KeyFrame *pKF = mit->first;
                    float dist = (float)cv::norm(pMP->GetWorldPos() - pKF->GetCameraCenter());
                    vOrderedDistKFPairs.push_back(make_pair(dist, pKF));
                }
                sort(vOrderedDistKFPairs.begin(),vOrderedDistKFPairs.end());      // in ascending order

                // locate where the RefKF is
                vector<pair<float, KeyFrame*>>::iterator itRefKF;
                for (vector<pair<float, KeyFrame*>>::iterator mit = vOrderedDistKFPairs.begin(), mend = vOrderedDistKFPairs.end(); mit != mend; mit++)
                {
                    if (mit->second->mnId == pRefKF->mnId)
                    {
                        itRefKF = mit;
                        break;
                    }
                }

                // align patches in frames further than the reference KF
                pa = PatchAligner(pMP, pRefKF, isLevelSpecific, PatchAligner::CENTRAL_DIFFERENCE, hasMeanDiff, isPatchNormalised);
                KeyFrame* pBestRefKF = pRefKF;
                for (vector<pair<float, KeyFrame*>>::iterator mit = itRefKF+1; mit != vOrderedDistKFPairs.end(); mit++)
                {
                    KeyFrame* pCurrentQurKF = mit->second;
                    pa.Align(pCurrentQurKF);

                    // write MP info observed by the query KF
                    int ind_qur = pa.GetIndexInQurKF();
                    ofs
                        << std::left << setw(SPACE_WIDTH) << pMP->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pCurrentQurKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pBestRefKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pCurrentQurKF->mnFrameId << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((float)cv::norm(pCurrentQurKF->GetCameraCenter() - pMP->GetWorldPos())) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << (int) (pa.GetIsConverged()) << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurBestSearchLevel() << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurOctave() << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].angle) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].size) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].response) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug1) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug2) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug3) << " "
                        << endl;

                    // if converged, the current query KF can be used as a reference KF to align other unaligned KFs
                    if (pa.GetIsConverged())
                    {
                        pBestRefKF = pCurrentQurKF;
                        vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                        pa = PatchAligner(pMP, pBestRefKF, isLevelSpecific, PatchAligner::CENTRAL_DIFFERENCE, hasMeanDiff, isPatchNormalised);
                        pa.SetRefImg(vAlignedQurImagePyramid);
                    }
                }

                // align patches in frames closer than the reference KF
                pa = PatchAligner(pMP, pRefKF, isLevelSpecific, PatchAligner::CENTRAL_DIFFERENCE, hasMeanDiff, isPatchNormalised);
                pBestRefKF = pRefKF;
                for (vector<pair<float, KeyFrame*>>::iterator mit = itRefKF; mit != vOrderedDistKFPairs.begin(); )
                {
                    mit--;

                    KeyFrame* pCurrentQurKF = mit->second;
                    pa.Align(pCurrentQurKF);

                    // write MP info observed by the query KF
                    int ind_qur = pa.GetIndexInQurKF();
                    ofs
                        << std::left << setw(SPACE_WIDTH) << pMP->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pCurrentQurKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pBestRefKF->mnId << " "
                        << std::left << setw(SPACE_WIDTH) << pCurrentQurKF->mnFrameId << " "
                        << std::left << setw(SPACE_WIDTH) << to_string((float)cv::norm(pCurrentQurKF->GetCameraCenter() - pMP->GetWorldPos())) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity()) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdIntensity(PatchAligner::NEAREST)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(0))<< " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRawLocation().at<float>(1))<< " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurProjLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetQurRefdLocation().at<float>(1)) << " "
                        << std::left << setw(SPACE_WIDTH) << (int) (pa.GetIsConverged()) << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurBestSearchLevel() << " "
                        << std::left << setw(SPACE_WIDTH) << pa.GetQurOctave() << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].angle) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].size) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pCurrentQurKF->mvKeys[ind_qur].response) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(0, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 0)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.GetAffineWarp().at<float>(1, 1)) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug1) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug2) << " "
                        << std::left << setw(SPACE_WIDTH) << to_string(pa.mf_debug3) << " "
                        << endl;

                    // if converged, the current query KF can be used as a reference KF to align other unaligned KFs
                    if (pa.GetIsConverged())
                    {
                        pBestRefKF = pCurrentQurKF;
                        vector<cv::Mat> vAlignedQurImagePyramid = pa.ComputeAlignedQurImagePyramid();
                        pa = PatchAligner(pMP, pBestRefKF, isLevelSpecific, PatchAligner::CENTRAL_DIFFERENCE, hasMeanDiff, isPatchNormalised);
                        pa.SetRefImg(vAlignedQurImagePyramid);
                    }
                }
                ofs << endl;
            }
        }
        ofs.close();
    }
}