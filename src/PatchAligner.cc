//
// Created by yd2007 on 10/02/23.
//
#include "PatchAligner.h"

namespace ORB_SLAM2
{

const int PatchAligner::WARP_PATCH_SIZE = 10;
const int PatchAligner::ALIGN_PATCH_SIZE = 8;
const int PatchAligner::ALIGN_PATCH_AREA = 64;
const float PatchAligner::NORMALSATION_COEFF = 255.0f;
const int PatchAligner::MAX_ITER = 20;
const float PatchAligner::MIN_UPDATE_SQUARED = 0.03f * 0.03f;

PatchAligner::PatchAligner(MapPoint* pMP, KeyFrame *pRefKF, bool isLevelSpecific, eGradientMethod gradientMethod, bool hasMeanDiff, bool isPatchNormalised):
        mpMP(pMP),
        mpRefKF(pRefKF),
        mnIndexInRefKF(mpMP->GetIndexInKeyFrame(mpRefKF)),
        mbIsLevelSpecific(isLevelSpecific),
        mnRefOctave(mpRefKF->mvKeysUn[mnIndexInRefKF].octave),
        mGradientMethod(gradientMethod),
        mbHasMeanDiff(hasMeanDiff),
        mbIsPatchNormalised(isPatchNormalised),
        mK(mpRefKF->mK.clone()),
        fx(mpRefKF->fx),
        fy(mpRefKF->fy),
        cx(mpRefKF->cx),
        cy(mpRefKF->cy),
        mbIsInputImageColour(mpRefKF->mbIsInputImageColour)
{
    if (mbIsLevelSpecific)
    {
        mfScaleFactor = mpRefKF->mfScaleFactor;
        mvScaleFactors = mpRefKF->mvScaleFactors;
        mRefImg = mpRefKF->mvImagePyramid[mnRefOctave].clone();
        if (mbIsInputImageColour)
            mRefImgColour = mpRefKF->mvImagePyramidColour[mnRefOctave].clone();
    }
    else
    {
        mfScaleFactor = 1.0f;
        mvScaleFactors.assign(mpRefKF->mnScaleLevels, 1.0f);
        mRefImg = mpRefKF->mvImagePyramid[0].clone();
        if (mbIsInputImageColour)
            mRefImgColour = mpRefKF->mvImagePyramidColour[0].clone();
    }

    // record the keypoint position in the reference frame
    mRefRawUV = cv::Mat(mpRefKF->mvKeysUn[mnIndexInRefKF].pt);

    // project MP onto reference KF's image plane
    // way 1: project the MP onto the reference KF to get the two points
    cv::Mat Trw = mpRefKF->GetPose();
    cv::Mat p3Dr = Trw.rowRange(0,3).colRange(0,3)*mpMP->GetWorldPos() + Trw.rowRange(0,3).col(3);
    mRefProjUV = project(p3Dr, mK);
    // way 2: unproject the keypoint to a 3D point, then reproject
//    cv::Mat p3Dr_2 = mpRefKF->UnprojectStereo(mnIndexInRefKF);  // TODO: need to check if the mvDepth at mnIndexInRefKF is -1
//    mRefProjUV = project(p3Dr_2, mK);

    // TODO: if mRefProjUV is out of range,

    // compute the MSCN image
//    mRefMSCNImg = compute_MSCN_img(mRefImg);
}

void PatchAligner::AddQueryKF(KeyFrame* pQurKF)
{
    mpQurKF = pQurKF;
    mnIndexInQurKF = mpMP->GetIndexInKeyFrame(mpQurKF);
    mnQurOctave = mpQurKF->mvKeysUn[mnIndexInQurKF].octave;
    assert(fx == mpQurKF->fx
           && fy == mpQurKF->fy
           && cx == mpQurKF->cx
           && cy == mpQurKF->cy
           && "The query frame and the reference frame do not have the same camera intrinsic parameters!");

    // record the keypoint position in the query frame
    mQurRawUV = cv::Mat(mpQurKF->mvKeysUn[mnIndexInQurKF].pt);

    // project MP onto query KF's image plane
    cv::Mat Tqw = mpQurKF->GetPose();
    cv::Mat p3Dq = Tqw.rowRange(0,3).colRange(0,3)*mpMP->GetWorldPos() + Tqw.rowRange(0,3).col(3);
    mQurProjUV = project(p3Dq, mK);

    // set best search level to 0; this will be updated in GetBestQurSearchLevel() later if alignment is needed
    mnQurBestSearchLevel = 0;
    // set the query image to the image at level 0; this will be updated in GetBestQurSearchLevel() later if alignment is needed
    mQurImg = mpQurKF->mvImagePyramid[0].clone();
    if (mbIsInputImageColour)
        mQurImgColour = mpQurKF->mvImagePyramidColour[0].clone();
}

void PatchAligner::Align(KeyFrame *pQurKF)
{
    AddQueryKF(pQurKF);

    // align
    ComputeAffineWarpMatrix();
    GetBestQurSearchLevel();
    WarpAffine();
    if (mbHasMeanDiff)
        ComputeOptimalLocationWithMeanDiff();
    else
        ComputeOptimalLocation();
}

void PatchAligner::ComputeAffineWarpMatrixOld()
{
    // in reference KF, unproject (u+1, v) and (u, v+1) to 3D point at depth z
    cv::Mat Trw = mpRefKF->GetPose();
    cv::Mat p3Dr = Trw.rowRange(0,3).colRange(0,3)*mpMP->GetWorldPos() + Trw.rowRange(0,3).col(3);
    cv::Mat uRefBasis = (cv::Mat_<float>(2,1) << 1, 0);
    cv::Mat vRefBasis = (cv::Mat_<float>(2,1) << 0, 1);
    cv::Mat p2Dr_uInc = mRefProjUV + mvScaleFactors[mnRefOctave] * uRefBasis;
    cv::Mat p2Dr_vInc = mRefProjUV + mvScaleFactors[mnRefOctave] * vRefBasis;
    float z = p3Dr.at<float>(2);
    cv::Mat p3Dr_uInc = unproject(p2Dr_uInc, z,mK);
    cv::Mat p3Dr_vInc = unproject(p2Dr_vInc, z,mK);

    // transform them to world coordinate
    cv::Mat Twr = mpRefKF->GetPoseInverse();
    cv::Mat p3Dw_uInc = Twr.rowRange(0,3).colRange(0,3)*p3Dr_uInc + Twr.rowRange(0,3).col(3);
    cv::Mat p3Dw_vInc = Twr.rowRange(0,3).colRange(0,3)*p3Dr_vInc + Twr.rowRange(0,3).col(3);

    // project MP onto query KF's image plane
    // way 1: project the MP onto the query KF to get the two points
    cv::Mat Tqw = mpQurKF->GetPose();
    cv::Mat p3Dq = Tqw.rowRange(0,3).colRange(0,3)*mpMP->GetWorldPos() + Tqw.rowRange(0,3).col(3);
    mQurProjUV = project(p3Dq, mK);
    // way 2: unproject the keypoint to a 3D point, then reproject
//    cv::Mat p3Dq_2 = mpQurKF->UnprojectStereo(mnIndexInQurKF);
//    mQurProjUV = project(p3Dq_2, mK);

    // project p3Dw_uInc and p3Dw_vInc onto query KF's image plane
    cv::Mat p3Dq_uInc = Tqw.rowRange(0,3).colRange(0,3)*p3Dw_uInc + Tqw.rowRange(0,3).col(3);
    cv::Mat p2Dq_uInc = project(p3Dq_uInc, mK);
    cv::Mat p3Dq_vInc = Tqw.rowRange(0,3).colRange(0,3)*p3Dw_vInc + Tqw.rowRange(0,3).col(3);
    cv::Mat p2Dq_vInc = project(p3Dq_vInc, mK);

    // form the affine warp matrix
    cv::Mat uBasis = p2Dq_uInc - mQurProjUV;
    cv::Mat vBasis = p2Dq_vInc - mQurProjUV;
    mAqr = cv::Mat(2, 2, CV_32F);
    uBasis.copyTo(mAqr.col(0));
    vBasis.copyTo(mAqr.col(1));

    mArq = mAqr.inv();
}

void PatchAligner::ComputeAffineWarpMatrix()
{
    // check if the KP in the reference KF has depth
    float depth = mpRefKF->mvDepth[mnIndexInRefKF];
    if (depth < 0)
    {
        cv::Mat Trw = mpRefKF->GetPose();
        cv::Mat p3Dr_from_w = Trw.rowRange(0,3).colRange(0,3)*mpMP->GetWorldPos() + Trw.rowRange(0,3).col(3);
        depth = p3Dr_from_w.at<float>(2);
    }
    cv::Mat p3Dr = unproject(mRefRawUV, depth,mK);;

    // in reference KF, unproject (u+1, v) and (u, v+1) to 3D point at depth z
    cv::Mat uRefBasis = (cv::Mat_<float>(2,1) << 1, 0);
    cv::Mat vRefBasis = (cv::Mat_<float>(2,1) << 0, 1);
    cv::Mat p2Dr_uInc = mRefRawUV + mvScaleFactors[mnRefOctave] * uRefBasis;
    cv::Mat p2Dr_vInc = mRefRawUV + mvScaleFactors[mnRefOctave] * vRefBasis;
    cv::Mat p3Dr_uInc = unproject(p2Dr_uInc, depth,mK);
    cv::Mat p3Dr_vInc = unproject(p2Dr_vInc, depth,mK);

    // transform them to world coordinate
    cv::Mat Twr = mpRefKF->GetPoseInverse();
    cv::Mat p3Dw = Twr.rowRange(0,3).colRange(0,3)*p3Dr + Twr.rowRange(0,3).col(3);
    cv::Mat p3Dw_uInc = Twr.rowRange(0,3).colRange(0,3)*p3Dr_uInc + Twr.rowRange(0,3).col(3);
    cv::Mat p3Dw_vInc = Twr.rowRange(0,3).colRange(0,3)*p3Dr_vInc + Twr.rowRange(0,3).col(3);

    // transform them to query coordinate then project
    cv::Mat Tqw = mpQurKF->GetPose();
    cv::Mat p3Dq = Tqw.rowRange(0,3).colRange(0,3)*p3Dw + Tqw.rowRange(0,3).col(3);
    cv::Mat p3Dq_uInc = Tqw.rowRange(0,3).colRange(0,3)*p3Dw_uInc + Tqw.rowRange(0,3).col(3);
    cv::Mat p3Dq_vInc = Tqw.rowRange(0,3).colRange(0,3)*p3Dw_vInc + Tqw.rowRange(0,3).col(3);
    cv::Mat p2Dq = project(p3Dq, mK);
    cv::Mat p2Dq_uInc = project(p3Dq_uInc, mK);
    cv::Mat p2Dq_vInc = project(p3Dq_vInc, mK);

    // form the affine warp matrix
    cv::Mat uBasis = p2Dq_uInc - p2Dq;
    cv::Mat vBasis = p2Dq_vInc - p2Dq;
    mAqr = cv::Mat(2, 2, CV_32F);
    uBasis.copyTo(mAqr.col(0));
    vBasis.copyTo(mAqr.col(1));

    mArq = mAqr.inv();
}

void PatchAligner::GetBestQurSearchLevel()
{
    if (mbIsLevelSpecific)
    {
        mnQurBestSearchLevel = 0;
        float scale_factor_squared = mfScaleFactor * mfScaleFactor;
        double D = cv::determinant(mAqr);
        while(D > 1.32f && mnQurBestSearchLevel < mvScaleFactors.size() - 1)
        {
            mnQurBestSearchLevel += 1;
            D /= scale_factor_squared;
        }
        mQurImg = mpQurKF->mvImagePyramid[mnQurBestSearchLevel].clone();
        if (mbIsInputImageColour)
            mQurImgColour = mpQurKF->mvImagePyramidColour[mnQurBestSearchLevel].clone();
    }
    else
    {
        mnQurBestSearchLevel = 0;
        mQurImg = mpQurKF->mvImagePyramid[0].clone();
        if (mbIsInputImageColour)
            mQurImgColour = mpQurKF->mvImagePyramidColour[0].clone();
    }

    // compute the MSCN image
//    mQurMSCNImg = compute_MSCN_img(mQurImg);
}

void PatchAligner::WarpAffine()
{
    mWarpedRefPatchWithBorder = cv::Mat(WARP_PATCH_SIZE, WARP_PATCH_SIZE, CV_32F);
    float* ptr_warped_ref_patch = mWarpedRefPatchWithBorder.ptr<float>(0);

    // generate the warped image with border
    const float halfSpan = ((float)WARP_PATCH_SIZE-1) / 2.0f;   // the half distance between the two pixels at the two ends of a side
    for (float v=-halfSpan; v<(float)WARP_PATCH_SIZE-halfSpan; ++v)
    {
        for (float u=-halfSpan; u<(float)WARP_PATCH_SIZE-halfSpan; ++u, ++ptr_warped_ref_patch)
        {
            cv::Mat delta_uv = (cv::Mat_<float>(2,1) << u, v);
            delta_uv *= mvScaleFactors[mnQurBestSearchLevel];
            cv::Mat warped_uv = mRefRawUV/mvScaleFactors[mnRefOctave] + mArq*delta_uv;
//            float warped_u = warped_uv.at<float>(0);
//            float warped_v = warped_uv.at<float>(1);
//            if (warped_u<0 || warped_v<0 || warped_u>mRefImg.cols-1 || warped_v>mRefImg.rows-1)
//                // out of boundary, set to zero
//                *ptr_warped_ref_patch = 0.0f;
//            else
//                // interpolate
//                *ptr_warped_ref_patch = interp(mRefImg, warped_u, warped_v, BILINEAR);
            *ptr_warped_ref_patch = interp(mRefImg, warped_uv, BILINEAR);   // will return -1 if out of boundary
//            *ptr_warped_ref_patch = interp_float(mRefMSCNImg, warped_uv, BILINEAR);   // will return -1 if out of boundary
        }
    }

    // generate the warped image
    int nrows = mWarpedRefPatchWithBorder.rows;
    int ncols = mWarpedRefPatchWithBorder.cols;
    mWarpedRefPatch = mWarpedRefPatchWithBorder.rowRange(1,nrows-1).colRange(1,ncols-1).clone();

    assert(mWarpedRefPatchWithBorder.rows == WARP_PATCH_SIZE
        && mWarpedRefPatchWithBorder.cols == WARP_PATCH_SIZE
        && "The warped patch with border has an unexpected size!");

    assert(mWarpedRefPatch.rows == ALIGN_PATCH_SIZE
        && mWarpedRefPatch.cols == ALIGN_PATCH_SIZE
        && "The warped patch (i.e. the patch to align) has an unexpected size!");
}

// the inverse compositional algorithm (Figure 4 in the paper "Lucas-Kanade 20 Years on")
// https://link.springer.com/article/10.1023/B:VISI.0000011205.11775.fd
void PatchAligner::ComputeOptimalLocation()
{
//    mf_debug1 = (float)cv::mean(mWarpedRefPatch)[0];

    /* variables to normalise patch */
    float ref_patch_mean = 0.0f, ref_patch_std = 1.0f;
    float qur_patch_resampled_mean = 0.0f, qur_patch_resampled_std = 1.0f;

    /* normalise the warped reference patch */
    if (mbIsPatchNormalised)
    {
        cv::Scalar ref_patch_mean_scalar, ref_patch_std_scalar;
        cv::meanStdDev(mWarpedRefPatch, ref_patch_mean_scalar, ref_patch_std_scalar);
        ref_patch_mean = (float)ref_patch_mean_scalar[0];
        ref_patch_std = (float)ref_patch_std_scalar[0];
    }
    mWarpedRefPatchWithBorder = (mWarpedRefPatchWithBorder - ref_patch_mean) / ref_patch_std;
    mWarpedRefPatch = (mWarpedRefPatch - ref_patch_mean) / ref_patch_std;

    /* compute image gradients of the warped reference patch */
    cv::Mat Gu, Gv;
    compute_patch_gradient(mWarpedRefPatchWithBorder, Gu, Gv, mGradientMethod);

    /* compute the steepest decent image (SDI) */
    // The Jacobian in this case is constant and is the 2-by-2 identity matrix,
    // so the steepest decent image (SDI) is the same as the gradient.
    cv::Mat SDI;     // to be of shape (ALIGN_PATCH_AREA, 2), each row consists of the horizontal and vertical gradients
    cv::hconcat(Gu.reshape(1,ALIGN_PATCH_AREA), Gv.reshape(1,ALIGN_PATCH_AREA), SDI);

    /* compute Hessian and its inverse */
    cv::Mat H = SDI.t() * SDI;      // eq. (36)
    cv::Mat Hinv = H.inv();

    const float halfSpan = ((float)ALIGN_PATCH_SIZE-1) / 2.0f;  // the half distance between the two pixels at the two ends of a side

    // variables to record the initial values
//    float res_sum_init;             // the initial residual sum (i.e. around mQurRawUV)

    // variables to record the latest values
    cv::Mat qur_patch_resampled;    // the latest query patch
//    float res_sum;                  // the latest residual sum (i.e. around current estimate)

    // initialise the optimal pixel location
    cv::Mat currentQurEstUV = mQurRawUV.clone() / mvScaleFactors[mnQurBestSearchLevel];
    // initialise the convergence flag to false
    mbIsConverged = false;

    // iteration starts
    for (int iter=0; iter<MAX_ITER; ++iter)
    {
        // resample the query frame image in a patch centred at currentQurEstUV
        cv::getRectSubPix(mQurImg,
                cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                cv::Point2f(currentQurEstUV),
                qur_patch_resampled,
                CV_32F);
//        cv::getRectSubPix(mQurMSCNImg,
//                          cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
//                          cv::Point2f(currentQurEstUV),
//                          qur_patch_resampled,
//                          CV_32F);

        // debug
//        if (iter == 0)
//        {
//            mf_debug2 = (float)cv::mean(qur_patch_resampled)[0];
//        }

        // normalise the resampled query patch
        if (mbIsPatchNormalised)
        {
            cv::Scalar qur_patch_resampled_mean_scalar, qur_patch_resampled_std_scalar;
            cv::meanStdDev(qur_patch_resampled, qur_patch_resampled_mean_scalar, qur_patch_resampled_std_scalar);
            qur_patch_resampled_mean = (float)qur_patch_resampled_mean_scalar[0];
            qur_patch_resampled_std = (float)qur_patch_resampled_std_scalar[0];
        }
        qur_patch_resampled = (qur_patch_resampled - qur_patch_resampled_mean) / qur_patch_resampled_std;

        cv::Mat res_patch = qur_patch_resampled - mWarpedRefPatch;
        cv::Mat res_patch_col_vec = res_patch.reshape(1,ALIGN_PATCH_AREA);
        cv::Mat SDI_res_product = SDI.t() * res_patch_col_vec;

        // update
        cv::Mat update = Hinv * SDI_res_product;         // eq. (35)
        currentQurEstUV -= update;

        // record the sum of residual value
//        if (iter == 0)
//        {
//            res_sum_init = (float)cv::sum(res_patch)[0];
//        }
//        else
//        {
//            res_sum = (float)cv::sum(res_patch)[0];
//        }

        /* checks */
        float currentU = currentQurEstUV.at<float>(0);
        float currentV = currentQurEstUV.at<float>(1);

        // check if a patch around currentQurEstUV is still completely within the image boundary
        if (currentU - halfSpan < 0
            || currentV - halfSpan < 0
            || currentU + halfSpan > mQurImg.cols - 1
            || currentV + halfSpan > mQurImg.rows - 1)
            break;

        // check if nan value
        if(isnan(currentU) || isnan(currentV))
            break;

        // check convergence
        if (pow(update.at<float>(0), 2) + pow(update.at<float>(1), 2) < MIN_UPDATE_SQUARED)
        {
            /*
            // FOR DEBUG
            cout << "Converged after iter " << iter << endl;
            cv::getRectSubPix(mpQurKF->mvImagePyramid[0],
                              cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                              cv::Point2f(mQurRefdUV),
                              qur_patch_resampled,
                              CV_32F);
            res_patch = qur_patch_resampled - mWarpedRefPatch;
            res_sum = cv::sum(res_patch)[0];
            cout << "final res_sum is " << res_sum << endl;
            */

//            cv::getRectSubPix(mQurImg,
//                              cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
//                              cv::Point2f(currentQurEstUV),
//                              qur_patch_resampled,
//                              CV_32F);
//            mf_debug3 = (float)cv::mean(qur_patch_resampled)[0];

            mbIsConverged=true;
            currentQurEstUV *= mvScaleFactors[mnQurBestSearchLevel];
            break;
        }
    }

    // set mQurRefdUV according to convergence
    if (mbIsConverged)
    {
        mQurRefdUV = currentQurEstUV.clone();   // set to currentQurEstUV if converged
    }
    else
    {
        mQurRefdUV = mQurRawUV.clone(); // set to currentQurEstUV if not converged
//        mf_debug3 = mf_debug2;
    }

    /*
    // FOR DEBUG
    cout << "raw: " << mQurRawUV.t() << endl;
    cout << "refined: " << mQurRefdUV.t() << endl;
    cout << "optimal shift is: " << cv::Mat(mQurRefdUV.() - mQurRawUV.t()) << endl;
    */
}

void PatchAligner::ComputeOptimalLocationWithMeanDiff()
{
    mf_debug1 = cv::mean(mWarpedRefPatch)[0];

    /* variables to normalise patch */
    float ref_patch_mean = 0.0f, ref_patch_std = 1.0f;
    float qur_patch_resampled_mean = 0.0f, qur_patch_resampled_std = 1.0f;

    /* normalise the warped reference patch */
    if (mbIsPatchNormalised)
    {
        cv::Scalar ref_patch_mean_scalar, ref_patch_std_scalar;
        cv::meanStdDev(mWarpedRefPatch, ref_patch_mean_scalar, ref_patch_std_scalar);
        ref_patch_mean = (float)ref_patch_mean_scalar[0];
        ref_patch_std = (float)ref_patch_std_scalar[0];
    }
    mWarpedRefPatchWithBorder = (mWarpedRefPatchWithBorder - ref_patch_mean) / ref_patch_std;
    mWarpedRefPatch = (mWarpedRefPatch - ref_patch_mean) / ref_patch_std;

    /* compute image gradients */
    cv::Mat Gu, Gv;
    compute_patch_gradient(mWarpedRefPatchWithBorder, Gu, Gv, mGradientMethod);

    /* compute the steepest decent image (SDI) */
    // The Jacobian in this case is constant and is the 2-by-2 identity matrix,
    // so the steepest decent image (SDI) is the same as the gradient.
    cv::Mat SDI = cv::Mat(ALIGN_PATCH_AREA, 3, CV_32F);     // each row consists of the horizontal and vertical gradients
    int pixelCount = 0;
    for (int r=0; r<ALIGN_PATCH_SIZE; r++)
    {
        for (int c=0; c<ALIGN_PATCH_SIZE; c++)
        {
            SDI.at<float>(pixelCount, 0) = Gu.at<float>(r,c);
            SDI.at<float>(pixelCount, 1) = Gv.at<float>(r,c);
            SDI.at<float>(pixelCount, 2) = 1.0f;
            pixelCount++;
        }

    }

    /* compute Hessian and its inverse */
    cv::Mat H = SDI.t() * SDI;      // eq. (36)
    cv::Mat Hinv = H.inv();

    float mean_diff = 0;

    const float halfSpan = ((float)ALIGN_PATCH_SIZE-1) / 2.0f;  // the half distance between the two pixels at the two ends of a side

    // variables to record the initial values
    float res_sum_init;             // the initial residual sum (i.e. around mQurRawUV)

    // variables to record the latest values
    cv::Mat qur_patch_resampled;    // the latest query patch
    float res_sum;                  // the latest residual sum (i.e. around mQurRefdUV)

    // initialise the optimal pixel location
    mQurRefdUV = mQurRawUV.clone() / mvScaleFactors[mnQurBestSearchLevel];
    // initialise the convergence flag to false
    mbIsConverged = false;

    // iteration starts
    for (int iter=0; iter<MAX_ITER; ++iter)
    {
        // resample the query frame image in a patch centred at mQurRefdUV
        cv::getRectSubPix(mQurImg,
                          cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                          cv::Point2f(mQurRefdUV),
                          qur_patch_resampled,
                          CV_32F);

        // debug
        if (iter == 0)
        {
            mf_debug2 = (float)cv::mean(qur_patch_resampled)[0];
        }

        // normalise the resampled query patch
        if (mbIsPatchNormalised)
        {
            cv::Scalar qur_patch_resampled_mean_scalar, qur_patch_resampled_std_scalar;
            cv::meanStdDev(qur_patch_resampled, qur_patch_resampled_mean_scalar, qur_patch_resampled_std_scalar);
            qur_patch_resampled_mean = (float)qur_patch_resampled_mean_scalar[0];
            qur_patch_resampled_std = (float)qur_patch_resampled_std_scalar[0];
        }
        qur_patch_resampled = (qur_patch_resampled - qur_patch_resampled_mean) / qur_patch_resampled_std;

        cv::Mat res_patch = qur_patch_resampled - mWarpedRefPatch + mean_diff;
        cv::Mat res_patch_col_vec = res_patch.reshape(1,ALIGN_PATCH_AREA);
        cv::Mat SDI_res_product = SDI.t() * res_patch_col_vec;

        // update
        cv::Mat update = Hinv * SDI_res_product;         // eq. (35)
        mQurRefdUV -= update.rowRange(0, 2);
        mean_diff -= update.at<float>(2);

        // record the sum of residual value
        if (iter == 0)
        {
            res_sum_init = cv::sum(res_patch)[0];
        }
        else
        {
            res_sum = cv::sum(res_patch)[0];
        }

        // check if a patch around mQurRefdUV is still completely within the image boundary
        if (mQurRefdUV.at<float>(0) - halfSpan < 0
            || mQurRefdUV.at<float>(1) - halfSpan < 0
            || mQurRefdUV.at<float>(0) + halfSpan > mQurImg.cols - 1
            || mQurRefdUV.at<float>(1) + halfSpan > mQurImg.rows - 1)
            break;

        // check convergence
        if (pow(update.at<float>(0), 2) + pow(update.at<float>(1), 2) < MIN_UPDATE_SQUARED)
        {
            /*
            // FOR DEBUG
            cout << "Converged after iter " << iter << endl;
            cv::getRectSubPix(mpQurKF->mvImagePyramid[0],
                              cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                              cv::Point2f(mQurRefdUV),
                              qur_patch_resampled,
                              CV_32F);
            res_patch = qur_patch_resampled - mWarpedRefPatch;
            res_sum = cv::sum(res_patch)[0];
            cout << "final res_sum is " << res_sum << endl;
            */

            cv::getRectSubPix(mQurImg,
                              cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                              cv::Point2f(mQurRefdUV),
                              qur_patch_resampled,
                              CV_32F);
            mf_debug3 = cv::mean(qur_patch_resampled)[0];

            mbIsConverged=true;
            mQurRefdUV *= mvScaleFactors[mnQurBestSearchLevel];
            break;
        }
    }

    // if not converged, check if the residual sum at the end is even greater than the initial value
    if (!mbIsConverged)
    {
        if (res_sum > res_sum_init)
        {
            mQurRefdUV = mQurRawUV.clone();   // the final estimate is not good, use the initial estimate instead
            mf_debug3 = mf_debug2;
        }
        else
        {
            cv::getRectSubPix(mQurImg,
                              cv::Size2d(ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE),
                              cv::Point2f(mQurRefdUV),
                              qur_patch_resampled,
                              CV_32F);
            mf_debug3 = cv::mean(qur_patch_resampled)[0];
        }
    }



    /*
    // FOR DEBUG
    cout << "raw: " << mQurRawUV.t() << endl;
    cout << "refined: " << mQurRefdUV.t() << endl;
    cout << "optimal shift is: " << cv::Mat(mQurRefdUV.() - mQurRawUV.t()) << endl;
    */
}

// compute the query KF's image pyramid after alignment
vector<cv::Mat> PatchAligner::ComputeAlignedQurImagePyramid(bool is_matlab_consistent)
{
    // initialise the pyramid
    vector<cv::Mat> outImagePyramid;
    outImagePyramid.reserve(mpQurKF->mvImagePyramid.size());
    for (size_t level=0; level<mpQurKF->mvImagePyramid.size(); level++)
    {
        outImagePyramid.push_back(mpQurKF->mvImagePyramid[level].clone());
    }

    // compute the transform
    cv::Mat uv_refined = mQurRefdUV.clone();
    cv::Mat uv_raw = mQurRawUV.clone();
    cv::Mat uv_shift = -(uv_refined - uv_raw);
    cv::Mat translationTransform = (cv::Mat_<float>(2,3) << 1, 0, 0, 0, 1, 0);    // 2x3 transformation matrix that represents a translation
    uv_shift.copyTo(translationTransform.col(2));

    // compute the aligned query image at level 0
    if (!is_matlab_consistent)
    {
        cv::warpAffine(outImagePyramid[0],
                       outImagePyramid[0],
                       translationTransform,
                       outImagePyramid[0].size());
    }
    else
    {
        cv::Mat temp_src, temp_dst;
        outImagePyramid[0].convertTo(temp_src, CV_32F);
        cv::warpAffine(temp_src,
                       temp_dst,
                       translationTransform,
                       outImagePyramid[0].size());
        cv::Mat temp(temp_dst.size(), CV_32F);
        for (int r=0; r<temp_dst.rows; r++)
        {
            for (int c=0; c<temp_dst.cols; c++)
            {
                temp.at<float>(r,c) = round(temp_dst.at<float>(r, c));
            }
        }
        temp.convertTo(outImagePyramid[0], CV_8U);
    }

    // compute the aligned query image at higher levels
    for (size_t level= 1; level < outImagePyramid.size(); level++)
    {
        if (!is_matlab_consistent)
        {
            cv::resize(outImagePyramid[level-1],
                       outImagePyramid[level],
                       outImagePyramid[level].size(),0,0,cv::INTER_LINEAR);
        }
        else
        {
            cv::Mat src_img, dst_img;
            outImagePyramid[level-1].convertTo(src_img, CV_32F);
            resize(src_img, dst_img, outImagePyramid[level].size(), 0, 0, cv::INTER_LINEAR);
            cv::Mat temp_mat(outImagePyramid[level].size(), CV_32F);
            for (int r=0; r<dst_img.rows; r++)
            {
                for (int c=0; c<dst_img.cols; c++)
                {
                    temp_mat.at<float>(r,c) = round(dst_img.at<float>(r, c));
                }
            }
            temp_mat.convertTo(outImagePyramid[level], CV_8U);
        }
    }

    return outImagePyramid;
}

int PatchAligner::GetIndexInRefKF() const
{
    return mnIndexInRefKF;
}

int PatchAligner::GetIndexInQurKF() const
{
    return mnIndexInQurKF;
}

cv::Mat PatchAligner::GetAffineWarp() const
{
    return mArq;
}

bool PatchAligner::GetIsConverged() const
{
    return mbIsConverged;
}

cv::Mat PatchAligner::GetRefRawLocation() const
{
    return mRefRawUV;
}

cv::Mat PatchAligner::GetRefProjLocation() const
{
    return mRefProjUV;
}

cv::Mat PatchAligner::GetQurRawLocation() const
{
    return mQurRawUV;
}

cv::Mat PatchAligner::GetQurProjLocation() const
{
    return mQurProjUV;
}

cv::Mat PatchAligner::GetQurRefdLocation() const
{
    return mQurRefdUV;
}

int PatchAligner::GetRefOctave() const
{
    return mnRefOctave;
}

int PatchAligner::GetQurOctave() const
{
    return mnQurOctave;
}

int PatchAligner::GetQurBestSearchLevel() const
{
    return mnQurBestSearchLevel;
}

float PatchAligner::GetRefRawIntensity(eInterpMethod interpMethod) const
{
    return interp(mRefImg, mRefRawUV / mvScaleFactors[mnRefOctave], interpMethod);
}

float PatchAligner::GetRefProjIntensity(eInterpMethod interpMethod) const
{
    return interp(mRefImg, mRefProjUV / mvScaleFactors[mnRefOctave], interpMethod);
}

float PatchAligner::GetQurRawIntensity(eInterpMethod interpMethod) const
{
    return interp(mQurImg, mQurRawUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
}

float PatchAligner::GetQurProjIntensity(eInterpMethod interpMethod) const
{
    return interp(mQurImg, mQurProjUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
}

float PatchAligner::GetQurRefdIntensity(eInterpMethod interpMethod) const
{
    return interp(mQurImg, mQurRefdUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
}

cv::Vec4f PatchAligner::GetRefRawIntensities(eInterpMethod interpMethod) const
{
    assert(mbIsInputImageColour);
    cv::Vec4f out;
    for (int i=0; i<3; i++)
    {
        cv::Mat current_channel;
        cv::extractChannel(mRefImgColour, current_channel, i);
        out[i] = interp(current_channel, mRefRawUV / mvScaleFactors[mnRefOctave], interpMethod);
    }
    out[3] = interp(mRefImg, mRefRawUV / mvScaleFactors[mnRefOctave], interpMethod);
    return out;
}

cv::Vec4f PatchAligner::GetRefProjIntensities(eInterpMethod interpMethod) const
{
    assert(mbIsInputImageColour);
    cv::Vec4f out;
    for (int i=0; i<3; i++)
    {
        cv::Mat current_channel;
        cv::extractChannel(mRefImgColour, current_channel, i);
        out[i] = interp(current_channel, mRefProjUV / mvScaleFactors[mnRefOctave], interpMethod);
    }
    out[3] = interp(mRefImg, mRefProjUV / mvScaleFactors[mnRefOctave], interpMethod);
    return out;
}

cv::Vec4f PatchAligner::GetQurRawIntensities(eInterpMethod interpMethod) const
{
    assert(mbIsInputImageColour);
    cv::Vec4f out;
    for (int i=0; i<3; i++)
    {
        cv::Mat current_channel;
        cv::extractChannel(mQurImgColour, current_channel, i);
        out[i] = interp(current_channel, mQurRawUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    }
    out[3] = interp(mQurImg, mQurRawUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    return out;
}

cv::Vec4f PatchAligner::GetQurProjIntensities(eInterpMethod interpMethod) const
{
    assert(mbIsInputImageColour);
    cv::Vec4f out;
    for (int i=0; i<3; i++)
    {
        cv::Mat current_channel;
        cv::extractChannel(mQurImgColour, current_channel, i);
        out[i] = interp(current_channel, mQurProjUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    }
    out[3] = interp(mQurImg, mQurProjUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    return out;
}

cv::Vec4f PatchAligner::GetQurRefdIntensities(eInterpMethod interpMethod) const
{
    assert(mbIsInputImageColour);
    cv::Vec4f out;
    for (int i=0; i<3; i++)
    {
        cv::Mat current_channel;
        cv::extractChannel(mQurImgColour, current_channel, i);
        out[i] = interp(current_channel, mQurRefdUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    }
    out[3] = interp(mQurImg, mQurRefdUV / mvScaleFactors[mnQurBestSearchLevel], interpMethod);
    return out;
}

float PatchAligner::GetRefRawGradientSquared() const
{
    return compute_pixel_gradient_squared(mRefImg, mRefRawUV / mvScaleFactors[mnRefOctave]);
}

float PatchAligner::GetRefProjGradientSquared() const
{
    return compute_pixel_gradient_squared(mRefImg, mRefProjUV / mvScaleFactors[mnRefOctave]);
}

float PatchAligner::GetQurRawGradientSquared() const
{
    return compute_pixel_gradient_squared(mQurImg, mQurRawUV / mvScaleFactors[mnQurBestSearchLevel]);
}

float PatchAligner::GetQurProjGradientSquared() const
{
    return compute_pixel_gradient_squared(mQurImg, mQurProjUV / mvScaleFactors[mnQurBestSearchLevel]);
}

float PatchAligner::GetQurRefdGradientSquared() const
{
    return compute_pixel_gradient_squared(mQurImg, mQurRefdUV / mvScaleFactors[mnQurBestSearchLevel]);
}


void PatchAligner::SetRefImg(const vector<cv::Mat>& refImagePyramid)
{
    if (mbIsLevelSpecific)
    {
        mRefImg = refImagePyramid[mnRefOctave].clone();
    }
    else
    {
        mRefImg = refImagePyramid[0].clone();
    }
}

void PatchAligner::UpdateRefRawLocation(cv::Mat raw_location_update)
{
    assert(raw_location_update.rows == 2
        && raw_location_update.cols == 1
        && "update has to be of size 2-by-1");

    mRefRawUV += raw_location_update;
}

// static methods
cv::Mat PatchAligner::project(cv::Mat p3Dc, cv::Mat K)
{
    float invz = 1.0f / p3Dc.at<float>(2);
    cv::Mat p2D = K * p3Dc * invz;
    return p2D.rowRange(0,2);
}

cv::Mat PatchAligner::unproject(cv::Mat p2D, float z, cv::Mat K)
{
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    const float u = p2D.at<float>(0,0);
    const float v = p2D.at<float>(1,0);

    float x = (u-cx)/fx*z;
    float y = (v-cy)/fy*z;

    return (cv::Mat_<float>(3,1) << x, y, z);
}

float PatchAligner::interp(const cv::Mat& img, float u, float v, eInterpMethod method)
{
    assert(img.type() == CV_8U
        && "input image has to be of type CV_8U");

    const int step = img.step.p[0];               // pointer increment per row

    switch (method)
    {
        case BILINEAR:
        {
            // the integer and decimal pixels
            int u_int = floor(u);
            int v_int = floor(v);
            float u_dec = u-u_int;
            float v_dec = v-v_int;

            // the weights at the four corners
            float wtl = (1.0f-u_dec) * (1.0f-v_dec);
            float wtr = u_dec * (1.0-v_dec);
            float wbl = (1.0f-u_dec) * v_dec;
            float wbr = 1 - wtl - wtr - wbl;    // should be equal to u_dec*v_dec

            uchar* p = img.data + v_int*step + u_int;     // pointer to the top left pixel
            return wtl*p[0] + wtr*p[1] + wbl*p[step] + wbr*p[step+1];
        }
        case NEAREST:
        {
            int u_nearest = round(u);
            int v_nearest = round(v);

            uchar* p = img.data + v_nearest*step + u_nearest;     // pointer to the nearest pixel
            return p[0];
        }
    }
}

float PatchAligner::interp(const cv::Mat& img, const cv::Mat& uv, eInterpMethod method)
{
    assert(uv.rows == 2
        && uv.cols == 1
        && "uv is expected to be a CV::Mat of size 2-by-1");
    cv::Mat uv_float;
    uv.convertTo(uv_float, CV_32F);
    float u = uv_float.at<float>(0);
    float v = uv_float.at<float>(1);
    if (u < 0
        || u > img.cols-1
        || v < 0
        || v > img.rows-1)
        return -1.0f;           // out of boundary! return a negative value
    else
        return interp(img, u, v, method);
}

float PatchAligner::compute_pixel_gradient_squared(const cv::Mat &img, const cv::Mat& uv)
{
    /* ---> compute the pixel values on top, bottom, left and right */
    float top = interp(img, uv + (cv::Mat_<float>(2,1) << 0, -1), BILINEAR);
    float bottom = interp(img, uv + (cv::Mat_<float>(2,1) << 0, 1), BILINEAR);
    float left = interp(img, uv + (cv::Mat_<float>(2,1) << -1, 0), BILINEAR);
    float right = interp(img, uv + (cv::Mat_<float>(2,1) << 1, 0), BILINEAR);
    /* ---< compute the pixel values on top, bottom, left and right */

    if (top < 0 || bottom < 0 || left < 0 || right < 0)
        return -1.0f;           // out of boundary! return a negative value
    else
        return 0.25f *((top-bottom)*(top-bottom) + (left-right)*(left-right));
}

void PatchAligner::compute_patch_gradient(const cv::Mat& img_patch, cv::Mat& Gu, cv::Mat& Gv, eGradientMethod method)
{
    // compute image gradients
    cv::Mat GuWithBorder, GvWithBorder;
    cv::Rect rect(1,1,ALIGN_PATCH_SIZE,ALIGN_PATCH_SIZE);
    // There are several methods to calculate image gradients by finite differences
    switch (method)
    {
        case SOBEL:
        {
            cv::Sobel(img_patch, GuWithBorder, CV_32F, 1, 0);
            cv::Sobel(img_patch, GvWithBorder, CV_32F, 0, 1);
            break;
        }
        case SCHARR:
        {
            cv::Scharr(img_patch, GuWithBorder, CV_32F, 1, 0);
            cv::Scharr(img_patch, GvWithBorder, CV_32F, 0, 1);
            break;
        }
        case CENTRAL_DIFFERENCE:
        {
            cv::Mat kernelU = (cv::Mat_<float>(1,3) << -0.5, 0, 0.5);
            cv::Mat kernelV = (cv::Mat_<float>(3,1) << -0.5, 0, 0.5);
            cv::filter2D(img_patch, GuWithBorder, CV_32F, kernelU);
            cv::filter2D(img_patch, GvWithBorder, CV_32F, kernelV);
            break;
        }
    }
    // discard the border
    GuWithBorder(rect).copyTo(Gu);
    GvWithBorder(rect).copyTo(Gv);
}

cv::Mat PatchAligner::compute_MSCN_img(const cv::Mat& src_img)
{
    const int gaussian_kernel_size = 7;
    const float gaussian_kernel_sigma = 7.0f / 6.0f;

    cv::Mat float_src_img;
    src_img.convertTo(float_src_img, CV_32F);

    cv::Mat mu_img;
    GaussianBlur(float_src_img, mu_img, cv::Size(gaussian_kernel_size, gaussian_kernel_size), gaussian_kernel_sigma, gaussian_kernel_sigma, cv::BORDER_REPLICATE);

    cv::Mat mu2_img;
    cv::multiply(mu_img, mu_img, mu2_img);

    cv::Mat float_src2_img;
    cv::multiply(float_src_img, float_src_img, float_src2_img);

    cv::Mat temp_img;
    GaussianBlur(float_src2_img, temp_img, cv::Size(gaussian_kernel_size, gaussian_kernel_size), gaussian_kernel_sigma, gaussian_kernel_sigma, cv::BORDER_REPLICATE);
    temp_img = cv::abs(temp_img - mu2_img);
    cv::Mat sigma_img;
    cv::sqrt(temp_img, sigma_img);

    cv::Mat numerator = float_src_img - mu_img;
    cv::Mat denominator = sigma_img + 1;
    cv::Mat MSCN_img;
    cv::divide(numerator, denominator, MSCN_img);

    return MSCN_img;
}

float PatchAligner::interp_float(const cv::Mat& img, float u, float v, eInterpMethod method)
{
    assert(img.type() == CV_32F
           && "input image has to be of type CV_32F");

//    const int step = img.step.p[0];               // pointer increment per row

    switch (method)
    {
        case BILINEAR:
        {
            // the integer and decimal pixels
            int u_int = floor(u);
            int v_int = floor(v);
            float u_dec = u-u_int;
            float v_dec = v-v_int;

            // the weights at the four corners
            float wtl = (1.0f-u_dec) * (1.0f-v_dec);
            float wtr = u_dec * (1.0-v_dec);
            float wbl = (1.0f-u_dec) * v_dec;
            float wbr = 1 - wtl - wtr - wbl;    // should be equal to u_dec*v_dec

//            uchar* p = img.data + v_int*step + u_int;     // pointer to the top left pixel
            const float* p = img.ptr<float>(v_int) + u_int;     // pointer to the top left pixel
            return wtl*p[0] + wtr*p[1] + wbl*p[img.cols] + wbr*p[img.cols+1];
        }
        case NEAREST:
        {
            int u_nearest = round(u);
            int v_nearest = round(v);

//            uchar* p = img.data + v_nearest*step + u_nearest;     // pointer to the nearest pixel
            const float* p = img.ptr<float>(v_nearest) + u_nearest;     // pointer to the top left pixel
            return p[0];
        }
    }
}

float PatchAligner::interp_float(const cv::Mat& img, const cv::Mat& uv, eInterpMethod method)
{
    assert(uv.rows == 2
           && uv.cols == 1
           && "uv is expected to be a CV::Mat of size 2-by-1");
    cv::Mat uv_float;
    uv.convertTo(uv_float, CV_32F);
    float u = uv_float.at<float>(0);
    float v = uv_float.at<float>(1);
    if (u < 0
        || u > img.cols-1
        || v < 0
        || v > img.rows-1)
        return -1.0f;           // out of boundary! return a negative value
    else
        return interp_float(img, u, v, method);
}

}