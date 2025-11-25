//
// Created by yd2007 on 09/02/23.
//

#ifndef ORB_SLAM2_PATCHALIGNER_H
#define ORB_SLAM2_PATCHALIGNER_H

#include <opencv2/opencv.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2
{
class KeyFrame;

class PatchAligner
{
public:
    enum eGradientMethod{
        SOBEL=0,        // https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d
        SCHARR=1,       // https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gaa13106761eedf14798f37aa2d60404c9
        CENTRAL_DIFFERENCE=2,   // (p[1] - p[-1]) / 2
    };

    enum eInterpMethod{
        BILINEAR=0,
        NEAREST=1,
    };

    float mf_debug1, mf_debug2,  mf_debug3;

public:
    PatchAligner(MapPoint* pMP,
                 KeyFrame* pRefKF,
                 bool isLevelSpecific = false,
                 eGradientMethod gradientMethod = PatchAligner::SOBEL,
                 bool hasMeanDiff = false,
                 bool isPatchNormalised = true);
    void AddQueryKF(KeyFrame* pQurKF);
    void Align(KeyFrame* pQurKF);

    int GetIndexInRefKF() const;
    int GetIndexInQurKF() const;
    cv::Mat GetAffineWarp() const;
    bool GetIsConverged() const;
    cv::Mat GetRefRawLocation() const;
    cv::Mat GetRefProjLocation() const;
    cv::Mat GetQurRawLocation() const;
    cv::Mat GetQurProjLocation() const;
    cv::Mat GetQurRefdLocation() const;
    int GetRefOctave() const;
    int GetQurOctave() const;
    int GetQurBestSearchLevel() const;
    float GetRefRawIntensity(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    float GetRefProjIntensity(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    float GetQurRawIntensity(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    float GetQurProjIntensity(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    float GetQurRefdIntensity(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    cv::Vec4f GetRefRawIntensities(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    cv::Vec4f GetRefProjIntensities(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    cv::Vec4f GetQurRawIntensities(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    cv::Vec4f GetQurProjIntensities(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    cv::Vec4f GetQurRefdIntensities(eInterpMethod interpMethod=PatchAligner::BILINEAR) const;
    float GetRefRawGradientSquared() const;
    float GetRefProjGradientSquared() const;
    float GetQurRawGradientSquared() const;
    float GetQurProjGradientSquared() const;
    float GetQurRefdGradientSquared() const;

    void SetRefImg(const vector<cv::Mat>& refImagePyramid);
    void UpdateRefRawLocation(cv::Mat raw_location_update);

    vector<cv::Mat> ComputeAlignedQurImagePyramid(bool is_matlab_consistent=false);

protected:
    MapPoint* mpMP;         // pointer to the MP
    KeyFrame* mpRefKF;      // pointer to the reference KF
    KeyFrame* mpQurKF;      // pointer to the query KF

    int mnIndexInRefKF, mnIndexInQurKF;

    // level related
    bool mbIsLevelSpecific;
    int mnRefOctave, mnQurOctave;
    float mfScaleFactor;
    std::vector<float> mvScaleFactors;
    int mnQurBestSearchLevel;

//    float mRefDist, mQurDist;

    eGradientMethod mGradientMethod;
    bool mbHasMeanDiff;
    bool mbIsPatchNormalised;

    // camera parameters
    cv::Mat mK;
    float fx, fy, cx, cy;

    // the reference image and the query image;
    cv::Mat mRefImg, mQurImg;
    cv::Mat mRefMSCNImg, mQurMSCNImg;   // mean subtracted, contrast normalised

    bool mbIsInputImageColour;
    cv::Mat mRefImgColour, mQurImgColour;

    // pixel locations in the reference KF
    cv::Mat mRefRawUV;  // the corresponding keypoint raw position in the reference KF
    cv::Mat mRefProjUV; // projection of the MP onto the reference KF

    // 2-by-2 affine transform matrices to warp between the query KF and the reference KF
    cv::Mat mAqr, mArq;

    cv::Mat mWarpedRefPatchWithBorder; // the warped image patch with boarder in the Ref KF (10-by-10)
    cv::Mat mWarpedRefPatch;            // the warped image patch in the Ref KF (8-by-8)

    bool mbIsConverged;                 // whether the inverse compositional aligned has converged

    // pixel locations in the query KF
    cv::Mat mQurRawUV;      // the corresponding keypoint raw position in the query KF
    cv::Mat mQurProjUV;      // projection of the MP onto the query KF
    cv::Mat mQurRefdUV;  // optimal pixel location in the image domain of the query KF

protected:
    void ComputeAffineWarpMatrix();     // compute the 2-by-2 affine transform matrices
    void ComputeAffineWarpMatrixOld();     // compute the 2-by-2 affine transform matrices
    void GetBestQurSearchLevel();          // compute the best search level in the query frame
    void WarpAffine();                  // warp an image patch from the query KF to the reference KF
    void ComputeOptimalLocation();      // compute the optimal pixel location in the query KF using the inverse compositional algorithm
    void ComputeOptimalLocationWithMeanDiff();

    static cv::Mat project(cv::Mat p3Dc, cv::Mat K);
    static cv::Mat unproject(cv::Mat p2D, float z, cv::Mat K);
    static float interp(const cv::Mat& img, float u, float v, eInterpMethod method);
    static float interp(const cv::Mat& img, const cv::Mat& uv, eInterpMethod method);
    static float compute_pixel_gradient_squared(const cv::Mat& img, const cv::Mat& uv);      // compute the pixel gradient squared using the central difference
    static void compute_patch_gradient(const cv::Mat& img_patch, cv::Mat& Gu, cv::Mat& Gv, eGradientMethod method);
    static cv::Mat compute_MSCN_img(const cv::Mat& src_img);
    static float interp_float(const cv::Mat& img, float u, float v, eInterpMethod method);
    static float interp_float(const cv::Mat& img, const cv::Mat& uv, eInterpMethod method);

protected:
    static const int WARP_PATCH_SIZE;
    static const int ALIGN_PATCH_SIZE;
    static const int ALIGN_PATCH_AREA;
    static const float NORMALSATION_COEFF;
    static const int MAX_ITER;              // max number of iterations
    static const float MIN_UPDATE_SQUARED;  // termination condition
};

}// namespace ORB_SLAM

#endif //ORB_SLAM2_PATCHALIGNER_H