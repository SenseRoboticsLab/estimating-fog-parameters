//
// Created by tedyiningding on 24/04/23.
//

#ifndef ORB_SLAM2_SINGLEIMAGEDEHAZERABSTRACT_H
#define ORB_SLAM2_SINGLEIMAGEDEHAZERABSTRACT_H

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

struct Pixel
{
    uchar m_intensity;
    int m_u, m_v;
    Pixel(uchar intensity, int u, int v);
};

bool operator> (const struct Pixel & a, const struct Pixel & b);

class SingleImageDehazerAbstract
{
protected:
    cv::Mat mHazyImg;
    int mnRows, mnCols, mnChs;
    cv::Vec3f mAtmosLight;
    cv::Mat mInitialTransmissionMap, mFinalTransmissionMap;
    cv::Mat mDehazedImg;
public:
    SingleImageDehazerAbstract(const cv::Mat &hazy_img);

    virtual void CalcAtmosLight() = 0;

//    virtual void CalcInitialTransmissionMap() = 0;
//    virtual void CalcFinalTransmissionMap() = 0;
//    virtual void CalcDehazedImg() = 0;

//    void Dehaze();
public:
    cv::Vec3f GetAtmosLight() const;
};

// https://ieeexplore.ieee.org/document/7780554 "Non-local Image Dehazing)
class BermanDehazer: public SingleImageDehazerAbstract
{

};

}

#endif //ORB_SLAM2_SINGLEIMAGEDEHAZERABSTRACT_H
