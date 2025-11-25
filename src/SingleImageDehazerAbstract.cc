//
// Created by tedyiningding on 24/04/23.
//

#include "SingleImageDehazerAbstract.h"

namespace ORB_SLAM2
{
Pixel::Pixel(uchar intensity, int u, int v): m_intensity(intensity), m_u(u), m_v(v) {}

bool operator> (const struct Pixel & a, const struct Pixel & b)
{
    return a.m_intensity > b.m_intensity;
}

SingleImageDehazerAbstract::SingleImageDehazerAbstract(const cv::Mat &hazy_img)
{
    assert((hazy_img.type() == CV_8U || hazy_img.type() == CV_8UC3)
           && "input image has to be of type CV_8U or CV_8UC3");
    mHazyImg = hazy_img;
    mnRows = mHazyImg.rows;
    mnCols = mHazyImg.cols;
    mnChs = mHazyImg.channels();
    mAtmosLight[0] = -1.0f, mAtmosLight[1] = -1.0f, mAtmosLight[2] = -1.0f;
    mInitialTransmissionMap = -1.0f * cv::Mat::ones(mnRows, mnCols, CV_32F);
    mFinalTransmissionMap = -1.0f * cv::Mat::ones(mnRows, mnCols, CV_32F);
    mDehazedImg = mHazyImg.clone();
}

//void SingleImageDehazerAbstract::Dehaze()
//{
//    CalcAtmosLight();
//    CalcInitialTransmissionMap();
//    CalcFinalTransmissionMap();
//    CalcDehazedImg();
//}

cv::Vec3f SingleImageDehazerAbstract::GetAtmosLight() const
{
    return mAtmosLight;
}

}