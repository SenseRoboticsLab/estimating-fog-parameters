//
// Created by tedyiningding on 26/04/23.
//

#include "SingleImageDehazerChiang.h"

namespace ORB_SLAM2
{

void SingleImageDehazerChiang::CalcAtmosLight()
{
    // as per eq. (15)

    // min operation per patch
    const int patch_size = 15;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(patch_size, patch_size));
    cv::Mat eroded_img;
    cv::erode(mHazyImg, eroded_img, kernel);

    // get atmos light
    if (mnChs == 3)
    {
        cv::Mat eroded_img_channel_split[mnChs];
        cv::split(eroded_img, eroded_img_channel_split);
        for (int c=0; c < mnChs; c++)
        {
            double max_intensity;
            cv::minMaxLoc(eroded_img_channel_split[c], nullptr, &max_intensity, nullptr, nullptr);
            mAtmosLight[c] = float(max_intensity);
        }
    }
    else
    {
        double max_intensity;
        cv::minMaxLoc(eroded_img, nullptr, &max_intensity, nullptr, nullptr);
        mAtmosLight[0] = float(max_intensity);
        mAtmosLight[1] = float(max_intensity);
        mAtmosLight[2] = float(max_intensity);
    }
}

}