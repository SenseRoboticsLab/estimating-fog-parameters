//
// Created by tedyiningding on 26/04/23.
//

#include "SingleImageDehazerTan.h"

namespace ORB_SLAM2
{

void SingleImageDehazerTan::CalcAtmosLight()
{
    // according to the paper, the global atmospheric light can be obtained from pixels that have the highest intensity in the input image

    if (mnChs == 3)
    {
        // convert the hazy image from colour to gray
        cv::Mat hazy_gray_img_from_colour;
        cv::cvtColor(mHazyImg, hazy_gray_img_from_colour, cv::COLOR_BGR2GRAY);

        // locate the highest intensity
        cv::Point max_loc;
        cv::minMaxLoc(hazy_gray_img_from_colour, nullptr, nullptr, nullptr, &max_loc);

        // get the atmospheric light
        cv::Vec3b atmos_u8 = mHazyImg.at<cv::Vec3b>(max_loc.y, max_loc.x);
        mAtmosLight[0] = float(atmos_u8[0]);
        mAtmosLight[1] = float(atmos_u8[1]);
        mAtmosLight[2] = float(atmos_u8[2]);
    }
    else
    {
        // find the highest intensity
        double max_intensity;
        cv::minMaxLoc(mHazyImg, nullptr, &max_intensity, nullptr, nullptr);

        // get the atmospheric light
        mAtmosLight[0] = float(max_intensity);
        mAtmosLight[1] = float(max_intensity);
        mAtmosLight[2] = float(max_intensity);
    }
}

}