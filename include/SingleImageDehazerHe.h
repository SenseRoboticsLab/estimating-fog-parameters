//
// Created by tedyiningding on 24/04/23.
//

#ifndef ORB_SLAM2_SINGLEIMAGEDEHAZERHE_H
#define ORB_SLAM2_SINGLEIMAGEDEHAZERHE_H

#include <numeric>
#include "SingleImageDehazerAbstract.h"

namespace ORB_SLAM2
{

// https://ieeexplore.ieee.org/document/5567108 - "Single image haze removal using dark channel prior"
class SingleImageDehazerHe: public ORB_SLAM2::SingleImageDehazerAbstract
{
public:
    enum eAtmosMode
    {
        MAX_INTENSITY=0,        // the original method proposed in the DCP paper
        MEDIAN_INTENSITY=1,     // the refined method proposed in "Investigating Haze-Relevant Features in a Learning Framework for Image Dehazing"
        MEAN_INTENSITY=2,       // a widely adopted method by many implementations
    };
protected:
    cv::Mat mDarkChannelImg;
    eAtmosMode mAtmosMode;

public:
    SingleImageDehazerHe(const cv::Mat &hazy_img, eAtmosMode atmos_mode=MAX_INTENSITY);

    void CalcDarkChannel(int patch_size = 15);

    void CalcAtmosLight() override;
};

}

#endif //ORB_SLAM2_SINGLEIMAGEDEHAZERHE_H
