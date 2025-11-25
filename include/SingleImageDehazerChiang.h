//
// Created by tedyiningding on 26/04/23.
//

#ifndef ORB_SLAM2_SINGLEIMAGEDEHAZERCHIANG_H
#define ORB_SLAM2_SINGLEIMAGEDEHAZERCHIANG_H

#include "SingleImageDehazerAbstract.h"

namespace ORB_SLAM2
{

// https://ieeexplore.ieee.org/document/6104148 "Underwater Image Enhancement by Wavelength Compensation and Dehazing"
class SingleImageDehazerChiang: public ORB_SLAM2::SingleImageDehazerAbstract
{
public:
    void CalcAtmosLight() override;

};

}

#endif //ORB_SLAM2_SINGLEIMAGEDEHAZERCHIANG_H
