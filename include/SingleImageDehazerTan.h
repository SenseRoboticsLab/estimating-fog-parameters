//
// Created by tedyiningding on 26/04/23.
//

#ifndef ORB_SLAM2_SINGLEIMAGEDEHAZERTAN_H
#define ORB_SLAM2_SINGLEIMAGEDEHAZERTAN_H

#include "SingleImageDehazerAbstract.h"

namespace ORB_SLAM2
{

// https://ieeexplore.ieee.org/document/4587643 - "Visibility in bad weather from a single image"
class SingleImageDehazerTan: public ORB_SLAM2::SingleImageDehazerAbstract
{
public:
    void CalcAtmosLight() override;

};

}

#endif //ORB_SLAM2_SINGLEIMAGEDEHAZERTAN_H
