//
// Created by yd2007 on 27/02/23.
//


#ifndef ORB_SLAM2_LOG_H
#define ORB_SLAM2_LOG_H

#include <string>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "PatchAligner.h"

#define SPACE_WIDTH 10

namespace ORB_SLAM2
{
    void MyLog(list<MapPoint*>& localMapPoints, const string& beforeOrAfterLBA, bool isLevelSpecific, bool hasMeanDiff, bool isPatchNormalised, bool isProgressive);
}

#endif //ORB_SLAM2_LOG_H
