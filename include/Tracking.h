/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include "FogEstimation.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class Frame;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetFogEstimator(FogEstimation* pFogEstimator);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,    // 系统没有准备好,一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0,        // 还没有接收到输入图像
        NOT_INITIALIZED=1,      // 接收到图像但未初始化成功
        OK=2,                   // 跟踪成功
        LOST=3                  // 跟踪失败
    };

    eTrackingState mState;              // 当前帧mCurrentFrame的跟踪状态
    eTrackingState mLastProcessedState; // 前一帧mLastFrame的跟踪状态

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;            // 当前正在处理的帧
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;              // 单目初始化中参考帧与当前帧的匹配关系
    std::vector<cv::Point2f> mvbPrevMatched;    // 单目初始化参考帧地图点
    std::vector<cv::Point3f> mvIniP3D;          // 单目初始化中三角化得到的地图点坐标
    Frame mInitialFrame;                        // 单目初始化参考帧(实际上就是前一帧)

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();            // 双目相机初始化，同时建立初始局部地图

    // Map initialization for monocular
    void MonocularInitialization();         // 单目相机初始化
    void CreateInitialMapMonocular();       // 单目初始化成功后建立初始局部地图

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();          // 根据参考帧估计位姿
    void UpdateLastFrame();                 // 对于双目/RGBD相机,为前一帧生成临时地图点
    bool TrackWithMotionModel();            // 根据恒速运动模型估计初始位姿

    bool Relocalization();

    void UpdateLocalMap();                  // 更新局部地图
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();                   // 更新局部地图并优化当前帧位姿
    void SearchLocalPoints();               // 将局部地图点投影到当前帧特征点上

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;
    FogEstimation* mpFogEstimator;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;             // 单目初始化器

    //Local Map
    KeyFrame* mpReferenceKF;    // 参考关键帧, 初始化成功的帧会被设为参考关键帧; TrackReferenceKeyFrame()与该关键帧匹配搜索关键点
    // 以下两个构成局部地图。即 局部关键帧 + 局部地图点 构成 局部地图
    std::vector<KeyFrame*> mvpLocalKeyFrames;   // 局部关键帧列表,初始化成功后向其中添加局部关键帧
    std::vector<MapPoint*> mvpLocalMapPoints;   // 局部地图点列表,初始化成功后向其中添加局部地图点
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;                   // 前一帧,TrackWithMotionModel()与该帧匹配搜索关键点
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;                  // 相机前一帧运动速度,跟踪完局部地图后更新该成员变量

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;  // 双目/RGBD相机输入时,为前一帧生成的临时地图点,跟踪成功后该容器会被清空,其中的地图点会被删除
};

} //namespace ORB_SLAM

#endif // TRACKING_H
