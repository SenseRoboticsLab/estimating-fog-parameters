//
// Created by yd2007 on 17/03/23.
//

#ifndef ORB_SLAM2_FOGESTIMATION_H
#define ORB_SLAM2_FOGESTIMATION_H

#include <thread>
#include <mutex>
#include <future>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "SingleImageDehazerHe.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <ceres/ceres.h>

#include <chrono>

#define SAVE_FOGGY_OBSERVATIONS 0           // save foggy observations for debug
#define SAVE_FOGGY_OBSERVATIONS_OUR_METHOD 1           // save foggy observations for debug

#define RUN_SINGLE_OURS 1                   // only run one intensity mode
#define OURS_SINGLE_MODE RAW                // which single intensity mode to run

#define TEST_ABLATION 0                   // test ablation settings

#define TEST_PARTIAL_GT 0                   // test partial groundtruth
#define GROUNDTRUTH_A (255.0f * 0.8f)       // 0.7 for VKITTI2, 0.8 for KITTI_CARLA, and 0.9 for DRIVING
#define GROUNDTRUTH_BETA (-std::log(0.05f) / 30.0f)

#define CERES_USE_AUTO_DIFF 1
#define SAVE_BETA_ESTIMATES 0               // save Li's beta estimates for histogram visualization

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;

class ClearVertex: public g2o::BaseVertex<1, float>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override // reset
    {
        _estimate = 0.0f;
    }

    void oplusImpl(const double* update) override // update
    {
        _estimate += (float)*update;
    }

    // leave empty
    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}
};

class BetaAtmosVertex: public g2o::BaseVertex<2, Eigen::Vector2f>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override // reset
    {
        _estimate << 0.0f, 0.0f;
    }

    void oplusImpl(const double* update) override // update
    {
        _estimate += Eigen::Vector2d(update).cast<float>();
    }

    // leave empty
    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}
};

class FoggyEdge: public g2o::BaseBinaryEdge<1, float, ClearVertex, BetaAtmosVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FoggyEdge(float x): BaseBinaryEdge(), _x(x) {}

    void computeError() override
    {
        const ClearVertex* vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const BetaAtmosVertex* vBetaA = static_cast<const BetaAtmosVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const Eigen::Vector2f BetaA_hat = vBetaA->estimate();
        const float Beta_hat = BetaA_hat[0];
        const float A_hat = BetaA_hat[1];

        _error(0,0) = _measurement - ( (J_hat-A_hat)*std::exp(-Beta_hat * _x) + A_hat );
    }

    // compute the Jacobian
    void linearizeOplus() override
    {
        const ClearVertex *vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const BetaAtmosVertex *vBetaA = static_cast<const BetaAtmosVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const Eigen::Vector2f BetaA_hat = vBetaA->estimate();
        const float Beta_hat = BetaA_hat[0];
        const float A_hat = BetaA_hat[1];

        float t = std::exp(-Beta_hat * _x);
        // _jacobianOplusXi is for _vertices[0], i.e. the ClearVertex
        _jacobianOplusXi[0] = -t;
        // _jacobianOplusXj is for _vertices[1], i.e. the BetaAtmosVertex
        _jacobianOplusXj[0] = -(J_hat-A_hat) * t * (-_x);
        _jacobianOplusXj[1] = -(1-t);
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}

public:
    float _x;
};

#if TEST_PARTIAL_GT
class BetaVertex: public g2o::BaseVertex<1, float>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override // reset
    {
        _estimate = 0.0f;
    }

    void oplusImpl(const double* update) override // update
    {
        _estimate += (float)*update;
    }

    // leave empty
    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}
};

class AVertex: public g2o::BaseVertex<1, float>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override // reset
    {
        _estimate = 0.0f;
    }

    void oplusImpl(const double* update) override // update
    {
        _estimate += (float)*update;
    }

    // leave empty
    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}
};

class FoggyBetaEdge: public g2o::BaseBinaryEdge<1, float, ClearVertex, BetaVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FoggyBetaEdge(float x): BaseBinaryEdge(), _x(x) {}

    void computeError() override
    {
        const ClearVertex* vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const BetaVertex* vBeta = static_cast<const BetaVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const float Beta_hat = vBeta->estimate();

        _error(0,0) = _measurement - ( (J_hat-GROUNDTRUTH_A)*std::exp(-Beta_hat * _x) + GROUNDTRUTH_A );
    }

    // compute the Jacobian
    void linearizeOplus() override
    {
        const ClearVertex *vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const BetaVertex *vBeta = static_cast<const BetaVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const float Beta_hat = vBeta->estimate();

        float t = std::exp(-Beta_hat * _x);
        // _jacobianOplusXi is for _vertices[0], i.e. the ClearVertex
        _jacobianOplusXi[0] = -t;
        // _jacobianOplusXj is for _vertices[1], i.e. the BetaVertex
        _jacobianOplusXj[0] = -(J_hat-GROUNDTRUTH_A) * t * (-_x);
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}

public:
    float _x;
};

class FoggyAEdge: public g2o::BaseBinaryEdge<1, float, ClearVertex, AVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FoggyAEdge(float x): BaseBinaryEdge(), _x(x) {}

    void computeError() override
    {
        const ClearVertex* vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const AVertex* vA = static_cast<const AVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const float A_hat = vA->estimate();

        _error(0,0) = _measurement - ( (J_hat-A_hat)*std::exp(-GROUNDTRUTH_BETA * _x) + A_hat );
    }

    // compute the Jacobian
    void linearizeOplus() override
    {
        const ClearVertex *vJ = static_cast<const ClearVertex*>(_vertices[0]);
        const AVertex *vA = static_cast<const AVertex*>(_vertices[1]);
        const float J_hat = vJ->estimate();
        const float A_hat = vA->estimate();

        float t = std::exp(-GROUNDTRUTH_BETA * _x);
        // _jacobianOplusXi is for _vertices[0], i.e. the ClearVertex
        _jacobianOplusXi[0] = -t;
        // _jacobianOplusXj is for _vertices[1], i.e. the AVertex
        _jacobianOplusXj[0] = -(1-t);
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}

public:
    float _x;
};
#endif

typedef Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::ColMajor> VectorXF;
class ClearMinusAtmosVarEdge: public g2o::BaseMultiEdge<-1, VectorXF>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ClearMinusAtmosVarEdge(Eigen::VectorXf x): g2o::BaseMultiEdge<-1, VectorXF>(), _x(x)
    {
        resize(1);
        setDimension(_x.size());
    }

    void setDimension(int dimension_)
    {
        _dimension = dimension_;
        _information.resize(dimension_, dimension_);
        _error.resize(dimension_, 1);
        _measurement.resize(dimension_, 1);
    }

    void computeError() override
    {
        const BetaAtmosVertex* vBetaA = static_cast<const BetaAtmosVertex*>(_vertices[0]);
        const Eigen::Vector2f BetaA_hat = vBetaA->estimate();
        const float Beta_hat = BetaA_hat[0];
        const float A_hat = BetaA_hat[1];

        const int num_obs = _dimension;
        std::vector<float> v_x;
        float x_sum = 0;
        for (int i = 0; i < num_obs; i++)
        {
            float x = (_measurement[i] - A_hat) * std::exp(Beta_hat * _x[i]);
            v_x.push_back(x);
            x_sum += x;
        }
        float x_mean = x_sum / num_obs;

        for (int i = 0; i < num_obs; i++)
        {
            _error[i] = (v_x[i] - x_mean) / std::sqrt(num_obs-1);
        }
    }

    // compute the Jacobian
    void linearizeOplus() override
    {
        const BetaAtmosVertex *vBetaA = static_cast<const BetaAtmosVertex*>(_vertices[0]);
        const Eigen::Vector2f BetaA_hat = vBetaA->estimate();
        const float Beta_hat = BetaA_hat[0];
        const float A_hat = BetaA_hat[1];

        const int num_obs = _dimension;
        std::vector<float> v_x;
        v_x.reserve(num_obs);
        float sum_x_d = 0, sum_e_beta_d = 0;
        for (int i = 0; i < num_obs; i++)
        {
            float e_beta_d = std::exp(Beta_hat * _x[i]);
            float x = (_measurement[i] - A_hat) * e_beta_d;
            v_x.push_back(x);
            sum_x_d += x * _x[i];
            sum_e_beta_d += e_beta_d;
        }

        Eigen::Matrix<double, Eigen::Dynamic, 2> J(_dimension, 2);
        for (int i = 0; i < num_obs; i++)
        {
            J(i, 0) = (v_x[i] * _x[i] - sum_x_d/num_obs) / std::sqrt(num_obs-1);
            J(i, 1) = (-std::exp(Beta_hat * _x[i]) + sum_e_beta_d/num_obs) / std::sqrt(num_obs-1);
        }
        _jacobianOplus[0] = J;
    }

    bool read(std::istream& in) override {}
    bool write(std::ostream& out) const override {}

public:
    Eigen::VectorXf _x;
};

struct FoggyResidualAuto {
    FoggyResidualAuto(double x, double y, double w)
            : d_(x), I_(y), w_(w) {}

    template <typename T>
    bool operator()(const T* const J, const T* const beta, const T* const A, T* residual) const {
        residual[0] = ( I_ - ( (J[0] - A[0])*ceres::exp(-beta[0] * d_) + A[0] ) ) * ceres::sqrt(w_);
        return true;
    }

private:
    // Observations for a sample.
    const double d_;
    const double I_;
    const double w_;        // the sqrt of the diagonal element of the information matrix
};

class FoggyResidualAnalytic : public ceres::SizedCostFunction<1, 1, 1, 1> {
public:
    FoggyResidualAnalytic(const double d, const double I, const double w) : d_(d), I_(I), w_(w) {}
    virtual ~FoggyResidualAnalytic() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double J = parameters[0][0];
        const double beta = parameters[1][0];
        const double A = parameters[2][0];

        double t = ceres::exp(-beta * d_);
        residuals[0] = ( I_ - ( (J - A)*t + A ) ) * ceres::sqrt(w_);

        if (!jacobians) return true;

        jacobians[0][0] = -t;
        jacobians[1][0] = -(J-A) * t * (-d_);
        jacobians[2][0] = -(1-t);
        return true;
    }

private:
    const double d_;
    const double I_;
    const double w_;        // the sqrt of the diagonal element of the information matrix
};

struct FoggyResidualAutoGTA {
    FoggyResidualAutoGTA(double x, double y, double w, double A)
            : d_(x), I_(y), w_(w), A_(A) {}

    template <typename T>
    bool operator()(const T* const J, const T* const beta, T* residual) const {
        residual[0] = ( I_ - ( (J[0] - A_)*ceres::exp(-beta[0] * d_) + A_ ) ) * ceres::sqrt(w_);
        return true;
    }

private:
    // Observations for a sample.
    const double d_;
    const double I_;
    const double w_;        // the sqrt of the diagonal element of the information matrix
    const double A_;
};

struct FoggyResidualAutoGTBeta {
    FoggyResidualAutoGTBeta(double x, double y, double w, double beta)
            : d_(x), I_(y), w_(w), beta_(beta) {}

    template <typename T>
    bool operator()(const T* const J, const T* const A, T* residual) const {
        residual[0] = ( I_ - ( (J[0] - A[0])*ceres::exp(-beta_ * d_) + A[0] ) ) * ceres::sqrt(w_);
        return true;
    }

private:
    // Observations for a sample.
    const double d_;
    const double I_;
    const double w_;        // the sqrt of the diagonal element of the information matrix
    const double beta_;
};

struct FoggyObservation
{
    MapPoint* m_pMP;
    KeyFrame* m_pKF;
    float m_dist, m_intensity;
    cv::Vec4f m_intensities;        // B, G, R, Grey
    int m_octave;
    float m_gradient_squared;
    FoggyObservation(MapPoint* pMP, KeyFrame* pKF, float dist, float intensity, int octave, float gradient_squared=-1.0f);
    FoggyObservation(MapPoint* pMP, KeyFrame* pKF, float dist, const cv::Vec4f& intensities, int octave, float gradient_squared=-1.0f);
};

bool operator< (const struct FoggyObservation & a, const struct FoggyObservation & b);
bool operator> (const struct FoggyObservation & a, const struct FoggyObservation & b);

struct BetaAtmos
{
    float m_beta, m_atmos;
    BetaAtmos(float beta = 0.02f, float atmos = 255.0f);
};

class FogEstimation
{
public:
    enum eIntensityMode
    {
        RAW=0,              // use raw intensity
        PROJ=1,             // use projected intensity
        REFD_REF_NONPRG=2,  // use refined intensity, ORB-SLAM2's RKF as RKF, non-progressive alignment
        REFD_MED_NONPRG=3,  // use refined intensity, the KF with median distance as RKF, non-progressive alignment
        REFD_REF_PRG=4,     // use refined intensity, ORB-SLAM2's RKF as RKF, progressive alignment
        REFD_MED_PRG=5,     // use refined intensity, the KF with median distance as RKF, progressive alignment
    };

public:

    FogEstimation(Map* pMap, const string &strPhotometricSettings);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    // called by the local mapping thread
    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    void RequestFinish();

    bool isFinished();

    float GetBeta() const;
    float GetAtmos() const;
    void SetBeta(float beta);
    void SetAtmos(float atmos);

    void UpdateEstimations(KeyFrame* pKF, float beta, float atmos);
    void EraseEstimation();

    static float CalcDistMP2KF(MapPoint* pMP, KeyFrame* pKF);

    bool GetBusy();                         // mbBusy's get method
    void SetBusy(bool flag);                // mbBusy's set method

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // FogEstimation states     // TODO
    enum eFogEstimationState
    {
        NOT_READY=-1,           // there are not enough KFs in the system
        NOT_INITIALIZED=0,      // there are enough KFs but no valid estimates yet
        OK=1,                   // estimates from the latest KF is valid
        NOT_UP_TO_DATE=2        // estimates from the latest KF is not valid but there has been valid estimate before
    };

    const std::array<std::array<float ,3>, 4> mPhotometricParams;       // 1st dimension BGR+grey, 2nd dimension abc
    // from intensity to radiance
    float GammaExpansion(float intensity);               // grayscale
    float GammaExpansion(float intensity, int c);        // per channel
    cv::Vec3f GammaExpansion(cv::Vec3f intensities);     // BGR
    cv::Vec4f GammaExpansion(cv::Vec4f intensities);     // BGR + grey
    // from radiance to intensity
    float GammaCompression(float radiance);              // grayscale
    float GammaCompression(float radiance, int c);       // per channel
    cv::Vec3f GammaCompression(cv::Vec3f radiances);     // BGR
    cv::Vec4f GammaCompression(cv::Vec4f radiances);     // BGR + grey

protected:

    bool CheckNewKeyFrames();

    bool NeedUpdateFog();

    void UpdateFog();
    void UpdateFog(eIntensityMode intensity_mode);

    void CollectLMPs(list<MapPoint *> &lLocalMapPoints);        // collect LKFs and LMPs
    void GenerateFoggyObservationsRawProj(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, std::vector<std::vector<FoggyObservation>> &vvFoggyObservationLi, std::vector<std::vector<FoggyObservation>> &vvFoggyObservationWoGc, eIntensityMode intensity_mode, bool also_li=false, bool also_wo_gc=false);
    void GenerateFoggyObservationsRefdRefNonPrg(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation);
    void GenerateFoggyObservationsRefdMedNonPrg(const list<MapPoint *> &lLocalMapPoints, std::vector<std::vector<FoggyObservation>> &vvFoggyObservation);
    void GenerateFoggyObservationsAllModes(const list<MapPoint *> &lLocalMapPoints, std::map<eIntensityMode, std::vector<std::vector<FoggyObservation>>> &FoggyObservationsAllModes);
    void InitialiseEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates);
    void InitialiseEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li=false);
    void InitialiseEstimates2(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode);
    void InitialiseEstimates3(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode);
    void InitialiseEstimates4(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li=false);
    void InitialiseEstimates5(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode);
    void InitialiseEstimates6(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li=false);
    void InitialiseEstimates7(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, vector<float> &estimates, eIntensityMode intensity_mode, bool use_li=false);
    void InitialiseEstimatesAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, array<vector<float>, 4> &estimates, eIntensityMode intensity_mode, bool use_li=false);
    void InitialiseEstimatesAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, array<vector<float>, 4> &estimates, eIntensityMode intensity_mode);
    void OptimiseFog0(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation);
    void OptimiseFogCeresLoose(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);      // one-stage, abs(J-A) * inlier_count
    void OptimiseFogG2o(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count then uniform
    void OptimiseFogCeresUnbounded(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, uniform
    void OptimiseFogCeresTight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count
    void OptimiseFogCeresTightProductWeight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A), ceres
    void OptimiseFogCeresTightUniformWeight(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count
    void OptimiseFogCeresTightOneStage(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count
    void OptimiseFogCeresTightAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count
    void OptimiseFogCeresTightAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode);     // two-stage, abs(J-A) * inlier_count
    void UpdateEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates);
    void UpdateEstimates(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates2(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates3(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates4(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates5(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates6(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimates7(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode);
    void UpdateEstimatesAllChannels(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode, int nChannel);
    void UpdateEstimatesAllChannelsWoGc(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, const vector<float> &estimates, const vector<bool> &vIsMPActive, eIntensityMode intensity_mode, int nChannel);

#if TEST_PARTIAL_GT
    // partial groundtruth
    void OptimiseBeta(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode, bool use_li_to_init=false);     // two-stage, abs(J-A) * inlier_count (ours)
    void OptimiseA(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, eIntensityMode intensity_mode, bool use_li_to_init=false);     // two-stage, abs(J-A) * inlier_count (ours)
#endif

    // Li's method to estimate A and beta
    // Sec. 4.5 in https://openaccess.thecvf.com/content_cvpr_2015/papers/Li_Simultaneous_Video_Defogging_2015_CVPR_paper.pdf
    void EstimateBetaLi(const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, int channel_index, bool preserve_positive_beta_only, float atmos_estimate, float* beta_estimate, std::vector<float>* vec_valid_beta_estimate, long* total_pairs_count);
    void EstimateFogLi(SingleImageDehazerHe::eAtmosMode eAtmosIntensityMode, const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, bool preserve_positive_beta_only, float atmos_gt);
    void EstimateFogLiAllChannels(SingleImageDehazerHe::eAtmosMode eAtmosIntensityMode, const std::vector<std::vector<FoggyObservation>> &vvFoggyObservation, bool preserve_positive_beta_only, cv::Vec4f atmos_gt);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;
    KeyFrameDatabase* mpKeyFrameDB;
    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpFogKeyFrameQueue;

    std::mutex mMutexFogQueue;

    KeyFrame* mpCurrentKF;
    bool mbIsEstimatePerColourChannel;

    bool mbHasFogBeenEstimated;
    cv::Mat mPreviousFogEstimateKFPos;

    long unsigned int mLastFogKFid;

    BetaAtmos mCurrentEstimate;
    map<KeyFrame*, BetaAtmos> mEstimates;
    eFogEstimationState mState;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes;
    map<eIntensityMode, eFogEstimationState> mAllModesState;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate2;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates2;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes2;
    map<eIntensityMode, eFogEstimationState> mAllModesState2;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate3;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates3;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes3;
    map<eIntensityMode, eFogEstimationState> mAllModesState3;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate4;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates4;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes4;
    map<eIntensityMode, eFogEstimationState> mAllModesState4;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate5;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates5;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes5;
    map<eIntensityMode, eFogEstimationState> mAllModesState5;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate6;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates6;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes6;
    map<eIntensityMode, eFogEstimationState> mAllModesState6;

    map<eIntensityMode, BetaAtmos> mAllModesCurrentEstimate7;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesEstimates7;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimes7;
    map<eIntensityMode, eFogEstimationState> mAllModesState7;

    map<eIntensityMode, array<BetaAtmos, 4>> mAllModesCurrentEstimateAllChannels;
    map<eIntensityMode, map<KeyFrame*, array<BetaAtmos, 4>>> mAllModesEstimatesAllChannels;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, array<int, 4>>> mAllModesNumInlierTimesAllChannels;
    map<eIntensityMode, array<eFogEstimationState, 4>> mAllModesStateAllChannels;

    map<eIntensityMode, array<BetaAtmos, 4>> mAllModesCurrentEstimateAllChannelsWoGc;
    map<eIntensityMode, map<KeyFrame*, array<BetaAtmos, 4>>> mAllModesEstimatesAllChannelsWoGc;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, array<int, 4>>> mAllModesNumInlierTimesAllChannelsWoGc;
    map<eIntensityMode, array<eFogEstimationState, 4>> mAllModesStateAllChannelsWoGc;

    map<bool, BetaAtmos> mAllModesCurrentEstimateLiMax;
    map<bool, map<KeyFrame*, BetaAtmos>> mAllModesEstimatesLiMax;
    map<bool, BetaAtmos> mAllModesCurrentEstimateLiMedian;
    map<bool, map<KeyFrame*, BetaAtmos>> mAllModesEstimatesLiMedian;
    map<bool, BetaAtmos> mAllModesCurrentEstimateLiMean;
    map<bool, map<KeyFrame*, BetaAtmos>> mAllModesEstimatesLiMean;

    map<bool, array<BetaAtmos, 4>> mAllModesCurrentEstimateLiMaxAllChannels;
    map<bool, map<KeyFrame*, array<BetaAtmos, 4>>> mAllModesEstimatesLiMaxAllChannels;
    map<bool, array<BetaAtmos, 4>> mAllModesCurrentEstimateLiMedianAllChannels;
    map<bool, map<KeyFrame*, array<BetaAtmos, 4>>> mAllModesEstimatesLiMedianAllChannels;
    map<bool, array<BetaAtmos, 4>> mAllModesCurrentEstimateLiMeanAllChannels;
    map<bool, map<KeyFrame*, array<BetaAtmos, 4>>> mAllModesEstimatesLiMeanAllChannels;

#if TEST_PARTIAL_GT
    // partial groundtruth
    map<eIntensityMode, BetaAtmos> mAllModesCurrentBetaEstimate;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesBetaEstimates;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimesBeta;
    map<eIntensityMode, eFogEstimationState> mAllModesStateBeta;
    map<eIntensityMode, BetaAtmos> mAllModesCurrentAEstimate;
    map<eIntensityMode, map<KeyFrame*, BetaAtmos>> mAllModesAEstimates;
    map<eIntensityMode, map<pair<MapPoint*, KeyFrame*>, int>> mAllModesNumInlierTimesA;
    map<eIntensityMode, eFogEstimationState> mAllModesStateA;


    map<bool, float> mAllModesCurrentBetaEstimateLiMax;
    map<bool, map<KeyFrame*, float>> mAllModesBetaEstimatesLiMax;
    map<bool, float> mAllModesCurrentBetaEstimateLiMedian;
    map<bool, map<KeyFrame*, float>> mAllModesBetaEstimatesLiMedian;
    map<bool, float> mAllModesCurrentBetaEstimateLiMean;
    map<bool, map<KeyFrame*, float>> mAllModesBetaEstimatesLiMean;

    map<bool, cv::Vec3f> mAllModesCurrentBetaEstimateLiMaxAllChannels;
    map<bool, map<KeyFrame*, cv::Vec3f>> mAllModesBetaEstimatesLiMaxAllChannels;
    map<bool, cv::Vec3f> mAllModesCurrentBetaEstimateLiMedianAllChannels;
    map<bool, map<KeyFrame*, cv::Vec3f>> mAllModesBetaEstimatesLiMedianAllChannels;
    map<bool, cv::Vec3f> mAllModesCurrentBetaEstimateLiMeanAllChannels;
    map<bool, map<KeyFrame*, cv::Vec3f>> mAllModesBetaEstimatesLiMeanAllChannels;
#endif

    std::vector<std::future<void>> mFutures;        // for asynchronously multi-threading

    bool mbBusy;                // is the thread busy
    std::mutex mMutexBusy;

protected:
    static const int MIN_NUM_OBS;               // min number of observations of a MP for it to be valid
    static const float MIN_DIST_BETWEEN_KFS;    // min distance between KFs to update fog parameters
//    static const float MIN_INTENSITY_RANGE;     // min intensity range of a MP for it to be valid
    static const int MIN_NUM_MPS;               // min number of valid MPs to conduct optimisation
    static const float TH_HUBER2;               // delta^2 value in Huber loss

    static const float MU;                      // mu constant in the gradient-related weight

    static const float TH_SLOPE;                // threshold for the slope to determine if J<A or J>A

    static const float BETA_LOW;
    static const float BETA_HIGH;
    static const float ATMOS_LOW;
    static const float ATMOS_HIGH;
    static const float J_LOW;
    static const float J_HIGH;

    static const float HIST_BIN_WIDTH;
    static const float HIST_RANGE_LOW;
    static const float HIST_RANGE_HIGH;
    static const float HIST_RANGE_LOW_POSITIVE;

public:
    static const std::map<std::string, std::array<std::array<float ,3>, 4>> PHOTOMETRIC_PARAMS_ALL;
    static const bool APPLY_GAMMA_CORRECTION;
};

} //namespace ORB_SLAM

#endif //ORB_SLAM2_FOGESTIMATION_H