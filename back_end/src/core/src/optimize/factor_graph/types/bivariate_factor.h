#ifndef __BIVARIATE_FACTOR_H__
#define __BIVARIATE_FACTOR_H__

#include "../core/factor.h"
#include "variable_type.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class RotVec2RotVecFactor : public Factor
{
public:
    RotVec2RotVecFactor(Pose3D *v_a, Pose3D *v_b, Pose3D measurement, const Eigen::MatrixXd &sqrt_info)
        : m_measurement(measurement), m_sqrt_info(sqrt_info)
    {
        AddVariable(v_a);
        AddVariable(v_b);
        SetSqrtInfo(sqrt_info);
        // SetSqrtInfo(Eigen::MatrixXd::Identity(6,6));
    }

    virtual int Dim() const { return 6; }

    virtual Eigen::VectorXd Error() const override 
    {
        GRAPH_ASSERT(this->NumVariables() == 2);
        Pose3D *v_a = static_cast<Pose3D *>(this->VariableAt(0));
        Pose3D *v_b = static_cast<Pose3D *>(this->VariableAt(1));
        Eigen::VectorXd ret = Eigen::VectorXd::Zero(6);
        
        Pose3D v_ba = v_a->TransformFrom(*v_b);
        RotVec d_r_v(m_measurement.Rot().ToQuaternion().conjugate()*v_a->Rot().ToQuaternion());

        ret.head<3>() = v_ba.Point() - m_measurement.Point();
        ret.tail<3>() = d_r_v.Rot();
        
        // std::cout<<ret.transpose()<<std::endl;
        return ret;
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const override
    {
        Eigen::VectorXd diff = Eigen::VectorXd::Zero(6);
        diff << (e1(0) - e2(0)), (e1(1) - e2(1)), (e1(2) - e2(2)),
            NormalizeAngle(e1(3) - e2(3)), NormalizeAngle(e1(4) - e2(4)), NormalizeAngle(e1(5) - e2(5));
        return diff;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
    double NormalizeAngle(double theta_rad) const
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }
private:
    Pose3D m_measurement;
    Eigen::MatrixXd m_sqrt_info;
};



#endif // __BIVARIATE_FACTOR_H__