#include <cmath>
#include <iostream>
#include <type_traits>

#include "cppTypes.h"
#include "MathUtilities.h"

namespace ori
{

    static constexpr double quaternionDerviativeStabilization = 0.1;

    enum class CoordinateAxis
    {
        X,
        Y,
        Z
    };

    // 弧度转角度
    template <typename T>
    T rad2deg(T red)
    {
        static_assert(std::is_floating_point<T>::value,
                      "must use float point value");
        return red * T(180) / T(M_PI);
    }

    // 角度转弧度
    template <typename T>
    T deg2rad(T deg)
    {
        static_assert(std::is_floating_point<T>::value,
                      "must use float point value");
        return deg * T(M_PI) / T(180);
    }

    template <typename T>
    Mat3<T> coordinateRotation(CoordinateAxis axis, T theta)
    {
        static_assert(std::is_floating_point<T>::value,
                      "must use float point value");
        T s = std::sin(theta);
        T c = std::cos(theta);

        Mat3<T> R;

        if (axis == CoordinateAxis::X)
        {
            R << 1, 0, 0, 0, c, s, 0, -s, c;
        }
        else if (axis == CoordinateAxis::Y)
        {
            R << c, 0, -s, 0, 1, 0, s, 0, c;
        }
        else if (axis == CoordinateAxis::Z)
        {
            R << c, s, 0, -s, c, 0, 0, 0, 1;
        }

        return R;
    }

    template <typename T>
    Quat<typename T::Scalar> rotationMatrixToQuaternion(
        const Eigen::MatrixBase<T> &r1)
    {
        static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                      "Must have 3x3 matrix");
        Quat<typename T::Scalar> q;
        Mat3<typename T::Scalar> r = r1.transpose();
        typename T::Scalar tr = r.trace();
        if (tr > 0.0)
        {
            typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
            q(0) = 0.25 * S;
            q(1) = (r(2, 1) - r(1, 2)) / S;
            q(2) = (r(0, 2) - r(2, 0)) / S;
            q(3) = (r(1, 0) - r(0, 1)) / S;
        }
        else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2)))
        {
            typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
            q(0) = (r(2, 1) - r(1, 2)) / S;
            q(1) = 0.25 * S;
            q(2) = (r(0, 1) + r(1, 0)) / S;
            q(3) = (r(0, 2) + r(2, 0)) / S;
        }
        else if (r(1, 1) > r(2, 2))
        {
            typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
            q(0) = (r(0, 2) - r(2, 0)) / S;
            q(1) = (r(0, 1) + r(1, 0)) / S;
            q(2) = 0.25 * S;
            q(3) = (r(1, 2) + r(2, 1)) / S;
        }
        else
        {
            typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
            q(0) = (r(1, 0) - r(0, 1)) / S;
            q(1) = (r(0, 2) + r(2, 0)) / S;
            q(2) = (r(1, 2) + r(2, 1)) / S;
            q(3) = 0.25 * S;
        }
        return q;
    }

    template <typename T>
    Mat3<typename T::Scalar> quaternionToRotationMatrix(
        const Eigen::MatrixBase<T> &q)
    {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                      "Must have 4x1 quat");
        typename T::Scalar e0 = q(0);
        typename T::Scalar e1 = q(1);
        typename T::Scalar e2 = q(2);
        typename T::Scalar e3 = q(3);

        Mat3<typename T::Scalar> R;

        R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
        R.transposeInPlace();
        return R;
    }

    template <typename T>
    Vec3<typename T::Scalar> quatToRPY(const Eigen::MatrixBase<T> &q)
    {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                      "Must have 4x1 quat");
        Vec3<typename T::Scalar> rpy;
        typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
        rpy(2) =
            std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                       square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
        rpy(1) = std::asin(as);
        rpy(0) =
            std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                       square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
        return rpy;
    }

    template <typename T>
    Quat<typename T::Scalar> rpyToQuat(const Eigen::MatrixBase<T> &rpy)
    {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                      "Must have 3x1 vec");
        Mat3<typename T::Scalar> R = rpyToRotMat(rpy);
        Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
        return q;
    }

    /*!
     * Go from rpy to rotation matrix.
     */
    template <typename T>
    Mat3<typename T::Scalar> rpyToRotMat(const Eigen::MatrixBase<T> &v)
    {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                      "must have 3x1 vector");
        Mat3<typename T::Scalar> m = coordinateRotation(CoordinateAxis::X, v[0]) *
                                     coordinateRotation(CoordinateAxis::Y, v[1]) *
                                     coordinateRotation(CoordinateAxis::Z, v[2]);
        return m;
    }
};