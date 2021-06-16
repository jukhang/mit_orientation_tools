#include <Eigen/Dense>

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

#endif // PROJECT_CPPTYPES_H