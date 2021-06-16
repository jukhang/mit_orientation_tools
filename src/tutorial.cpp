#include <iostream>
#include <Eigen/Dense>
#include "cppTypes.h"
#include "orientation_tools.h"

int main()
{
    Mat3<float> g_R_imu, r_R_imup, Rot;
    g_R_imu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    r_R_imup << 0, 0, 1, 0, -1, 0, 1, 0, 0;
    Rot << 1, 0, 0, 0, 0, 1, 1, 1, 1;

    std::cout << "Cmake World!" << std::endl;
    std::cout << ori::rotationMatrixToQuaternion(g_R_imu.transpose()) << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << ori::rotationMatrixToQuaternion(r_R_imup) << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << ori::coordinateRotation(ori::CoordinateAxis::Y, -M_PI_2) << std::endl;
    // std::cout << g_R_imu * Rot << std::endl;
    // std::cout << Rot.transpose() << std::endl;

    Quat<float>
        quat;
    quat << 1, 0, 0, 0;
    // std::cout << ori::quaternionToRotationMatrix(quat) << std::endl;
    return 0;
}