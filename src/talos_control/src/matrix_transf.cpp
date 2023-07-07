#include <kdl/frames.hpp>
#include <iostream>

int main(int argc, char **argv) {
  
    // obtained with aruco_ros package
    KDL::Frame bl_2_ar = KDL::Frame(KDL::Rotation::RPY(-1.358, -1.544, 2.875), KDL::Vector(0.9037, 0.03, 0.514));
    // Desired point
    // KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-0.942, 1.307, 2.246), KDL::Vector(-4.89, 2.085, 0.573)); OLD
    // KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-1.57, 1.358,1.57), KDL::Vector(-5.00, 2.04, 0.65)); // NEW //Works
    // KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-2.075, 1.327, 1.052), KDL::Vector(-4.98, 2.048, 0.60)); //  mejor
    // KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-2.409, 1.352, 0.707), KDL::Vector(-4.98, 2.048, 0.60)); // HEY-FIVE
    // KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-1.586, 1.322, 1.573), KDL::Vector(-4.97, 2.093, 0.65)); // GRIPPERS
    KDL::Frame world_2_pto = KDL::Frame(KDL::Rotation::RPY(-1.586, 0.0, 1.573), KDL::Vector(-4.97, 2.093, 0.65)); // GRIPPERS

    // From gazebo
    KDL::Frame world_2_bl = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(-5.625, 1.915, 0.585));
    //Obtained with gazebo
    KDL::Frame world_2_aruco = KDL::Frame(KDL::Rotation::RPY(-1.358, -1.544, 2.875), KDL::Vector(-4.688, 1.945, 1.099));
    KDL::Frame aruco_2_world = world_2_aruco.Inverse();

    KDL::Frame aruco_2_pto = aruco_2_world * world_2_pto;

    std::cout << "XYZ" << std::endl;
    std::cout << aruco_2_pto.p.x() << std::endl;
    std::cout << aruco_2_pto.p.y() << std::endl;
    std::cout << aruco_2_pto.p.z() << std::endl;
    double roll, pitch, yaw;
    aruco_2_pto.M.GetRPY(roll, pitch, yaw);
    std::cout << "RPY" << std::endl;
    std::cout << roll << std::endl;
    std::cout << pitch << std::endl;
    std::cout << yaw << std::endl;

    return 0;

}