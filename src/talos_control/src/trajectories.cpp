// KDL Trajectory
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <ros/ros.h>
#include <iostream>


KDL::Trajectory* trajectoryPlanner(const KDL::Frame start, const KDL::Frame ending, double duration){

    // Geometrical path
    KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis()); 
    path->Add(start); 
    path->Add(ending);
    path->Finish();

    KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();
    //(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, Duration)
    velprof->SetProfileDuration(0, 0.0, 0.0, path->PathLength(), 0, 0.0,  duration); 
    KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velprof);

    return traject;

}

int main(){

    double duration = ros::Duration(7.0).toSec();
    double random_time = ros::Duration(6.56).toSec();

    KDL::Frame start = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0));
    KDL::Frame ending = KDL::Frame(KDL::Rotation::RPY(0.0,1.57,0.0), KDL::Vector(1.0,1.0,1.0));
    KDL::Trajectory* trajectory;

    trajectory = trajectoryPlanner(start, ending, duration);

    KDL::Frame final_frame = trajectory->Pos(7);
    KDL::Frame initial_frame = trajectory->Pos(0.0);
    KDL::Frame random = trajectory->Pos(random_time);

    std::cout << "Final frame: " << std::endl;
    std::cout << "x: " << final_frame.p.x() << std::endl;
    std::cout << "y: " << final_frame.p.y() << std::endl;
    std::cout << "z: " << final_frame.p.z() << std::endl;

    std::cout << "Initial frame: " << std::endl;
    std::cout << "x: " << initial_frame.p.x() << std::endl;
    std::cout << "y: " << initial_frame.p.y() << std::endl;
    std::cout << "z: " << initial_frame.p.z() << std::endl;

    std::cout << "Random frame: " << std::endl;
    std::cout << "x: " << random.p.x() << std::endl;
    std::cout << "y: " << random.p.y() << std::endl;
    std::cout << "z: " << random.p.z() << std::endl;

    return 0;
}


    // KDL::Frame start = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0));
    // KDL::Frame ending = KDL::Frame(KDL::Rotation::RPY(0.0,1.57,0.0), KDL::Vector(1.0,1.0,1.0));

    // // Geometrical path
    // KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis()); 
    // path->Add(start); 
    // path->Add(ending);
    // path->Finish();

    // KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();
    // //(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, Duration)
    // velprof->SetProfileDuration(0, 0.0, 0.0, path->PathLength(), 0, 0.0, 7); 
    // KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velprof);

