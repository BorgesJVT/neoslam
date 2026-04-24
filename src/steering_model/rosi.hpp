#ifndef ROSI_H
#define ROSI_H

#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>

#include <tf2/transform_datatypes.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <epos_msgs/msg/info.hpp>
#include <std_msgs/msg/u_int8.hpp>

// #include <rosi_msgs/msg/vel_control.hpp>
// #include <rosi_msgs/msg/arms_control.hpp>

namespace rosi_controller
{   
    class Rosi /* : public PiControl */
    {       
        private:
            double BODY_WIDTH_WHEELS;
            double BODY_WIDTH_WHEELS_2;
            double INV_BODY_WIDTH_WHEELS;
            double BODY_WIDTH_TRACKS;
            double BODY_WIDTH_TRACKS_2;
            double INV_BODY_WIDTH_TRACKS;
            double WHEELS_RADIUS;
            double WHEELS_RADIUS_2;
            double INV_WHEELS_RADIUS;
            double TRACKS_RADIUS;
            double INV_TRACKS_RADIUS;
            double MASS;
            double INV_MASS;
            double IZZ;
            double INV_IZZ;
            double WHEEL_CURRENT_2_TORQUE;
            double ARM_CURRENT_2_TORQUE;

            sensor_msgs::msg::JointState current_joint_state;
            // std::vector<double> odom_covariance;
            // std::vector<std_msgs::msg::UInt8> epos_state;
            // std::vector<epos_msgs::msg::Info> epos_info;
            
            // rosi_msgs::msg::VelControl twist_cmd;
            // rosi_msgs::msg::ArmsControl arm_cmd;

        public:
            Rosi();
            ~Rosi();

            double setBodyWidthWheels(double value);
            double setBodyWidthTracks(double value);
            double setWheelsRadius(double value);
            double setTracksRadius(double value);
            double setMass(double value);
            double setIzz(double value);
            double setWheelCurrent2Torque(double value);
            double setArmCurrent2Torque(double value);

            inline double mapFromPiToMinusPi(double angle)
            {
                return angle - M_PI_2 * std::floor((angle + M_PI)/(M_PI_2));
            }

            const double* getWheelCurrent2TorquePtr(){return &WHEEL_CURRENT_2_TORQUE;}
            const double* getArmCurrent2TorquePtr(){return &ARM_CURRENT_2_TORQUE;}

            sensor_msgs::msg::JointState* getCurrentJointStatePtr(){return &current_joint_state;}
            // std::vector<std_msgs::msg::UInt8>* getEposStatePtr(){return &epos_state;}
            // std::vector<epos_msgs::msg::Info>* getEposInfoPtr(){return &epos_info;}
            // rosi_msgs::msg::VelControl* getTwistCmdPtr(){return &twist_cmd;}
            // rosi_msgs::msg::ArmsControl* getArmsCmdPtr(){return &arm_cmd;}

            void robot2Wheels(const std::vector<double> robot_vel, std::vector<double>::iterator wheels_vel);
            void robot2Tracks(const std::vector<double> robot_vel, std::vector<double>::iterator tracks_vel);
            void wheels2Robot(const std::vector<double> wheels_vel, std::vector<double>::iterator robot_vel);
            void getOdomVelocity(const std::vector<double> jointVel, double &xLinVel, double &zAngVel);
            void updateRobotStates(Eigen::Vector3d &pos, Eigen::Quaterniond &ori, Eigen::Vector3d &lSpeed, Eigen::Vector3d &aSpeed)
            {

            }
   };
}

#endif
