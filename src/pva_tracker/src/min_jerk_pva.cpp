//
// Created by cc on 2020/8/5.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <string>

using namespace std;

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

geometry_msgs::Vector3 jerk_pva_set_P;
geometry_msgs::Vector3 jerk_pva_set_V;
geometry_msgs::Vector3 jerk_pva_set_A;

geometry_msgs::Vector3 jerk_take_off;

// tool func
template<typename T>
void readParam(ros::NodeHandle &nh, std::string param_name, T& loaded_param) {
    // template to read param from roslaunch
    const string& node_name = ros::this_node::getName();
    param_name = node_name + "/" + param_name;
    if (!nh.getParam(param_name, loaded_param)) {
        ROS_ERROR_STREAM("Failed to load " << param_name << ", use default value");
        //TODO:print default value
    }
    else{
        ROS_INFO_STREAM("Load " << param_name << " success");
        //TODO:print loaded value
    }
}

int swarm_ID = 0;
int ctrl_rate = 30;

std::string uav_name = "/uav"+to_string(swarm_ID);
void loadRosParams(ros::NodeHandle &nh) {
    readParam<int>(nh, "swarm_ID", swarm_ID);
    readParam<int>(nh, "ctrl_rate", ctrl_rate);

    uav_name = "/uav" + to_string(swarm_ID);
}

// void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void jerkPosCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    jerk_pva_set_P = *msg;
}

void jerkVelCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    jerk_pva_set_V = *msg;
}

void jerkAccCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    jerk_pva_set_A = *msg;
}

void jerkTakeOffCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    jerk_take_off = *msg;
}


void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));

    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));

    pva_pub.publish(pva_setpoint);

//    ROS_INFO_THROTTLE(1.0, "P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO_THROTTLE(1.0, "V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO_THROTTLE(1.0, "A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);

//    ROS_INFO("P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO("V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO("A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
}



int main(int argc  , char** argv )
{

    ros::init(argc, argv, "straight_line");
    ros::NodeHandle nh;

    // load param
    loadRosParams(nh);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 1, stateCallback);
    // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 1, positionCallback);

    ros::Subscriber jerk_pva_p_sub = nh.subscribe<geometry_msgs::Vector3>(uav_name + "/jerk_pva_set_p", 1, jerkPosCallback);
    ros::Subscriber jerk_pva_v_sub = nh.subscribe<geometry_msgs::Vector3>(uav_name + "/jerk_pva_set_v", 1, jerkVelCallback);
    ros::Subscriber jerk_pva_a_sub = nh.subscribe<geometry_msgs::Vector3>(uav_name + "/jerk_pva_set_a", 1, jerkAccCallback);

    ros::Subscriber jerk_pva_take_off = nh.subscribe<geometry_msgs::Vector3>(uav_name + "/take_off_pva", 1, jerkTakeOffCallback);


    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>(uav_name + "/pva_setpoint", 1);

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);


    double delt_t = 1.0 / LOOPRATE;

    double yaw_target= 0;
    Vector3d recorded_p = current_p;

    double offset = 2;
    while(ros::ok())
    {
        if(jerk_take_off.x) {
            if (current_state.mode != "OFFBOARD") {
                ROS_INFO_STREAM_THROTTLE(1, uav_name + " waiting for Offboard cmd");
                setPVA(current_p, Vector3d::Zero(), Vector3d::Zero(), yaw_target);
                recorded_p = current_p;
            } else {
                Eigen::Vector3d goal_p = recorded_p;
                //            goal_p(0) += offset;
                goal_p(0) = jerk_pva_set_P.x;
                goal_p(1) = jerk_pva_set_P.y;
                goal_p(2) = jerk_pva_set_P.z;

                Eigen::Vector3d goal_v;
//                goal_v(0) = jerk_pva_set_V.x;
//                goal_v(1) = jerk_pva_set_V.y;
//                goal_v(2) = jerk_pva_set_V.z;
//
                goal_v(0) = 0;
                goal_v(1) = 0;
                goal_v(2) = 0;

                Eigen::Vector3d goal_a;
//                goal_a(0) = jerk_pva_set_A.x;
//                goal_a(1) = jerk_pva_set_A.y;
//                goal_a(2) = jerk_pva_set_A.z;

                goal_a(0) = 0;
                goal_a(1) = 0;
                goal_a(2) = 0;

                setPVA(goal_p, goal_v, goal_a, yaw_target);
//                ROS_INFO_THROTTLE(1, "GOAL_POSITION: %f %f %f", goal_p(0), goal_p(1), goal_p(2));
            }
        }
        else {
            ROS_INFO_STREAM_THROTTLE(1, uav_name + " waiting For PerFlight work!");
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}