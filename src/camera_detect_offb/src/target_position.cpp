//
// Created by up on 2020/9/22.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include<cv_bridge/cv_bridge.h>

#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <chrono>
#include<sstream>

#include<librealsense2/rs.hpp>
#include <csignal>

using namespace std;

/*
 * global variable
 */
geometry_msgs::PoseStamped plane_atti_msg;
geometry_msgs::PoseStamped vision_pose_msg;

cv::Point2i target_left_up;
cv::Point2i target_right_down;
cv::Point2i pickup_left_up;
cv::Point2i pickup_right_down;
int target_width;
int target_height;
bool targetDetectFlag = false;
int id_name;
Eigen::Vector4d target_position_of_img;
Eigen::Vector4d target_position_of_drone;
Eigen::Vector4d target_position_of_world;
geometry_msgs::Vector3 drone_euler;
geometry_msgs::Vector3 drone_euler_init;
Eigen::Quaterniond drone_quaternion;
Eigen::Vector3d drone_pos_vision;
geometry_msgs::PoseStamped msg_drone_pos_vision;
geometry_msgs::PoseStamped msg_target_pose_from_img;
geometry_msgs::PoseStamped msg_target_pose_world;
bool got_attitude_init = false;

int swarm_ID = 0;
geometry_msgs::Vector3 drone_euler_real;
std::string uav_name = "/uav"+to_string(swarm_ID);

//param
//uav01
// double cx_color = 338.960065;
// double cy_color = 244.856900;
// double fx_color = 446.8877;
// double fy_color = 446.032355;

//uav02
double cx_color = 341.530207;
double cy_color = 243.635387;
double fx_color = 451.664427;
double fy_color = 450.081841;

Eigen::Isometry3d tf_image_to_enu;
Eigen::Isometry3d tf_camera_to_drone;
Eigen::Isometry3d tf_drone_to_world;
Eigen::Vector3d tf_camera_drone;

//yolo
float target_width_world = 0.30;
float scale_size = 1;
cv::Point2f yolo_center;

std::queue<geometry_msgs::PoseStamped> pose_queue; //???????????????quene
double image_ros_time = 0.0;


// function declarations
void plane_attitude_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
void vision_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
void target_corner_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
void tf_param_set();
void get_init_yaw();
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

// watchsignal flag
bool flag = true;

void shutdown_handler(int)
{
    cout<<"shutdown"<<endl;
    flag = false;
} 

// capture the signal from keyboard
void watchSignal() {
    signal(SIGINT, shutdown_handler);
    signal(SIGTERM, shutdown_handler);
    signal(SIGKILL, shutdown_handler);
    signal(SIGQUIT, shutdown_handler);
}

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

void loadRosParams(ros::NodeHandle &nh) {
    readParam<int>(nh, "swarm_ID", swarm_ID);
    uav_name = "/uav" + to_string(swarm_ID);
}

//main function
int main(int argc, char **argv) {

    ros::init(argc, argv, "pnp_target_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate rate(30);
    loadRosParams(nh);

    // ??????????????????,????????????quene???
    ros::Subscriber plane_attitude = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose",1,plane_attitude_sub);
    // ??????????????????,????????????targetDetectFlag??????1???????????????????????????????????????????????????????????????
    ros::Subscriber target_corner = nh.subscribe<geometry_msgs::PoseStamped>("yolo_target_corner",1,target_corner_sub);
    // vision_pose
    ros::Subscriber vision_pose = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose",1,vision_pose_sub);
    // ???????????????????????????
    ros::Publisher msg_target_pose_from_img_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_target_pose_from_img",1);
    // ?????????????????????????????????
    ros::Publisher drone_pos_vision_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose_drone",1);
    // target pose to the world
    ros::Publisher target_pose_world_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose_world",1);


    //tf ????????????
    tf_param_set();

    //ros::duration
    cout << "Start Ros Duration" << endl;
    ros::Duration(5).sleep();
    cout << "End Ros Duration" << endl;

    //????????????????????????
    while (! got_attitude_init){
        ros::spinOnce();
        ROS_INFO("getting yaw init ... ");
        //git initial yaw
        get_init_yaw(); 
        rate.sleep();
    }


    while(ros::ok() && flag){
        // check if there is signal input from the keyboard
        watchSignal();

         ros::spinOnce();//??????????????????
         if(!targetDetectFlag){
             //???????????????
             msg_target_pose_from_img.header.stamp = ros::Time::now();
             msg_target_pose_from_img.pose.position.z = 0;
             msg_target_pose_from_img.pose.position.x = 0;
             msg_target_pose_from_img.pose.position.y = 0;
             msg_target_pose_from_img.pose.orientation.w = -1000;
             //???????????????
             msg_drone_pos_vision.header.stamp = ros::Time::now();
             msg_drone_pos_vision.pose.position.z = 0;
             msg_drone_pos_vision.pose.position.x = 0;
             msg_drone_pos_vision.pose.position.y = 0;
             msg_drone_pos_vision.pose.orientation.w = -1000;
         }
         else{
               //???????????????????????????????????????
               target_position_of_img.x() = msg_target_pose_from_img.pose.position.x;
               target_position_of_img.y() = msg_target_pose_from_img.pose.position.y;
               target_position_of_img.z() = msg_target_pose_from_img.pose.position.z;
               //?????????????????????????????????^^^

               //??????????????????TF????????????????????????????????????????????????target_position_of_drone
               target_position_of_drone = tf_camera_to_drone * (tf_image_to_enu * target_position_of_img);
            //    cout << "target_position_of_drone-x = " << target_position_of_drone.x() << endl;
                // cout << "target_position_of_drone-y = " << target_position_of_drone.y() << endl;
            //    cout << "target_position_of_drone-z = " << target_position_of_drone.z() << endl;

               //??????????????????TF????????????????????????????????????????????????target_position_of_drone ^^^

               //???????????????????????????????????????
               geometry_msgs::PoseStamped synchronized_att;
               while(1){
                   if(pose_queue.empty()){ //????????????
                       break;
                   }
                   else{ //?????????
                       synchronized_att = pose_queue.front();
                       double temp = double(synchronized_att.header.stamp.toSec());
                      // printf("tenp is %.6f\n",temp);
                      // printf("pic is %.6f\n",image_ros_time);
                       if(image_ros_time >= temp){
                           break;
                       }else{
                           pose_queue.pop();
                       }
                   }
               }
               //???????????????????????????????????????^^^

               //?????????????????????euler???
               drone_euler = quaternion2euler(synchronized_att.pose.orientation.x,synchronized_att.pose.orientation.y,synchronized_att.pose.orientation.z,synchronized_att.pose.orientation.w);
               //?????????????????????euler???^^^

               //????????????euler???
            //    cout << "euler-z real-time = " << drone_euler.z << endl;
            //    cout << "init ------z =" << drone_euler_init.z << endl;
               drone_euler.z  = drone_euler.z - drone_euler_init.z;
               
            //    cout << "euler-z final = " << drone_euler.z << endl;
               //????????????euler???^^^

               //?????????????????????????????????
               drone_quaternion = euler2quaternion_eigen(drone_euler.x,drone_euler.y,drone_euler.z);
               //?????????????????????????????????^^^

               //???????????????????????????????????????
               tf_drone_to_world = Eigen::Isometry3d::Identity();
               tf_drone_to_world.prerotate(drone_quaternion.toRotationMatrix());
               tf_drone_to_world.pretranslate(Eigen::Vector3d(0,0,0));
               //???????????????????????????????????????^^^

               //???????????????????????????
               target_position_of_world = tf_drone_to_world * target_position_of_drone;
            //    cout <<  target_position_of_drone.x();

               drone_pos_vision.x() = target_position_of_world.x();
               drone_pos_vision.y() = target_position_of_world.y();
               drone_pos_vision.z() = target_position_of_world.z();
               //???????????????????????????^^^
               
               //
/*
               if((drone_pos_vision.x()>=2||drone_pos_vision.x()<=0)||(drone_pos_vision.y()>=0.6||drone_pos_vision.y()<=-0.6)||(drone_pos_vision.z()>=0.6||drone_pos_vision.z()<=-0.6)){
	           msg_drone_pos_vision.pose.orientation.w = -1000;}
               else{
		   msg_drone_pos_vision.pose.orientation.w = 1;}
*/
               //????????????local_position
               //drone_pos_vision.x() = drone_pos_vision.x() + synchronized_att.pose.position.x;
               //drone_pos_vision.x() = drone_pos_vision.y() + synchronized_att.pose.position.y;
               //drone_pos_vision.z() = drone_pos_vision.z() + synchronized_att.pose.position.z;

               msg_drone_pos_vision.header.stamp = ros::Time::now();
               msg_drone_pos_vision.pose.position.x = drone_pos_vision.x();
               msg_drone_pos_vision.pose.position.y = drone_pos_vision.y();
               msg_drone_pos_vision.pose.position.z = drone_pos_vision.z();

               msg_drone_pos_vision.pose.orientation.x = id_name;
               msg_drone_pos_vision.pose.orientation.w = 1;

                // target_pose_world
               msg_target_pose_world.header.stamp = ros::Time::now();
               msg_target_pose_world.pose.position.x = msg_drone_pos_vision.pose.position.x + vision_pose_msg.pose.position.x;
               msg_target_pose_world.pose.position.y = msg_drone_pos_vision.pose.position.y + vision_pose_msg.pose.position.y;
               msg_target_pose_world.pose.position.z = msg_drone_pos_vision.pose.position.z + vision_pose_msg.pose.position.z;


               
            //    cout<<"x is :"<<msg_drone_pos_vision.pose.position.x<<"    "<<"y is :"<<msg_drone_pos_vision.pose.position.y<<"     "<<"z is :"<<msg_drone_pos_vision.pose.position.z<<endl;
            //    cout<<"id is :"<<msg_drone_pos_vision.pose.orientation.x<<endl;
            //    cout<<"????????????:"<<msg_drone_pos_vision.pose.orientation.w<<endl;
         }

        if(msg_drone_pos_vision.pose.position.y>=0.6||msg_drone_pos_vision.pose.position.y<=-0.6||msg_drone_pos_vision.pose.position.z>=0.6||msg_drone_pos_vision.pose.position.z<=-0.6){
               msg_drone_pos_vision.pose.orientation.w = -1000;
        }
        msg_target_pose_from_img_pub.publish(msg_target_pose_from_img);
    // cout<<"x is :"<<msg_drone_pos_vision.pose.position.x<<"    "<<"y is :"<<msg_drone_pos_vision.pose.position.y<<"     "<<"z is :"<<msg_drone_pos_vision.pose.position.z<<endl;
    // cout<<"id is :"<<msg_drone_pos_vision.pose.orientation.x<<endl;
    // cout<<"????????????:"<<msg_drone_pos_vision.pose.orientation.w<<endl;
	drone_pos_vision_pub.publish(msg_drone_pos_vision);     
    target_pose_world_pub.publish(msg_target_pose_world);   
        rate.sleep();
    }
    return 0;
}

void plane_attitude_sub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    plane_atti_msg = *msg;
    // cout << "msg->pose.orientation.z: " << msg->pose.orientation.z << endl;
    drone_euler_real = quaternion2euler(plane_atti_msg.pose.orientation.x,plane_atti_msg.pose.orientation.y,plane_atti_msg.pose.orientation.z,plane_atti_msg.pose.orientation.w);
    // cout  <<  "drone_euler_real z  =" << drone_euler_real.z << endl; 
    pose_queue.push(*msg);
}

void vision_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vision_pose_msg = *msg;
}


void target_corner_sub(const geometry_msgs::PoseStamped::ConstPtr& msg){

    targetDetectFlag = false;
    if(msg->pose.orientation.x < 0) //?????????????????????
    {

         targetDetectFlag = false;
    }
    else{
        //????????????????????????
        image_ros_time = double(msg->pose.position.x);
        //??????id
        id_name = int(msg->pose.position.z)+1;
        //?????????????????? xmax - xmin
        target_width = int(msg->pose.orientation.z - msg->pose.orientation.x);
        //?????????????????? ymax - ymin
        target_height = int(msg->pose.orientation.w - msg->pose.orientation.y);
        //????????????????????????
        target_left_up.x = int(max(0,int(msg->pose.orientation.x)));  //xmin
        target_left_up.y = int(max(0,int(msg->pose.orientation.y)));  //ymin
        //????????????????????????
        target_right_down.x = int(min(640,int(msg->pose.orientation.z)));  //xmax
        target_right_down.y = int(min(640,int(msg->pose.orientation.w)));  //ymax
        //???????????????????????????
        yolo_center.x = (target_left_up.x + target_right_down.x)/2.0;
        yolo_center.y = (target_left_up.y + target_right_down.y)/2.0;
        //????????????????????????????????????
        msg_target_pose_from_img.header.stamp = ros::Time::now();
        msg_target_pose_from_img.pose.orientation.x = 1;
        msg_target_pose_from_img.pose.position.z = scale_size * target_width_world * fx_color / target_width;
        msg_target_pose_from_img.pose.position.x = msg_target_pose_from_img.pose.position.z * (yolo_center.x-cx_color) / fx_color;
        msg_target_pose_from_img.pose.position.y = msg_target_pose_from_img.pose.position.z * (yolo_center.y-cy_color) / fy_color;

        targetDetectFlag = true;
    }
}

void tf_param_set() {
    //image coordinate to ENU coordinate

    tf_image_to_enu = Eigen::Isometry3d::Identity();
    tf_image_to_enu.matrix() << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    //the camera position of drone, now only translate , not rotation yet
    // tf is based on ENU axis
    // tf_camera_drone[0] = 0.06;
    // tf_camera_drone[1] = -0.05;
    // tf_camera_drone[2] = 0.1;
    tf_camera_drone[0] = 0.0;
    tf_camera_drone[1] = 0.0;
    tf_camera_drone[2] = 0.0;

    Eigen::Vector3d pose_camera_of_drone;
    pose_camera_of_drone.x() = tf_camera_drone[0];
    pose_camera_of_drone.y() = tf_camera_drone[1];
    pose_camera_of_drone.z() = tf_camera_drone[2];

    tf_camera_to_drone = Eigen::Isometry3d::Identity();
    tf_camera_to_drone.matrix() << 1, 0, 0, pose_camera_of_drone.x(),
            0, 1, 0, pose_camera_of_drone.y(),
            0, 0, 1, pose_camera_of_drone.z(),
            0 ,0, 0, 1;
}

void get_init_yaw() {
    //????????????????????????
    drone_euler_init = quaternion2euler(plane_atti_msg.pose.orientation.x,plane_atti_msg.pose.orientation.y,plane_atti_msg.pose.orientation.z,plane_atti_msg.pose.orientation.w);
    if(abs(drone_euler_init.z) > 0.000001)
        got_attitude_init = true;
}

/**
 * ??????????????????????????????
 * @param roll
 * @param pitch
 * @param yaw
 * @return ???????????????
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw){
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw){
    Eigen::Quaterniond temp;
    temp.w() = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x() = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y() = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z() = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * ????????????????????????????????????
 * @param x
 * @param y
 * @param z
 * @param w
 * @return ??????Vector3????????????
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w){
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y = asin(2.0 * (- z * x + w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}
