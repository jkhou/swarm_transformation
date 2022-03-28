//
// Created by hjk on 2022/3/27.
//
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <curses.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

//geometry_msgs::PoseStamped commond_rotate;
//geometry_msgs::PoseStamped commond_scale;
//geometry_msgs::PoseStamped commond_trans;

geometry_msgs::PoseStamped uav0_form_pose;
geometry_msgs::PoseStamped uav1_form_pose;
geometry_msgs::PoseStamped uav2_form_pose;
geometry_msgs::PoseStamped uav3_form_pose;
geometry_msgs::PoseStamped uav4_form_pose;

double PI = 3.1415926;

bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "commond_pub_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);

//    // 旋转
//    ros::Publisher commond_rotate_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("affine_commond/rotate", 10);
//    // 缩放
//    ros::Publisher commond_scale_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("affine_commond/scale", 10);
//    // 平移
//    ros::Publisher commond_trans_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("affine_commond/trans", 10);


    // 发布每架飞机的位置
    ros::Publisher uav0_form_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav0/form_pose", 10);

    ros::Publisher uav1_form_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/form_pose", 10);

    ros::Publisher uav2_form_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/form_pose", 10);

    ros::Publisher uav3_form_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/form_pose", 10);

    ros::Publisher uav4_form_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav4/form_pose", 10);

    sleep(1);
    char input;

    Eigen::Matrix<double, 5, 2> r;
    r <<
          2, 0,
          1, 1,
          1,-1,
          -1, 1,
          -1,-1;

    Matrix<double, 5, 2> rNow;

    // 编队的中心
    Eigen::Matrix<double, 1, 2> via = Matrix<double, 1, 2>::Zero();
    double viaX = 0.0;
    double viaY = 0.0;

    // 缩放系数  左上角为长  右下角为宽
    Eigen::Matrix<double, 2 ,2> T1;
    T1 <<
       1.0, 0,
       0, 1.0;
    double T1x = 1.0;
    double T1y = 1.0;

    // 旋转系数
    Eigen::Matrix<double, 2, 2> T2;
    T2 <<
       1,  0,
       0,  1;
    double ang = 0.0;

    while(ros::ok())
    {
//        commond_rotate.header.stamp = ros::Time::now();
//        commond_scale.header.stamp = ros::Time::now();
//        commond_trans.header.stamp = ros::Time::now();

        // 每次开始都进行一次初始化，全部置零
//        commond_trans.pose.orientation.w = 0;
//        commond_trans.pose.orientation.x = 0;
//        commond_trans.pose.orientation.y = 0;
//        commond_trans.pose.orientation.z = 0;
//
//        commond_rotate.pose.position.x = 0;
//        commond_rotate.pose.position.y = 0;
//
//        commond_scale.pose.position.x = 0;
//        commond_scale.pose.position.y = 0;

        // 如果键盘上有操作
        if(kbhit()) {
            input = fgetc(stdin);
            cout << endl;
            // w->w   s->z   a->x    d->y
            if(input == 's') {
//                commond_trans.pose.orientation.z = 1;
                viaY -= 0.03;
                cout << "Get Input : s ! Backward!" << endl;
            }
            else if(input == 'w') {
//                commond_trans.pose.orientation.w = 1;
                viaY += 0.03;
                cout << "Get Input : w ! Foreward!" << endl;
            }
            else if(input == 'a') {
//                commond_trans.pose.orientation.x = 1;
                viaX -= 0.03;
                cout << "Get Input : a ! Moveleft!" << endl;
            }
            else if(input == 'd'){
//                commond_trans.pose.orientation.y = 1;
                viaX += 0.03;
                cout << "Get Input : d ! MoveRight!" << endl;
            }
            else if(input == 'h') {
//                commond_rotate.pose.position.x = 1;
                if(T1x <= 0.25) {
                    T1x = 0.25;
                }
                else {
                    T1x -= 0.02;
                }

                cout << "Get Input : h !" << endl;
            }
            else if(input == 'k') {
//                commond_rotate.pose.position.y = 1;
                T1x += 0.02;
                cout << "Get Input : k !" << endl;
            }
            else if(input == 'u') {
//                commond_scale.pose.position.x = 1;
                T1y += 0.02;
                cout << "Get Input : u !" << endl;
            }
            else if(input == 'j') {
//                commond_scale.pose.position.y = 1;
                if(T1y <= 0.25) {
                    T1y = 0.25;
                }
                else {
                    T1y -= 0.02;
                }
                cout << "Get Input : j !" << endl;
            }
            else if(input == '0') {
                ang -= PI/60.0;
                cout << "Get Input : 0！" << endl;
            }
            else if(input == '9') {
                ang += PI/60.0;
                cout << "Get Input : 9！" << endl;
            }
        }

        via << viaX, viaY;
        T1 << T1x , 0.0,
              0.0, T1y;

        T2 << cos(ang), -sin(ang),
            sin(ang), cos(ang);

        rNow = r*T2.transpose()*T1.transpose();
        for(int i = 0; i < rNow.rows(); i ++) {
            rNow.row(i) += via;
        }

        uav0_form_pose.header.stamp = ros::Time::now();
        uav0_form_pose.pose.position.x = rNow(0,0) - r(0,0);
        uav0_form_pose.pose.position.y = rNow(0,1) - r(0,1);

        uav1_form_pose.header.stamp = ros::Time::now();
        uav1_form_pose.pose.position.x = rNow(1,0) - r(1,0);
        uav1_form_pose.pose.position.y = rNow(1,1) - r(1,1);

        uav2_form_pose.header.stamp = ros::Time::now();
        uav2_form_pose.pose.position.x = rNow(2,0) - r(2,0);
        uav2_form_pose.pose.position.y = rNow(2,1) - r(2,1);

        uav3_form_pose.header.stamp = ros::Time::now();
        uav3_form_pose.pose.position.x = rNow(3,0) - r(3,0);
        uav3_form_pose.pose.position.y = rNow(3,1) - r(3,1);

        uav4_form_pose.header.stamp = ros::Time::now();
        uav4_form_pose.pose.position.x = rNow(4,0) - r(4,0);
        uav4_form_pose.pose.position.y = rNow(4,1) - r(4,1);


        uav0_form_pose_pub.publish(uav0_form_pose);
        uav1_form_pose_pub.publish(uav1_form_pose);
        uav2_form_pose_pub.publish(uav2_form_pose);
        uav3_form_pose_pub.publish(uav3_form_pose);
        uav4_form_pose_pub.publish(uav4_form_pose);


//        commond_rotate_pub.publish(commond_rotate);
//        commond_scale_pub.publish(commond_scale);
//        commond_trans_pub.publish(commond_trans);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


