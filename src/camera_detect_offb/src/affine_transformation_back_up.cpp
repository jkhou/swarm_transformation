//
// Created by hjk on 2022/2/24.
//
// px4 offboard node for testing consensus

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <vector>
#include <ros/package.h>
#include <csignal>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <minimum_jerk/minimum_jerk_traj.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

// global variable
mavros_msgs::State uav_cur_state;
geometry_msgs::PoseStamped uav_cur_pose;
// 其他无人机的位置
geometry_msgs::PoseStamped uav0_cur_pose;
geometry_msgs::PoseStamped uav1_cur_pose;
geometry_msgs::PoseStamped uav2_cur_pose;
geometry_msgs::PoseStamped uav3_cur_pose;
geometry_msgs::PoseStamped uav4_cur_pose;

geometry_msgs::PoseStamped target_cur_pose;
trajectory_msgs::JointTrajectoryPoint uav_t_init;
trajectory_msgs::JointTrajectoryPoint uav_t_times;
geometry_msgs::PoseStamped start_move_time;


float uav_init_x = 0;
float uav_init_y = 0;
double PI = 3.1415926;

int ctrl_rate = 30;
int aruco_num = 0;
bool repeat_path = false;

int swarm_ID = 0;
std::string uav_name = "/uav"+to_string(swarm_ID);

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

std::vector<std::vector<double>> read_csv_traj(const std::string &file)
{
    std::vector<std::vector<double>> csv_traj;

    FILE *fp;
    fp=fopen(file.c_str(),"r");
    if(!fp)
    {
        ROS_ERROR_STREAM("cannot open file: " + file);
    }
    while(1){
        std::vector<double> traj_point(3);
        fscanf(fp,"%lf,%lf,%lf",&traj_point[0],&traj_point[1],&traj_point[2]);
        csv_traj.push_back(traj_point);
        if(feof(fp)) break;
    }
    fclose(fp);
    return csv_traj;
}

void loadRosParams(ros::NodeHandle &nh) {
    readParam<int>(nh, "swarm_ID", swarm_ID);
    readParam<int>(nh, "ctrl_rate", ctrl_rate);
//    readParam<float>(nh,"uav_init_x",uav_init_x);
//    readParam<float>(nh,"uav_init_y",uav_init_y);
    uav_name = "/uav" + to_string(swarm_ID);
    uav_cur_pose.pose.orientation.w = 1;
//    uav_cur_pose.pose.position.x = uav_init_x;
//    uav_cur_pose.pose.position.y = uav_init_y;
    //print params
    ROS_INFO_STREAM("swarm_ID = " << swarm_ID);
    ROS_INFO_STREAM("ctrl_rate = " << ctrl_rate);
//    ROS_INFO_STREAM("uav_init_pos = "<<uav_init_x<<","<<uav_init_y);
}


// CB function
void uav_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav_cur_state = *msg;
}

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav_cur_pose = *msg;
}

// 获取其他无人机的位置
void uav0_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav0_cur_pose = *msg;
}

void uav1_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_cur_pose = *msg;
}

void uav2_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_cur_pose = *msg;
}

void uav3_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav3_cur_pose = *msg;
}

void uav4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav4_cur_pose = *msg;
}


void jerk_T_initial_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) {
    if(swarm_ID == 0)
        return;

    uav_t_init = *msg;
}

void jerk_times_initial_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) {
    if(swarm_ID == 0)
        return;

    uav_t_times = *msg;
}

void start_move_time_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if(swarm_ID == 0)
        return;

    start_move_time = *msg;
}

int main (int argc, char** argv) {
    // ros init
    ros::init(argc, argv, "affine_transformation_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // min_jerk init
    std::string TABLE_PATH="/home/hjk/Downloads/p3-2_v3_a4_res0-1.csv";
    Minimum_Jerk_Traj traj;
    traj.init(TABLE_PATH,ctrl_rate,1,0.2);

    double dt = 1.0/ctrl_rate;

    // load param
    loadRosParams(nh);

    //ros pub and sub
    ros::Publisher jerk_T_initial_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>
            ("/jerk_T_initial", 10);

    ros::Subscriber jerk_T_initial_sub = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>
            ("/jerk_T_initial", 10, jerk_T_initial_cb);


    ros::Publisher jerk_times_initial_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>
            ("/jerk_times_initial", 10);

    ros::Subscriber jerk_times_initial_sub = nh.subscribe<trajectory_msgs::JointTrajectoryPoint>
            ("/jerk_times_initial", 10, jerk_times_initial_cb);


    ros::Publisher start_move_time_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/off_board_move_time", 10);

    ros::Subscriber start_move_time_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/off_board_move_time", 10, start_move_time_cb);


    ros::Subscriber uav_state_sub = nh.subscribe<mavros_msgs::State>
            (uav_name+"/mavros/state", 10, uav_state_cb);

    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (uav_name+"/mavros/local_position/pose", 2, uav_pose_cb);

    // 订阅其他飞机的位置
    ros::Subscriber uav0_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 2, uav0_pose_cb);

    ros::Subscriber uav1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 2, uav1_pose_cb);

    ros::Subscriber uav2_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 2, uav2_pose_cb);

    ros::Subscriber uav3_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 2, uav3_pose_cb);

    ros::Subscriber uav4_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 2, uav4_pose_cb);



    ros::Publisher offb_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            (uav_name+"/mavros/setpoint_position/local", 10);

    ros::Publisher jerk_pva_p_pub = nh.advertise<geometry_msgs::Vector3>
            (uav_name + "/jerk_pva_set_p", 10);

    ros::Publisher jerk_pva_v_pub = nh.advertise<geometry_msgs::Vector3>
            (uav_name + "/jerk_pva_set_v", 10);

    ros::Publisher jerk_pva_a_pub = nh.advertise<geometry_msgs::Vector3>
            (uav_name + "/jerk_pva_set_a", 10);

    ros::Publisher take_off_pva_pub = nh.advertise<geometry_msgs::Vector3>
            (uav_name + "/take_off_pva", 10);

    ros::Publisher att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            (uav_name + "/mavros/setpoint_raw/attitude", 10);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (uav_name+"/mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (uav_name+"/mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate ctrl_loop(ctrl_rate);



    // wait for FCU connection
    while(ros::ok() && !uav_cur_state.connected){
        ros::spinOnce();
        ctrl_loop.sleep();
    }

    // 关联矩阵
    Eigen::Matrix<double, 5, 10> D;
    D <<
         1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
        -1, 0, 0, 0, 1, 1, 1, 0, 0, 0,
         0,-1, 0, 0,-1, 0, 0, 1, 1, 0,
         0, 0,-1, 0, 0,-1, 0,-1, 0, 1,
         0, 0, 0,-1, 0, 0,-1, 0,-1,-1;

    // 关联矩阵行数
    int n = D.rows();

    // 关联矩阵列数
    int m = D.cols();

    // 无人机飞行高度
    double height = 1.5;

    int d = 2;

    // 矩阵H为矩阵D的转置矩阵
    Eigen::Matrix<double, 10, 5> H = D.transpose();

    // 初始时刻的期望位置
    Eigen::Matrix<double, 5, 2> r;
    r <<
         2, 0,
         1, 1,
         1,-1,
        -1, 1,
        -1,-1;

    // r的增广矩阵P
    Eigen::Matrix<double, 5, 3> P;
    P <<
         2, 0, 1,
         1, 1, 1,
         1,-1, 1,
        -1, 1, 1,
        -1,-1, 1;

    // 边矩阵
    Eigen::Matrix<double, 2, 10> edge;
    edge <<
            1, 1, 1, 1, 2, 2, 2, 3, 3, 4,
            2, 3, 4, 5, 3, 4, 5, 4, 5, 5;

    // 编队中心点经过的坐标
    Eigen::Matrix<double, 7, 2> via;
    via <<
            0, 0,
            10, 0,
            20, 0,
            20, -10,
            20, -20,
            10, -20,
            0, -20;
//    via <<
//            0, 0,
//            0, 0,
//            0, 0,
//            0, 0,
//            0, 0,
//            0, 0,
//            0, 0;

    // 缩放系数
    Eigen::Matrix<double, 2 ,2> T1;
    T1 <<
            1.0, 0,
            0, 0.6;

    // 旋转系数
    Eigen::Matrix<double, 2, 2> T2;
    T2 <<
            1,  0,
            0,  1;

    // 根据编队位置以及缩放和旋转系数，生成编队位置处每个点的位置
    // 第一个位置编队
    Eigen::Matrix<double, 5, 2> ra1;
    ra1 = r;
    for(int i = 0; i < ra1.rows(); i ++) {
        ra1.row(i) += via.row(0);
    }

    // 第二个位置编队
    Eigen::Matrix<double, 5, 2> ra2;
    ra2 = r*T2.transpose()*T1.transpose();
    for(int i = 0; i < ra2.rows(); i ++) {
        ra2.row(i) += via.row(1);
    }

    // 第三个位置编队
    Eigen::Matrix<double, 5, 2> ra3;
    T2 <<
            cos(-PI/2),  -sin(-PI/2),
            sin(-PI/2),  cos(-PI/2);
    T1 <<
            1.5,   0,
            0,   1.5;
    ra3 = r*T2.transpose()*T1.transpose();;
    for(int i = 0; i < ra3.rows(); i ++) {
        ra3.row(i) += via.row(2);
    }

    // 第四个位置编队
    Eigen::Matrix<double, 5, 2> ra4;
    T2 <<
            cos(-PI/2),  -sin(-PI/2),
            sin(-PI/2),  cos(-PI/2);
    T1 <<
            0.7,   0,
            0,   0.7;
    ra4 = r*T2.transpose()*T1.transpose();;
    for(int i = 0; i < ra4.rows(); i ++) {
        ra4.row(i) += via.row(3);
    }

    // 第五个位置编队
    Eigen::Matrix<double, 5, 2> ra5;
    T2 <<
            cos(-PI) ,  -sin(-PI),
            sin(-PI),   cos(-PI);
    T1 <<
            1,   0,
            0,   1;
    ra5 = r*T2.transpose()*T1.transpose();;
    for(int i = 0; i < ra5.rows(); i ++) {
        ra5.row(i) += via.row(4);
    }

    // 第六个位置编队
    Eigen::Matrix<double, 5, 2> ra6;
    T2 <<
            cos(-PI) ,  -sin(-PI),
            sin(-PI),   cos(-PI);
    T1 <<
            0.5,   0,
            0,   1;
    ra6 = r*T2.transpose()*T1.transpose();;
    for(int i = 0; i < ra1.rows(); i ++) {
        ra6.row(i) += via.row(5);
    }

    // 第七个位置编队
    Eigen::Matrix<double, 5, 2> ra7;
    T2 <<
            cos(-PI*3/2) ,  -sin(-PI*3/2),
            sin(-PI*3/2),   cos(-PI*3/2);
    T1 <<
            1,   0,
            0,   1;
    ra7 = r*T2.transpose()*T1.transpose();;
    for(int i = 0; i < ra7.rows(); i ++) {
        ra7.row(i) += via.row(6);
    }

    // 编队期望位置
    Eigen::Matrix<double, 5, 2> xr = r;

    // 应力矩阵E
    Eigen::Matrix<double ,15, 10> E;
    for(int i = 0; i < n; i ++) {
        // 由H矩阵构造对角阵temp
        Eigen::Matrix<double, 10, 10> diagH;
        diagH = Eigen::Matrix<double, 10, 10>::Zero();

        for(int j = 0; j < 10; j ++) {
            diagH(j, j) = H(j, i);
        }

        // 构造应力矩阵E = [E; p'*H'*diag(H(:,i))]
        E.block(i*3, 0, 3, 10) = P.transpose()*H.transpose()*diagH;
    }

    // z矩阵是E的零空间 null space
    Eigen::FullPivLU<Eigen::MatrixXd> lu(E);
//    Eigen::MatrixXd z = lu.kernel();
    Matrix<double, 10, 3> z;
    z << -0.2160, -0.6393, 0.2008,
        -0.4093, 0.0241, 0.5723,
        0.0076, 0.4342, 0.0569,
        0.2009, -0.2291, -0.3146,
        -0.1389, 0.2248, -0.4738,
        -0.3549, -0.4144, -0.2730,
        0.2469, 0.0948, 0.3734,
        0.3436, -0.2369, 0.1876,
        -0.5482, 0.2489, 0.0985,
        -0.3473, 0.0198, -0.2161;



    // Omega
    Eigen::Matrix<double, 5, 5> Omega;
    Eigen::Matrix<double, 10, 10> diagz;
    diagz =  Eigen::Matrix<double, 10, 10>::Zero();

    for(int i = 0; i < 10; i ++) {
        diagz(i, i) = z(i,0);
    }
    Omega = H.transpose()*diagz*H;

    // gamma
    Eigen::Matrix<double, 5, 1> gamma;
    for(int i = 0; i < 5; i ++) {
        gamma(i, 0) = Omega(i, i);
    }

    // 位置 - 初始位置即是第一个队形的位置
    Eigen::Matrix<double, 5, 2> x;
    x = ra1;

    // 速度
    Eigen::Matrix<double, 5 ,2> dx;
    dx = Eigen::Matrix<double, 5, 2>::Zero();

    // 加速度
    Eigen::Matrix<double, 5, 2> ddx;
    ddx = Eigen::Matrix<double, 5, 2>::Zero();

    // 设置输入点的矩阵
    vector<Eigen::Matrix<double, 9, 1>> minJerkInput;
    Eigen::Matrix<double, 9, 1> point1;
    if(swarm_ID == 0) {
        point1 << ra1(0,0),ra1(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point1 << ra1(1,0),ra1(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point1 << ra1(2,0),ra1(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point1 << ra1(3,0),ra1(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point1 << ra1(4,0),ra1(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point1);


    Eigen::Matrix<double, 9, 1> point2;
    if(swarm_ID == 0) {
        point2 << ra2(0,0),ra2(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point2 << ra2(1,0),ra2(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point2 << ra2(2,0),ra2(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point2 << ra2(3,0),ra2(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point2 << ra2(4,0),ra2(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point2);


    Eigen::Matrix<double, 9, 1> point3;
    if(swarm_ID == 0) {
        point3 << ra3(0,0),ra3(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point3 << ra3(1,0),ra3(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point3 << ra3(2,0),ra3(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point3 << ra3(3,0),ra3(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point3 << ra3(4,0),ra3(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point3);


    Eigen::Matrix<double, 9, 1> point4;
    if(swarm_ID == 0) {
        point4 << ra4(0,0),ra4(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point4 << ra4(1,0),ra4(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point4 << ra4(2,0),ra4(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point4 << ra4(3,0),ra4(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point4 << ra4(4,0),ra4(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point4);


    Eigen::Matrix<double, 9, 1> point5;
    if(swarm_ID == 0) {
        point5 << ra5(0,0),ra5(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point5 << ra5(1,0),ra5(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point5 << ra5(2,0),ra5(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point5 << ra5(3,0),ra5(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point5 << ra5(4,0),ra5(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point5);


    Eigen::Matrix<double, 9, 1> point6;
    if(swarm_ID == 0) {
        point6 << ra6(0,0),ra6(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point6 << ra6(1,0),ra6(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point6 << ra6(2,0),ra6(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point6 << ra6(3,0),ra6(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point6 << ra6(4,0),ra6(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point6);


    Eigen::Matrix<double, 9, 1> point7;
    if(swarm_ID == 0) {
        point7 << ra7(0,0),ra7(0,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 1) {
        point7 << ra7(1,0),ra7(1,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 2) {
        point7 << ra7(2,0),ra7(2,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 3) {
        point7 << ra7(3,0),ra7(3,1),height,0,0,0,0,0,0;
    }
    else if(swarm_ID == 4) {
        point7 << ra7(4,0),ra7(4,1),height,0,0,0,0,0,0;
    }
    minJerkInput.push_back(point7);

    // min_jerk计算中间点的pva
    traj.set_waypoint(minJerkInput);

    // min_jerk每一段运算时间
//    vector<double> t_init;

//    // 接收min_jerk计算结果
//    vector<Eigen::Matrix<double, 9, 1>> minJerkResult;
//    minJerkResult = traj.generate_minimum_jerk_bisection(swarm_ID, uav_t_init);


//    if(swarm_ID == 4) {
//        std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point.csv";
//        traj.write_traj_point_csv(CSV_PATH);
//    }

    // Send a few setpoints before starting
    // Before entering Offboard mode, you must have already started streaming setpoints.
    // Otherwise the mode switch will be rejected.
    geometry_msgs::PoseStamped pose_before_offb;
    for(int i = 10; ros::ok() && i > 0; --i){
        pose_before_offb = uav_cur_pose;
        offb_setpoint_pub.publish(pose_before_offb);
        ros::spinOnce();
        ctrl_loop.sleep();
    }

    bool takeoff = false;
    bool just_offb = true;//check whether plane change into offboard mode from other mode
    geometry_msgs::PoseStamped pose_offb_hover;//pose to hover in offboard mode
    pose_offb_hover = uav_cur_pose;
    int traj_index = 0;

    geometry_msgs::Vector3 jerk_to_pva_P;
    geometry_msgs::Vector3 jerk_to_pva_V;
    geometry_msgs::Vector3 jerk_to_pva_A;

    jerk_to_pva_P.x = ra1(0, 0);
    jerk_to_pva_P.y = ra1(0, 1);
    jerk_to_pva_P.z = height;

    geometry_msgs::Vector3 take_off;
    take_off.x = 0;
    bool transformation_fin = false;

    // 两个控制参数
    double kp = 5.0;
    double kv = 5.0;

    // 姿态指令值
    mavros_msgs::AttitudeTarget uav_setpoint_raw;

    // main ctrl loop
    while(ros::ok() && flag) {
        watchSignal();
        bool get_t_init_data = true;

        // 接收min_jerk计算结果
        vector<Eigen::Matrix<double, 9, 1>> minJerkResult;
        if(swarm_ID == 0) {
            minJerkResult = traj.generate_minimum_jerk_bisection(swarm_ID, uav_t_init, uav_t_times);
//            if(swarm_ID == 4) {
//                std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point0.csv";
//                traj.write_traj_point_csv(CSV_PATH);
//            }
        }
        else {
            if(uav_t_init.positions.empty()) {
                get_t_init_data = false;
            }
            else {
                get_t_init_data = true;
                minJerkResult = traj.generate_minimum_jerk_bisection(swarm_ID, uav_t_init, uav_t_times);

//                if(swarm_ID == 1) {
//                    std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point1.csv";
//                    traj.write_traj_point_csv(CSV_PATH);
//                }
//
//                if(swarm_ID == 2) {
//                    std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point2.csv";
//                    traj.write_traj_point_csv(CSV_PATH);
//                }
//
//                if(swarm_ID == 3) {
//                    std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point3.csv";
//                    traj.write_traj_point_csv(CSV_PATH);
//                }
//
//                if(swarm_ID == 4) {
//                    std::string CSV_PATH="/home/hjk/Desktop/minimium_jerk_traj_point4.csv";
//                    traj.write_traj_point_csv(CSV_PATH);
//                }
            }
        }


//        if(swarm_ID != 0 && uav_t_init.positions.empty()) {
//            get_t_init_data = false;
//        }

        // 解锁或已计算出jerk数据
        if (uav_cur_state.armed && get_t_init_data) {
            watchSignal();
            if (uav_cur_state.mode == "OFFBOARD") {
                watchSignal();
                bool get_start_move_time = true;
                bool get_start_move_time_follower = true;
                double off_board_start = ros::Time::now().toSec();

                if(swarm_ID == 0 && get_start_move_time) {
                    off_board_start += 8.0;
                    start_move_time.header.stamp.sec = off_board_start;
                    get_start_move_time = false;
                }

                if(swarm_ID != 0 && get_start_move_time_follower) {
                    while(start_move_time.header.stamp.isZero()) {
                        ros::spinOnce();
//                        off_board_start = start_move_time.header.stamp.toSec();
                    }
                    off_board_start = start_move_time.header.stamp.toSec();
                    get_start_move_time_follower = false;
                }
//                else {
//                if(swarm_ID != 0) {
//                    if (start_move_time.header.stamp.isZero()) {
//                        get_start_move_time = false;
//                    } else {
//                        off_board_start = start_move_time.header.stamp.toSec();
//                    }
//                }
//                }

                // 如果获取了出发时间
//                if(get_start_move_time) {
                if (!takeoff) {
                    watchSignal();
                    //take off to some height
                    geometry_msgs::PoseStamped takeoff_pose;
                    takeoff_pose.pose.position.x = uav_cur_pose.pose.position.x;
                    takeoff_pose.pose.position.y = uav_cur_pose.pose.position.y;
                    takeoff_pose.pose.position.z = 1.5;
                    double err = 0.05;
                    while (fabs(takeoff_pose.pose.position.z - uav_cur_pose.pose.position.z) > err) {
                        if(swarm_ID == 0) {
                            jerk_T_initial_pub.publish(uav_t_init);
                            start_move_time_pub.publish(start_move_time);
                        }
                        else {
                            off_board_start = start_move_time.header.stamp.toSec();
                        }

                        offb_setpoint_pub.publish(takeoff_pose);
                        ros::spinOnce();
                        ctrl_loop.sleep();
                    }

                    ROS_INFO_STREAM(uav_name + " take off done");
                    takeoff = true;
                    pose_before_offb = takeoff_pose;
                    pose_offb_hover = takeoff_pose;
                }

                if (just_offb) {
                    watchSignal();
                    // into offboard mode just now, stay still for a while, about several seconds
                    for (int i = 1000000; ros::ok() && i > 0; --i) {
                        offb_setpoint_pub.publish(pose_before_offb);
                        double off_board_exce = ros::Time::now().toSec();
                        if (off_board_exce >= off_board_start)
                            break;
                        ros::spinOnce();
                        ctrl_loop.sleep();
                    }
                    just_offb = false;
                    pose_offb_hover = pose_before_offb;
                }

                // 起飞准备工作完成
                take_off.x = 1;

                // has take off, into offboard control
                ROS_INFO_STREAM_THROTTLE(1, uav_name + " in Offboard cmd");

                if (transformation_fin == false) {
                    Eigen::Matrix<double, 5, 2> prex;

                    for (int i = 0; i < minJerkResult.size() && uav_cur_state.mode == "OFFBOARD"; i++) {
                        watchSignal();
//                        prex <<
//                            uav0_cur_pose.pose.position.x, uav0_cur_pose.pose.position.y,
//                            uav1_cur_pose.pose.position.x, uav1_cur_pose.pose.position.y,
//                            uav2_cur_pose.pose.position.x, uav2_cur_pose.pose.position.y,
//                            uav3_cur_pose.pose.position.x, uav3_cur_pose.pose.position.y,
//                            uav4_cur_pose.pose.position.x, uav4_cur_pose.pose.position.y;
                        prex = x;

                        // set P
                        if (swarm_ID == 0) {
                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(0, 0);
                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(0, 1);
                            jerk_to_pva_P.z = minJerkResult[i](2, 0);
                        } else if (swarm_ID == 1) {
                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(1, 0);
                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(1, 1);
                            jerk_to_pva_P.z = minJerkResult[i](2, 0);
                        } else if (swarm_ID == 2) {
                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(2, 0);
                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(2, 1);
                            jerk_to_pva_P.z = minJerkResult[i](2, 0);
                        } else if (swarm_ID == 3) {
                            // 当前各无人机在全局坐标系下的位置
                            x(0, 0) = uav0_cur_pose.pose.position.x + ra1(0,0);
                            x(0,1) = uav0_cur_pose.pose.position.y + ra1(0,1);

                            x(1, 0) = uav1_cur_pose.pose.position.x + ra1(1,0);
                            x(1,1) = uav1_cur_pose.pose.position.y + ra1(1,1);

                            x(2, 0) = uav2_cur_pose.pose.position.x + ra1(2,0);
                            x(2,1) = uav2_cur_pose.pose.position.y + ra1(2,1);

//                            x(3, 0) = uav3_cur_pose.pose.position.x + ra1(3,0);
//                            x(3,1) = uav3_cur_pose.pose.position.y + ra1(3,1);

                            x(3, 0) = minJerkResult[i](0, 0);
                            x(3,1) = minJerkResult[i](1, 0);

//                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(2, 0);
//                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(2, 1);

                            x(4, 0) = uav4_cur_pose.pose.position.x + ra1(4,0);
                            x(4,1) = uav4_cur_pose.pose.position.y + ra1(4,1);

                            Eigen::Matrix<double, 1 ,2> err_sum = Eigen::Matrix<double, 1, 2>::Zero();
                            vector<int> edge_ind;
                            for(int k = 0; k < D.cols(); k ++) {
                                if(D(3, k) != 0) {
                                    edge_ind.push_back(k);
                                }
                            }

                            for(auto& k : edge_ind) {
                                int j = 0;
                                for(int t = 0; t < D.rows(); t ++) {
                                    if(D(t, k) != 0 && t != swarm_ID) {
                                        j = t;
                                        break;
                                    }
                                }

                                dx.row(3) = (x.row(3) - prex.row(3))/dt;
                                dx.row(j) = (x.row(j) - prex.row(j))/dt;
//                                err_sum = err_sum + z(k,0)*(kp*(x.row(3) - x.row(j)) + kv*(dx.row(3) - dx.row(j)) - ddx.row(j));
                                err_sum = err_sum + z(k,0)*(kp*(x.row(3) - x.row(j)) + kv*(dx.row(3) - dx.row(j)) - ddx.row(j));
                            }

                            ddx.row(3) = -1.0/gamma(3)*err_sum;
                            x.row(3) = x.row(3) + dx.row(3)*dt + 0.5*ddx.row(3)*dt*dt;
                            jerk_to_pva_P.x = x(3,0) - ra1(3,0);
                            jerk_to_pva_P.y = x(3,1) - ra1(3,1);
                            jerk_to_pva_P.z = minJerkResult[i](2, 0);

                            /*
                            // 把加速度转换为姿态角
                            Vector3d a_des;
                            a_des << ddx(3,0), ddx(3,1), 9.8;
                            Vector3d att_des_norm = a_des / a_des.norm();
                            Vector3d z_w_norm(0, 0, 1.0);
                            Quaterniond att_des_q = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

                            //add yaw
                            double planned_yaw = 0.0;
                            Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
                                                 att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
                            att_des_q = yaw_quat * att_des_q;

                            //Calculate thrust
                            double thrust_factor = 0.06;
                            double thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

                            uav_setpoint_raw.header.stamp = ros::Time::now();
                            uav_setpoint_raw.orientation.w = att_des_q.w();
                            uav_setpoint_raw.orientation.x = att_des_q.x();
                            uav_setpoint_raw.orientation.y = att_des_q.y();
                            uav_setpoint_raw.orientation.z = att_des_q.z();
                            uav_setpoint_raw.thrust = thrust_des;*/

//                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(3, 0);
//                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(3, 1);
//                            jerk_to_pva_P.z = minJerkResult[i](2, 0);

                        } else if (swarm_ID == 4) {
                            // 当前各无人机在全局坐标系下的位置
                            x(0, 0) = uav0_cur_pose.pose.position.x + ra1(0,0);
                            x(0,1) = uav0_cur_pose.pose.position.y + ra1(0,1);

                            x(1, 0) = uav1_cur_pose.pose.position.x + ra1(1,0);
                            x(1,1) = uav1_cur_pose.pose.position.y + ra1(1,1);

                            x(2, 0) = uav2_cur_pose.pose.position.x + ra1(2,0);
                            x(2,1) = uav2_cur_pose.pose.position.y + ra1(2,1);

                            x(3, 0) = uav3_cur_pose.pose.position.x + ra1(3,0);
                            x(3,1) = uav3_cur_pose.pose.position.y + ra1(3,1);

                            x(4, 0) = uav4_cur_pose.pose.position.x + ra1(4,0);
                            x(4,1) = uav4_cur_pose.pose.position.y + ra1(4,1);

                            x(4, 0) = minJerkResult[i](0, 0);
                            x(4,1) = minJerkResult[i](1, 0);

                            Eigen::Matrix<double, 1 ,2> err_sum = Eigen::Matrix<double, 1, 2>::Zero();
                            vector<int> edge_ind;
                            for(int k = 0; k < D.cols(); k ++) {
                                if(D(4, k) != 0) {
                                    edge_ind.push_back(k);   // k = 3 6 8 9
                                }
                            }

                            for(auto& k : edge_ind) {   // k = 3 6 8 9
                                int j = 0;
                                for(int t = 0; t < D.rows(); t ++) {
                                    if(D(t, k) != 0 && t != 4) {
                                        j = t;
                                        break;
                                    }
                                }

                                dx.row(4) = (x.row(4) - prex.row(4))/dt;
                                dx.row(j) = (x.row(j) - prex.row(j))/dt;
                                err_sum = err_sum + z(k,0)*(kp*(x.row(4) - x.row(j)) + kv*(dx.row(4) - dx.row(j)) - ddx.row(j));
                            }

                            ddx.row(4) = -1.0/gamma(4,0)*err_sum;

                            x.row(4) = x.row(4) + dx.row(4)*dt + 0.5*ddx.row(4)*dt*dt;
                            jerk_to_pva_P.x = x(4,0) - ra1(4,0);
                            jerk_to_pva_P.y = x(4,1) - ra1(4,1);
                            jerk_to_pva_P.z = minJerkResult[i](2, 0);

                            /*
                            // 把加速度转换为姿态角
                            Vector3d a_des;
                            a_des << ddx(4,0), ddx(4,1), 9.8;
                            Vector3d att_des_norm = a_des / a_des.norm();
                            Vector3d z_w_norm(0, 0, 1.0);
                            Quaterniond att_des_q = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

                            // add yaw
                            double planned_yaw = 0.0;
                            Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
                                                 att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
                            att_des_q = yaw_quat * att_des_q;

                            // Calculate thrust
                            double thrust_factor = 0.06;
                            double thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

                            uav_setpoint_raw.header.stamp = ros::Time::now();
                            uav_setpoint_raw.orientation.w = att_des_q.w();
                            uav_setpoint_raw.orientation.x = att_des_q.x();
                            uav_setpoint_raw.orientation.y = att_des_q.y();
                            uav_setpoint_raw.orientation.z = att_des_q.z();
                            uav_setpoint_raw.thrust = thrust_des;*/
//                            jerk_to_pva_P.x = minJerkResult[i](0, 0) - ra1(4, 0);
//                            jerk_to_pva_P.y = minJerkResult[i](1, 0) - ra1(4, 1);
//                            jerk_to_pva_P.z = minJerkResult[i](2, 0);
                        }

                        prex = x;

//                        if(swarm_ID == 0 || swarm_ID == 1 || swarm_ID == 2) {
                            // set V
                            jerk_to_pva_V.x = minJerkResult[i](3, 0);
                            jerk_to_pva_V.y = minJerkResult[i](4, 0);
                            jerk_to_pva_V.z = minJerkResult[i](5, 0);

                            // set A
                            jerk_to_pva_A.x = minJerkResult[i](6, 0);
                            jerk_to_pva_A.y = minJerkResult[i](7, 0);
                            jerk_to_pva_A.z = minJerkResult[i](8, 0);

                            jerk_pva_p_pub.publish(jerk_to_pva_P);
                            jerk_pva_v_pub.publish(jerk_to_pva_V);
                            jerk_pva_a_pub.publish(jerk_to_pva_A);
//                        }
//                        else {
//                            att_ctrl_pub.publish(uav_setpoint_raw);
//                        }


                        take_off_pva_pub.publish(take_off);
                        ctrl_loop.sleep();
                        ros::spinOnce();
//                        ROS_INFO_STREAM_THROTTLE(1, uav_name + " in Affine Transformation");
                    }
                    transformation_fin = true;
                } else {
//                    if(swarm_ID == 0 || swarm_ID == 1 || swarm_ID == 2) {
                        jerk_pva_p_pub.publish(jerk_to_pva_P);
                        jerk_pva_v_pub.publish(jerk_to_pva_V);
                        jerk_pva_a_pub.publish(jerk_to_pva_A);
//                    }
//                    else {
//                        att_ctrl_pub.publish(uav_setpoint_raw);
//                    }

                    take_off_pva_pub.publish(take_off);
                }

            }
            else{ // not offb
                ROS_INFO_STREAM_THROTTLE(1, uav_name+" waiting for Offboard cmd");
                offb_setpoint_pub.publish(uav_cur_pose);
                take_off_pva_pub.publish(take_off);
                pose_before_offb = uav_cur_pose;
                just_offb = true;
            }
        }
        else{// not arm
            ROS_INFO_STREAM_THROTTLE(1, uav_name+" waiting for Vehicle arm");
            offb_setpoint_pub.publish(uav_cur_pose);
            take_off_pva_pub.publish(take_off);
            pose_before_offb = uav_cur_pose;
            takeoff=false;
            just_offb = true;
        }

        // 如果是0号无人机，就一直发jerk的T结果
        if(swarm_ID == 0) {
            jerk_T_initial_pub.publish(uav_t_init);
            jerk_times_initial_pub.publish(uav_t_times);
            start_move_time_pub.publish(start_move_time);
        }

        take_off_pva_pub.publish(take_off);
        ros::spinOnce();
        ctrl_loop.sleep();
    }

    return 0;
}


