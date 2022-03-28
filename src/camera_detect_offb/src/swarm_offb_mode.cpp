#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main (int argc, char** argv) 
{   
    ros::init(argc, argv, "swarm_offb_mode_node");

    ros::NodeHandle nh("~"); 

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client_0 = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
	ros::ServiceClient set_mode_client_1 = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::ServiceClient set_mode_client_2 = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    ros::ServiceClient set_mode_client_3 = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    ros::ServiceClient set_mode_client_4 = nh.serviceClient<mavros_msgs::SetMode>("/uav4/mavros/set_mode");

    ros::ServiceClient arming_client_0 = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    ros::ServiceClient arming_client_1 = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient arming_client_2 = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient arming_client_3 = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient arming_client_4 = nh.serviceClient<mavros_msgs::CommandBool>("/uav4/mavros/cmd/arming");

    mavros_msgs::CommandBool armed_cmd;
    armed_cmd.request.value = true;
    arming_client_0.call(armed_cmd);
    arming_client_1.call(armed_cmd);
    arming_client_2.call(armed_cmd);
    arming_client_3.call(armed_cmd);
    arming_client_4.call(armed_cmd);
    sleep(2);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
	ros::Time last_request = ros::Time::now();

	while(ros::ok())
	{
        set_mode_client_0.call(offb_set_mode);
		set_mode_client_1.call(offb_set_mode);
        set_mode_client_2.call(offb_set_mode);
        set_mode_client_3.call(offb_set_mode);
        set_mode_client_4.call(offb_set_mode);

		
		// if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(1.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } 

        ros::spinOnce();
        loop_rate.sleep();
    }
}
