#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>

#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>


#include <Eigen/Dense>
#include <offb_control/slsStates.h>
#include <offb_control/ActuatorControl0.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

struct sls_state {
    double x, y, z, alpha, beta, roll, pitch, yaw, vx, vy, vz, gamma_alpha, gamma_beta, omega_1, omega_2, omega_3;
    // /mavros/local_position/pose
}sls_state1;

void current_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

mavros_msgs::ActuatorControl actuator_setpoint;
void actuator_cb(const offb_control::ActuatorControl0::ConstPtr& msg){
    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = msg->controls[0];
    actuator_setpoint.controls[1] = msg->controls[1];
    actuator_setpoint.controls[2] = msg->controls[2];
    actuator_setpoint.controls[3] = msg->controls[3];
    actuator_setpoint.controls[4] = msg->controls[4];
    actuator_setpoint.controls[5] = msg->controls[5];
    actuator_setpoint.controls[6] = msg->controls[6];
    actuator_setpoint.controls[7] = msg->controls[7];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Subscriber position_state_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, current_position_cb);
    ros::Subscriber actuator_sub = nh.subscribe<offb_control::ActuatorControl0>("/offb_control/ActuatorControl0", 1000, actuator_cb);
    ros::Publisher actuator_setpoint_pub = nh.advertise<mavros_msgs::ActuatorControl> ("/mavros/actuator_control", 1000);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 200Hz
    ros::Rate rate(1000);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        actuator_setpoint_pub.publish(actuator_setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void current_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    sls_state1.x = msg -> pose.position.x;
    sls_state1.y = msg -> pose.position.y;
    sls_state1.z = msg -> pose.position.z;
    double quatx = msg->pose.orientation.x;
    double quaty = msg->pose.orientation.y;
    double quatz = msg->pose.orientation.z;
    double quatw = msg->pose.orientation.w;
}


