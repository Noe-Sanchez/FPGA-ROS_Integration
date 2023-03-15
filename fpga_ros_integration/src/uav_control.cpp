#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::PositionTarget current_ref;
void ref_cb(const geometry_msgs::Point::ConstPtr& msg){
	current_ref.velocity.x = msg -> x;
	current_ref.velocity.z = msg -> z;
	current_ref.yaw_rate = msg -> y;	
}

int main(int argc, char **argv){
    ROS_INFO("Control node online");
    ROS_INFO("Listening to logic node...");
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    //Subscribers
    //ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //        ("mavros/state", 10, state_cb);
    ros::Subscriber ref_sub = nh.subscribe<geometry_msgs::Point>
            ("uav/ref", 100, ref_cb);

    //Setpoints
    ros::Publisher ctrl_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 100);

    //Services
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //        ("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //       ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    //Contact services
    //Mode change
    //mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";

    //Arming
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    current_ref.position.x = 0;
    current_ref.position.y = 0;
    current_ref.position.z = 0;

    current_ref.velocity.y = 0;
    current_ref.yaw = 0;

    current_ref.acceleration_or_force.x = 0;
    current_ref.acceleration_or_force.y = 0;
    current_ref.acceleration_or_force.z = 0;

    current_ref.type_mask = 1991;
    //current_ref.type_mask = 2040;
    current_ref.header.frame_id = "uav_ref";
    current_ref.coordinate_frame = 8;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        /*if( current_state.mode != "OFFBOARD" &&
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
        }*/

	current_ref.header.stamp = ros::Time::now();
        ctrl_pub.publish(current_ref);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
