#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <fpga_ros_integration/fpga_data.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


geometry_msgs::Point current_ref;
bool isArmed = false;
const float vel_multiplier[3] = {1, 1.5, 2};
float vel = 1.5;
int vel_index = 1;
bool doReset = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void serial_cb(const fpga_ros_integration::fpga_data::ConstPtr& msg){

	if(msg -> action_trigger){
		switch(msg -> action_mux){
			case 0:{
				isArmed = not(isArmed);
				std_msgs::String msg;
				std::stringstream ss;
				ss << "Toggling arm state: " << isArmed;
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());
				break;}
			case 1:{
				vel_index++;
				vel = vel_multiplier[vel_index % 3];

				std_msgs::String msg;
				std::stringstream ss;
				ss << "New velocity: " << vel; 
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());
				break;}
			case 2:{
				std_msgs::String msg;
				std::stringstream ss;
				ss << "Doing grab";
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());
				break;}
			case 3:{
				doReset = true;
					
				std_msgs::String msg;
				std::stringstream ss;
				ss << "Resetting";
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());
				break;}
		}
	}

	if(isArmed){
		current_ref.x = msg -> differential.x * vel;
		current_ref.y = msg -> differential.y * 0.2;
		current_ref.z = msg -> differential.z * vel;
	}else{
		current_ref.x = 0;
		current_ref.y = 0;
		current_ref.z = 0;
	}
}

int main(int argc, char **argv){
    ROS_INFO("Logic node online");
    ROS_INFO("Listening to serial node...");
    ros::init(argc, argv, "logic_node");
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber serial_sub = nh.subscribe<fpga_ros_integration::fpga_data>
	    ("fpga/serial_data", 100, serial_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //Setpoints
    ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Point>
	    ("uav/ref", 100);
    ros::Publisher reset_pub = nh.advertise<std_msgs::Bool>
	    ("fpga/reset", 100);

    //Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
           ("mavros/set_mode");


    ros::Rate rate(20.0);

    //wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Mode change
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    //Arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
	if(isArmed){
		if( current_state.mode != "OFFBOARD" &&
		    (ros::Time::now() - last_request > ros::Duration(5.0))){
		    if( set_mode_client.call(offb_set_mode) &&
			offb_set_mode.response.mode_sent){
			ROS_INFO("Changed to offboard");
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
	}else{
		if( current_state.mode != "AUTO.LAND" &&
        	    (ros::Time::now() - last_request > ros::Duration(5.0))){
        	    if( set_mode_client.call(land_set_mode) &&
                	land_set_mode.response.mode_sent){
	                ROS_INFO("Changed to land");
        	    }
	            last_request = ros::Time::now();
        	}
	}

	ctrl_pub.publish(current_ref);

	if(doReset){
		std_msgs::Bool pub_bool;
		pub_bool.data = true;
		reset_pub.publish(pub_bool);
		rate.sleep();
		pub_bool.data = false;
		reset_pub.publish(pub_bool);
		doReset = false;
	}

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
