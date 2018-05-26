
#include "sarsatest.h"


void linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

}

// Reset World
void resetExp(){
    std_srvs::Empty resetWorldSrv;
   
    if(ros::service::call("/gazebo/reset_simulation", resetWorldSrv)){
      ROS_INFO("Falling:%d finished,Reset World",episode_index); 
    }
    else { 
      ROS_INFO("Failed topic reset the world"); 
    }
}


/**
 * Sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ROS_INFO("Start Simulation!!");

  ros::NodeHandle n;

  //controller
  base_to_lower = n.advertise<std_msgs::Float64>("cat_low_controller/command", 1);
  base_to_upper = n.advertise<std_msgs::Float64>("cat_up_controller/command", 1);

  //get the status
  ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("gazebo/link_states", 1, linkstatesCallback); 

  ros::Rate loop_rate(1);


  for(;episode_index<TARGET_EXP_TIME;episode_index++){
  	while (ros::ok()){
  	  	ROS_INFO("New Episode %d Started!\n",episode_index);
  	  	for(move_index=0;move_index<3;move_index++){
  	  		std_msgs::Float64 msg;
    		msg.data=0.5f;
      		base_to_lower.publish(msg);
        	base_to_upper.publish(msg);
        	ros::spinOnce();
        	loop_rate.sleep();
      		resetExp();
  	  	}
  	  	ROS_INFO("END OF Episode %d \n",episode_index);
  	  }
  }
  
  return 0;
}