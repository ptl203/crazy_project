#include "ros/ros.h"
#include <iostream>
#include "crazy_msgs/status.h"
#include "std_msgs/Bool.h"

class arbiterNode {
    public:
        bool launch_status, task_status, land_status, ranger_status;
        arbiterNode () 
        {
            launch_status = false;
            task_status = false;
            land_status = false;
            ranger_status = false;
            ros::Rate loop_rate(30);

            ROS_INFO("Create Subscribers");
            //Create Subscribers
            launch_sub = n.subscribe("/launch_ready", 1000, &arbiterNode::launchCallback, this);
            task_sub = n.subscribe("/task_ready", 1000, &arbiterNode::taskCallback, this);
            land_sub = n.subscribe("/land_ready", 1000, &arbiterNode::landCallback, this);
            ranger_sub = n.subscribe("/ranger_ready", 1000, &arbiterNode::rangerCallback, this);
            ROS_INFO("Createe Subscribers");

            ROS_INFO("Creating Publisher");
            //Creating Publisher
            status_pub = n.advertise<crazy_msgs::status>("/system_status", 1000);
            ROS_INFO("Created Publisher");

            while (ros::ok()) {
                crazy_msgs::status statusMsg;
                ROS_INFO("Launch status is %d", launch_status);
                if (launch_status && ranger_status) { // Add && task_status && land_status later
                    statusMsg.status = "READY";
                    status_pub.publish(statusMsg);
                    ROS_INFO("Set system_status to READY"); 
                } else {
                    statusMsg.status = "INITIALIZING";
                    status_pub.publish(statusMsg);
                    ROS_INFO("System is Initializing");
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            ros::spin();
        }

        void launchCallback(const std_msgs::Bool& msg) 
        {
            // Update launch_status variable
            launch_status = msg.data;
            ROS_INFO("Callback launch status is %d", launch_status);
        }
        void taskCallback(const std_msgs::Bool& msg) 
        {
            // Update launch_status variable
            task_status = msg.data;
        }
        void landCallback(const std_msgs::Bool& msg) 
        {
            // Update launch_status variable
            land_status = msg.data;
        }
        void rangerCallback(const std_msgs::Bool& msg) 
        {
            // Update launch_status variable
            ranger_status = msg.data;
        }
    private:
        ros::NodeHandle n;
        ros::Subscriber launch_sub;
        ros::Subscriber task_sub;
        ros::Subscriber land_sub;
        ros::Subscriber ranger_sub;
        ros::Publisher status_pub;
};
int main (int argc, char **argv) {
    ROS_INFO("Initializing node");
    ros::init(argc, argv, "arbiter_node");
    ROS_INFO("Creating arbiterNode object");
    arbiterNode arbiterNode;
    return 0; 
}
