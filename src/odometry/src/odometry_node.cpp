#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "nav_msgs/Odometry.h"




bool subCallback() {

}

int int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle n;

    

    return 0;
}