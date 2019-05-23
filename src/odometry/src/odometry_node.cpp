#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "nav_msgs/Odometry.h"

#define SPEED_L_TOPIC "/speedL_stamped"
#define SPEED_R_TOPIC "/speedR_stamped"
#define STEER_TOPIC "/steer_stamped"



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle n;


    /*message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_speedL(n, SPEED_L_TOPIC, 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_speedR(n, SPEED_R_TOPIC, 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_steer(n, STEER_TOPIC, 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs;;;;> CustomSyncPolicy;

    message_filters::Synchronizer<CustomSyncPolicy> sync(CustomSyncPolicy(10), sub_speedL, sub_speedR, sub_steer);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));*/

    ros::spin();

    return 0;
}