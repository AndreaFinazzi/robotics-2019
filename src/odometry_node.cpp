#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/connection.h>
#include <boost/bind.hpp>



//#include "custom_messages/floatStamped.h"
#include "odometry/floatStamped.h"

#include "rosbag/bag.h"


void subCallback(const odometry::floatStamped::ConstPtr& left,
                 const odometry::floatStamped::ConstPtr& right, 
                 const odometry::floatStamped::ConstPtr& steer){
    ROS_INFO("{MESSAGE-NUMBER:: %d}(L: %f4 - R: %f4 - S: %f4)", left->header.seq, left->data, right->data, steer->data);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "od_node");
    ros::NodeHandle nh;
    std::cout << "Node started...\n";
    message_filters::Subscriber<odometry::floatStamped> left_speed_sub(nh, "speedL_stamped", 1);
    message_filters::Subscriber<odometry::floatStamped> right_speed_sub(nh, "speedR_stamped", 1);
    message_filters::Subscriber<odometry::floatStamped> steer_sub(nh, "steer_stamped", 1);

    typedef message_filters::sync_policies::ApproximateTime<odometry::floatStamped, odometry::floatStamped, odometry::floatStamped> syncPolicy;

    std::cout << "Subscribers built...\n";
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), left_speed_sub, right_speed_sub, steer_sub);
    std::cout << "Synchronizer started...\n";
    sync.registerCallback(boost::bind(&subCallback, _1, _2, _3));
    std::cout << "Topics synchronized...\n";


    ros::spin();
    return 0;
}