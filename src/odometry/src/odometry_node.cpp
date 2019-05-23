#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "nav_msgs/Odometry.h"
#include "odometry/floatStamped.h"

#define SPEED_L_TOPIC "/speedL_stamped"
#define SPEED_R_TOPIC "/speedR_stamped"
#define STEER_TOPIC "/steer_stamped"

using namespace message_filters;

class Odometry
{
public:
    Odometry() {

        message_filters::Subscriber<odometry::floatStamped> sub_speedL(node, SPEED_L_TOPIC, 1);
        message_filters::Subscriber<odometry::floatStamped> sub_speedR(node, SPEED_R_TOPIC, 1);
        message_filters::Subscriber<odometry::floatStamped> sub_steer(node, STEER_TOPIC, 1);

        typedef message_filters::sync_policies::ApproximateTime<odometry::floatStamped, odometry::floatStamped, odometry::floatStamped> CustomSyncPolicy;

        message_filters::Synchronizer<CustomSyncPolicy> sync(CustomSyncPolicy(10), sub_speedL, sub_speedR);
        sync.registerCallback(boost::bind(&Odometry::callback, this, _1, _2, _3));

    };
    
    // virtual ~Odometry();

    bool callback(const odometry::floatStampedConstPtr& speedL, const odometry::floatStampedConstPtr& speedR, const odometry::floatStampedConstPtr& steer) {
      ROS_INFO ("Received messages: (%f,%f,%f)", speedL->data, speedR->data, steer->data);
    }

private:
    ros::NodeHandle node;
    /* data */
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_node");

    Odometry odometry();

    ros::spin();

    return 0;
}