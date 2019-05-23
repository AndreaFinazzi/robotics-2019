#include<math.h>

#include "ros/ros.h"
#include "ros/duration.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/connection.h>
#include <boost/bind.hpp>

#include <dynamic_reconfigure/server.h>
#include <odometry/parametersConfig.h>

//#include "custom_messages/floatStamped.h"
#include "odometry/floatStamped.h"

#define BASELINE 0.130
#define CAR_LENGTH 0.1765
#define STEERING_FACTOR 18
#define PI 3.14159265

#define SPEED_L_TOPIC "speedL_stamped"
#define SPEED_R_TOPIC "speedR_stamped"
#define STEER_TOPIC "steer_stamped"


typedef struct odometry_data{
    double x;
    double y;
    double theta;
}  OdometryData;

typedef struct diff_drive_control_variables{
    double speed_left;
    double speed_right; 
} DiffDriveControlVariables;

void configChangeCallback(odometry::parametersConfig& config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s",
            config.odometry_model_mode?"True":"False");
}

void differenrialDriveOdometry(double dt, double speed_L, double speed_R, OdometryData startingPose, OdometryData* storageStruct){
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = (speed_R - speed_L) / BASELINE;
    storageStruct->theta = startingPose.theta + angular_velocity * dt;
    if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        storageStruct->x = startingPose.x + (linear_velocity / angular_velocity) * (sin(storageStruct->theta * (360/PI)) - sin(startingPose.theta * (360/PI)));
        storageStruct->y = startingPose.y - (linear_velocity / angular_velocity) * (cos(storageStruct->theta * (360/PI)) - sin(startingPose.theta * (360/PI)));
    }
    else  //for low values of omega I use the Ruge-Kutta approximation
    {
        storageStruct->x = startingPose.x + linear_velocity * dt * cos(startingPose.theta + (angular_velocity * dt) / 2);
        storageStruct->y = startingPose.y + linear_velocity * dt * sin(startingPose.theta + (angular_velocity * dt) / 2);
    }
}

void ackermanDriveOdometry(double dt, double speed_L, double speed_R, double steer, OdometryData startingPose, OdometryData* storageStruct){
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = angular_velocity * tan(steer / STEERING_FACTOR) / CAR_LENGTH;
    storageStruct->theta = startingPose.theta + angular_velocity * dt;
    if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        storageStruct->x = startingPose.x + (linear_velocity / angular_velocity) * (sin(storageStruct->theta * (360/PI)) - sin(startingPose.theta * (360/PI)));
        storageStruct->y = startingPose.y - (linear_velocity / angular_velocity) * (cos(storageStruct->theta * (360/PI)) - sin(startingPose.theta * (360/PI)));
    }
    else  //for low values of omega I use the Ruge-Kutta approximation
    {
        storageStruct->x = startingPose.x + linear_velocity * dt * cos(startingPose.theta + (angular_velocity * dt) / 2);
        storageStruct->y = startingPose.y + linear_velocity * dt * sin(startingPose.theta + (angular_velocity * dt) / 2);
    }
}

void subCallback(const odometry::floatStamped::ConstPtr& left,
                 const odometry::floatStamped::ConstPtr& right, 
                 const odometry::floatStamped::ConstPtr& steer){
    
    ROS_INFO("{MESSAGE-NUMBER:: %d}(L: %f4 - R: %f4 - S: %f4)", left->header.seq, left->data, right->data, steer->data);
}


int main(int argc, char *argv[])
{

    //Odometry config variables
    OdometryData odometry;
    odometry.x = 0;
    odometry.y = 0;
    odometry.theta = 0;
    double last_msg_time = 0.0;

    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    // Config server init
    dynamic_reconfigure::Server<odometry::parametersConfig> config_server;
    config_server.setCallback(boost::bind(&configChangeCallback, _1, _2));

    // ### BEGIN ### wheels data subscriber and synchronizer
    std::cout << "Node started...\n";
    message_filters::Subscriber<odometry::floatStamped> left_speed_sub(nh, SPEED_L_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> right_speed_sub(nh, SPEED_R_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> steer_sub(nh, STEER_TOPIC, 1);

    typedef message_filters::sync_policies::ApproximateTime<odometry::floatStamped, odometry::floatStamped, odometry::floatStamped> syncPolicy;

    std::cout << "Subscribers built...\n";
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), left_speed_sub, right_speed_sub, steer_sub);
    std::cout << "Synchronizer started...\n";
    sync.registerCallback(boost::bind(&subCallback, _1, _2, _3));
    std::cout << "Topics synchronized...\n";

    // ### END ###

    ros::spin();
    return 0;
}