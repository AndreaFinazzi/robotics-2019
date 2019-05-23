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
 
 // Output-related libraries
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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


using namespace std;
OdometryData odom;
double last_msg_time;

void configChangeCallback(odometry::parametersConfig& config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s",
            config.odometry_model_mode?"True":"False");
}




void differenrialDriveOdometry(double dt, double speed_L, double speed_R, OdometryData startingPose, OdometryData* storageStruct){
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = (speed_R - speed_L) / BASELINE;
    storageStruct->theta = startingPose.theta + angular_velocity * dt;
    if (storageStruct->theta > 2*PI)
    {
        storageStruct->theta -= 2*PI;
    }else if (storageStruct->theta < 0){
        storageStruct->theta += 2*PI;
    }
    
    /*  if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        storageStruct->x = startingPose.x + (linear_velocity / angular_velocity) * (sin(storageStruct->theta * (180/PI)) - sin(startingPose.theta * (180/PI)));
        storageStruct->y = startingPose.y - (linear_velocity / angular_velocity) * (cos(storageStruct->theta * (180/PI)) - sin(startingPose.theta * (180/PI)));
    }*/
 //   else  //for low values of omega I use the Ruge-Kutta approximation
 //   {
        storageStruct->x = startingPose.x + linear_velocity * dt * cos(startingPose.theta + (angular_velocity * dt) / 2);
        storageStruct->y = startingPose.y + linear_velocity * dt * sin(startingPose.theta + (angular_velocity * dt) / 2);
//    }
}

void ackermanDriveOdometry(double dt, double speed_L, double speed_R, double steer, OdometryData startingPose, OdometryData* storageStruct){
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = linear_velocity * tan(steer / STEERING_FACTOR) / CAR_LENGTH;
    storageStruct->theta = startingPose.theta + angular_velocity * dt;
    if (storageStruct->theta > 2*PI)
    {
        storageStruct->theta -= 2*PI;
    }else if (storageStruct->theta < 0){
        storageStruct->theta += 2*PI;
    }
  /*  if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        storageStruct->x = startingPose.x + (linear_velocity / angular_velocity) * (sin(storageStruct->theta * (180/PI)) - sin(startingPose.theta * (180/PI)));
        storageStruct->y = startingPose.y - (linear_velocity / angular_velocity) * (cos(storageStruct->theta * (180/PI)) - sin(startingPose.theta * (180/PI)));
    }*/
 //   else  //for low values of omega I use the Ruge-Kutta approximation
 //   {
        storageStruct->x = startingPose.x + linear_velocity * dt * cos(startingPose.theta + (angular_velocity * dt) / 2);
        storageStruct->y = startingPose.y + linear_velocity * dt * sin(startingPose.theta + (angular_velocity * dt) / 2);
//    }
}

nav_msgs::Odometry populateOdometry() {
    
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "odometry";

    //TODO: these should be set with calculated values
    // position
    odometry.pose.pose.position.x = 1;
    odometry.pose.pose.position.y = 2;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(123.4);

    //set velocity
    odometry.child_frame_id = "base_link";
    odometry.twist.twist.linear.x = 0.5;
    odometry.twist.twist.linear.y = 0.5;
    odometry.twist.twist.angular.z = 123.4;

    return odometry;
}

geometry_msgs::TransformStamped populateTransform() {
    geometry_msgs::TransformStamped odometry_transform;
    odometry_transform.header.stamp = ros::Time::now();
    odometry_transform.header.frame_id = "odometry";
    odometry_transform.child_frame_id = "base_link";

    //TODO: these should be set with calculated values
    odometry_transform.transform.translation.x = 1;
    odometry_transform.transform.translation.y = 2;
    odometry_transform.transform.translation.z = 0.0;
    odometry_transform.transform.rotation = tf::createQuaternionMsgFromYaw(123.4);

    return odometry_transform;
}

void subCallback(const odometry::floatStamped::ConstPtr& left,
                 const odometry::floatStamped::ConstPtr& right, 
                 const odometry::floatStamped::ConstPtr& steer,
                 tf::TransformBroadcaster& broadcaster,
                 ros::Publisher& publisher){
    double time = (left->header.stamp.toSec() + right->header.stamp.toSec() + steer->header.stamp.toSec()) / 3;
    double dt = time - last_msg_time;
    last_msg_time = time;
    OdometryData storageStruct;
    differenrialDriveOdometry(dt, left->data, right->data, odom, &storageStruct);
    double deltax = storageStruct.x-odom.x;
    double deltay = storageStruct.y-odom.y;
    double speed = (right->data + left->data) / 2; 
    odom = storageStruct;

    ROS_INFO("[\nX COORDINATE: %f\nY COORDINATE: %f\nORIENTATION: %f\ndt: %f\nSPEED: %f\nDELTA: %f   %f", odom.x, odom.y, odom.theta * 360/ (2*PI), dt, speed, deltax, deltay);
    
    // publishing and broadcasting
    nav_msgs::Odometry odometry = populateOdometry();
    publisher.publish(odometry);

    geometry_msgs::TransformStamped transform = populateTransform();
    broadcaster.sendTransform(transform);
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

    // ### BEGIN ### output data publisher and broadcaster
    tf::TransformBroadcaster odometry_broadcaster;
    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odometry", 50);

    // ### BEGIN ### wheels data subscriber and synchronizer
    std::cout << "Node started...\n";
    message_filters::Subscriber<odometry::floatStamped> left_speed_sub(nh, SPEED_L_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> right_speed_sub(nh, SPEED_R_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> steer_sub(nh, STEER_TOPIC, 1);

    typedef message_filters::sync_policies::ApproximateTime<odometry::floatStamped, odometry::floatStamped, odometry::floatStamped> syncPolicy;

    std::cout << "Subscribers built...\n";
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), left_speed_sub, right_speed_sub, steer_sub);
    std::cout << "Synchronizer started...\n";
    sync.registerCallback(boost::bind(&subCallback, _1, _2, _3, odometry_broadcaster, odometry_publisher));
    std::cout << "Topics synchronized...\n";
    // ### END ###

    ros::spin();
    return 0;
}