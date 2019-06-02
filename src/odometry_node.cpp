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

#include "odometry/floatStamped.h"
#include "odometry/CustomOdometry.h"


// Numeric constants
#define BASELINE 1.30
#define CAR_LENGTH 1.765
#define STEERING_FACTOR 18
#define PI 3.14159265

// Default constants
#define DEFAULT_INIT_POSITION_X 0.0
#define DEFAULT_INIT_POSITION_Y 0.0
#define DEFAULT_INIT_POSITION_THETA 0.0
#define DEFAULT_INIT_VELOCITY_X 0.0
#define DEFAULT_INIT_VELOCITY_Y 0.0
#define DEFAULT_INIT_VELOCITY_ANGULAR 0.0
#define DEFAULT_INIT_VELOCITY_LEFT 0.0
#define DEFAULT_INIT_VELOCITY_RIGHT 0.0
#define DEFAULT_INIT_STEER 0.0

// Configuration constants
#define DIFFERENTIAL_MODEL_MODE true
#define ACKERMANN_MODEL_MODE false

// Topic/Frame names definition
#define SPEED_L_TOPIC "speedL_stamped"
#define SPEED_R_TOPIC "speedR_stamped"
#define STEER_TOPIC "steer_stamped"

#define CUSTOM_ODOMETRY_TOPIC_ID "custom_odometry"
#define ODOMETRY_TOPIC_ID "odometry"
#define ODOMETRY_FRAME_ID "odometry"
#define CHILD_FRAME_ID "base_link"


// Support data structures
typedef struct last_odometry_data {
    double x;
    double y;
    double theta;

    double linear_velocity_x;
    double linear_velocity_y;
    double angular_velocity;
    double steer;
    double velocity_R;
    double velocity_L;
}  OdometryData;

typedef struct diff_drive_control_variables{
    double speed_left;
    double speed_right; 
} DiffDriveControlVariables;

using namespace std;

odometry::parametersConfig last_config;
OdometryData last_odometry_data;
double last_msg_time;

void resetDefaultPosition() {
    last_odometry_data.x = DEFAULT_INIT_POSITION_X;
    last_odometry_data.y = DEFAULT_INIT_POSITION_Y;
}

void configChangeCallback(odometry::parametersConfig& config, uint32_t level) {

    if (config.odometry_model_mode != last_config.odometry_model_mode)
        ROS_INFO("Configuration changed: changing odometry model to: \t%s", config.odometry_model_mode?"Differential":"Ackerman");
        last_config.odometry_model_mode = config.odometry_model_mode;
    if (config.odometry_set_position) {
        ROS_INFO("Configuration changed: setting position to: \t[X: %f, Y: %f]", config.odometry_x_position, config.odometry_y_position);        
        last_odometry_data.x = config.odometry_x_position;
        last_odometry_data.y = config.odometry_y_position;
    } else if (config.odometry_reset_default) {
        ROS_INFO("Configuration changed: setting position to default: \t[X: %f, Y: %f]", DEFAULT_INIT_POSITION_X, DEFAULT_INIT_POSITION_Y);        
        resetDefaultPosition();
    }
}

void differenrialDriveOdometry(double delta_time, double speed_L, double speed_R, double steer, OdometryData& new_odometry_data){


    double linear_velocity = (last_odometry_data.velocity_R + last_odometry_data.velocity_L) / 2;
    double angular_velocity = last_odometry_data.angular_velocity;

    double delta_theta = angular_velocity * delta_time;
    
    new_odometry_data.theta = last_odometry_data.theta + delta_theta;
    
    

    if (new_odometry_data.theta > 2*PI) {
        new_odometry_data.theta -= 2*PI;
    } else if (new_odometry_data.theta < 0) {
        new_odometry_data.theta += 2*PI;
    }

    new_odometry_data.x = last_odometry_data.x + linear_velocity * delta_time * cos(last_odometry_data.theta + delta_theta / 2);
    new_odometry_data.y = last_odometry_data.y + linear_velocity * delta_time * sin(last_odometry_data.theta + delta_theta / 2);


    double new_linear_velocity = (speed_R + speed_L) / 2;

    new_odometry_data.angular_velocity = (speed_R - speed_L) / BASELINE;

    new_odometry_data.linear_velocity_x = new_linear_velocity * cos(new_odometry_data.theta);
    new_odometry_data.linear_velocity_y = new_linear_velocity * sin(new_odometry_data.theta);

    new_odometry_data.steer = steer;
    new_odometry_data.velocity_L = speed_L;
    new_odometry_data.velocity_R = speed_R;


}


void ackermannDriveOdometry(double delta_time, double speed_L, double speed_R, double steer, OdometryData& new_odometry_data){


    double linear_velocity = (last_odometry_data.velocity_R + last_odometry_data.velocity_L) / 2;
    double angular_velocity = last_odometry_data.angular_velocity;

    double delta_theta = angular_velocity * delta_time;
    
    new_odometry_data.theta = last_odometry_data.theta + delta_theta;


    if (new_odometry_data.theta > 2*PI) {
        new_odometry_data.theta -= 2*PI;
    } else if (new_odometry_data.theta < 0) {
        new_odometry_data.theta += 2*PI;
    }
    
    new_odometry_data.x = last_odometry_data.x + linear_velocity * delta_time * cos(last_odometry_data.theta + delta_theta / 2);
    new_odometry_data.y = last_odometry_data.y + linear_velocity * delta_time * sin(last_odometry_data.theta + delta_theta / 2);

    double new_linear_velocity = (speed_R + speed_L) / 2;

    new_odometry_data.angular_velocity = new_linear_velocity * tan((steer / STEERING_FACTOR) * (PI / 180)) / CAR_LENGTH;

    new_odometry_data.linear_velocity_x = new_linear_velocity * cos(new_odometry_data.theta);
    new_odometry_data.linear_velocity_y = new_linear_velocity * sin(new_odometry_data.theta);

    new_odometry_data.steer = steer;
    new_odometry_data.velocity_L = speed_L;
    new_odometry_data.velocity_R = speed_R;



}

nav_msgs::Odometry populateOdometry() {
    

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = ODOMETRY_FRAME_ID;

    //TODO: these should be set with calculated values
    // position
    odometry.pose.pose.position.x = last_odometry_data.x;
    odometry.pose.pose.position.y = last_odometry_data.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(last_odometry_data.theta);

    //set velocity
    odometry.child_frame_id = CHILD_FRAME_ID;
    odometry.twist.twist.linear.x = last_odometry_data.linear_velocity_x;
    odometry.twist.twist.linear.y = last_odometry_data.linear_velocity_y;
    odometry.twist.twist.angular.z = last_odometry_data.angular_velocity;

    return odometry;
}

geometry_msgs::TransformStamped populateTransform() {
    geometry_msgs::TransformStamped odometry_transform;
    odometry_transform.header.stamp = ros::Time::now();
    odometry_transform.header.frame_id = ODOMETRY_FRAME_ID;
    odometry_transform.child_frame_id = CHILD_FRAME_ID;

    //TODO: these should be set with calculated values
    odometry_transform.transform.translation.x = last_odometry_data.x;
    odometry_transform.transform.translation.y = last_odometry_data.y;
    odometry_transform.transform.translation.z = 0.0;
    odometry_transform.transform.rotation = tf::createQuaternionMsgFromYaw(last_odometry_data.theta);

    return odometry_transform;
}

odometry::CustomOdometry populateCustomOdometry(nav_msgs::Odometry odometry, string odometryModel) {
    
    odometry::CustomOdometry customOdometry;
    customOdometry.odometry = odometry;

    customOdometry.odometryMode.data = odometryModel;

    return customOdometry;

}

void subCallback(const odometry::floatStamped::ConstPtr& left,
                 const odometry::floatStamped::ConstPtr& right, 
                 const odometry::floatStamped::ConstPtr& steer,
                 tf::TransformBroadcaster& broadcaster,
                 ros::Publisher& publisherOdometry,
                 ros::Publisher& publisherCustomOdometry){

    string odometryModel;
    double time = (left->header.stamp.toSec() + right->header.stamp.toSec() + steer->header.stamp.toSec()) / 3;
    double delta_time = time - last_msg_time;
    last_msg_time = time;

    OdometryData new_odometry_data;
    if (last_config.odometry_model_mode == DIFFERENTIAL_MODEL_MODE) {
        differenrialDriveOdometry(delta_time, left->data, right->data, steer->data, new_odometry_data);
        odometryModel = "Differential Drive";
    } else if (last_config.odometry_model_mode == ACKERMANN_MODEL_MODE) {
        ackermannDriveOdometry(delta_time, left->data, right->data, steer->data, new_odometry_data);
        odometryModel = "Ackermann Drive";
    } else {
        ROS_INFO("!!!!!!!!!!! CRITICAL CONFIG ERROR - ODOMETRY MODEL NOT SET!!!!!!!!!!!\n\n");
    }

    last_odometry_data = new_odometry_data;

    
    // publishing and broadcasting
    nav_msgs::Odometry odometry = populateOdometry();
    publisherOdometry.publish(odometry);

    geometry_msgs::TransformStamped transform = populateTransform();
    broadcaster.sendTransform(transform);

    odometry::CustomOdometry customOdometry = populateCustomOdometry(odometry, odometryModel);
    publisherCustomOdometry.publish(customOdometry);

}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    // init support structure
    resetDefaultPosition();
    last_odometry_data.theta = DEFAULT_INIT_POSITION_THETA;
    last_odometry_data.linear_velocity_x = DEFAULT_INIT_VELOCITY_X;
    last_odometry_data.linear_velocity_y = DEFAULT_INIT_VELOCITY_Y;
    last_odometry_data.angular_velocity = DEFAULT_INIT_VELOCITY_ANGULAR;
    last_odometry_data.velocity_L = DEFAULT_INIT_VELOCITY_LEFT;
    last_odometry_data.velocity_R = DEFAULT_INIT_VELOCITY_RIGHT;
    last_odometry_data.steer = DEFAULT_INIT_STEER;
    last_msg_time = ros::Time::now().toSec();
    ROS_INFO("SUPPORT DATA STRUCTURES INITIALIZED\n");

    // Config server init
    dynamic_reconfigure::Server<odometry::parametersConfig> config_server;
    config_server.setCallback(boost::bind(&configChangeCallback, _1, _2));

    // ### BEGIN ### output data publisher and broadcaster
    tf::TransformBroadcaster odometry_broadcaster;
    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>(ODOMETRY_TOPIC_ID, 50);
    ros::Publisher custom_odometry_publisher = nh.advertise<odometry::CustomOdometry>(CUSTOM_ODOMETRY_TOPIC_ID ,50);

    ROS_INFO("NODE STARTED\n");

    // ### BEGIN ### wheels data subscriber and synchronizer
    message_filters::Subscriber<odometry::floatStamped> left_speed_sub(nh, SPEED_L_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> right_speed_sub(nh, SPEED_R_TOPIC, 1);
    message_filters::Subscriber<odometry::floatStamped> steer_sub(nh, STEER_TOPIC, 1);

    typedef message_filters::sync_policies::ApproximateTime<odometry::floatStamped, odometry::floatStamped, odometry::floatStamped> syncPolicy;

    ROS_INFO("SUBSCRIBER BUILT\n");
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), left_speed_sub, right_speed_sub, steer_sub);
    ROS_INFO("SYNCHRONIZER STARTED\n");
    sync.registerCallback(boost::bind(&subCallback, _1, _2, _3, odometry_broadcaster, odometry_publisher, custom_odometry_publisher));
    ROS_INFO("TOPICS SYNCHRONIZED\n");
    // ### END ###

    ROS_INFO("NODE RUNNING.....\n");
    ros::spin();
    return 0;
}