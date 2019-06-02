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

#define BASELINE 1.30
#define CAR_LENGTH 1.765
#define STEERING_FACTOR 18
#define PI 3.14159265

#define DEFAULT_INIT_POSITION_X 0.0
#define DEFAULT_INIT_POSITION_Y 0.0
#define DEFAULT_INIT_POSITION_THETA 0.0
#define DEFAULT_INIT_VELOCITY_X 0.0
#define DEFAULT_INIT_VELOCITY_Y 0.0
#define DEFAULT_INIT_VELOCITY_ANGULAR 0.0

#define DIFFERENTIAL_MODEL_MODE true
#define ACKERMAN_MODEL_MODE false

#define SPEED_L_TOPIC "speedL_stamped"
#define SPEED_R_TOPIC "speedR_stamped"
#define STEER_TOPIC "steer_stamped"

#define ODOMETRY_TOPIC_ID "odometry"
#define ODOMETRY_FRAME_ID "odometry"
#define CHILD_FRAME_ID "base_link"

typedef struct last_odometry_data {
    double x;
    double y;
    double theta;

    double linear_velocity_x;
    double linear_velocity_y;
    double angular_velocity;
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

void differenrialDriveOdometry(double delta_time, double speed_L, double speed_R, OdometryData& new_odometry_data){
    
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = (speed_R - speed_L) / BASELINE;
    
    double delta_theta = angular_velocity * delta_time;
    
    
    new_odometry_data.angular_velocity = angular_velocity;
    new_odometry_data.theta = last_odometry_data.theta + delta_theta;
    
    
    if (new_odometry_data.theta > 2*PI) {
        new_odometry_data.theta -= 2*PI;
    } else if (new_odometry_data.theta < 0) {
        new_odometry_data.theta += 2*PI;
    }

    if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        new_odometry_data.x = last_odometry_data.x + (linear_velocity / angular_velocity) * (sin(new_odometry_data.theta) - sin(last_odometry_data.theta));
        new_odometry_data.y = last_odometry_data.y - (linear_velocity / angular_velocity) * (cos(new_odometry_data.theta) - cos(last_odometry_data.theta));
    } else {
        new_odometry_data.x = last_odometry_data.x + (new_odometry_data.linear_velocity_x * delta_time);
        new_odometry_data.y = last_odometry_data.y + (new_odometry_data.linear_velocity_y * delta_time);
    }

    new_odometry_data.linear_velocity_x = linear_velocity * cos(last_odometry_data.theta + (delta_theta / 2));
    new_odometry_data.linear_velocity_y = linear_velocity * sin(last_odometry_data.theta + (delta_theta / 2));

}


void ackermanDriveOdometry(double delta_time, double speed_L, double speed_R, double steer, OdometryData& new_odometry_data){
    
    double linear_velocity = (speed_R + speed_L) / 2;
    double angular_velocity = linear_velocity * tan((steer / STEERING_FACTOR) * (PI / 180)) / CAR_LENGTH;
    
    double delta_theta = angular_velocity * delta_time;
    
    new_odometry_data.theta = last_odometry_data.theta + delta_theta;
    
    
    if (new_odometry_data.theta > 2*PI) {
        new_odometry_data.theta -= 2*PI;
    } else if (new_odometry_data.theta < 0) {
        new_odometry_data.theta += 2*PI;
    }


    if (angular_velocity > 0.0001)  //exact integration assuming velocities constamìnt in the time lapse
    {
        new_odometry_data.x = last_odometry_data.x + (linear_velocity / angular_velocity) * (sin(new_odometry_data.theta) - sin(last_odometry_data.theta));
        new_odometry_data.y = last_odometry_data.y - (linear_velocity / angular_velocity) * (cos(new_odometry_data.theta) - cos(last_odometry_data.theta));
    }
    else  //for low values of omega I use the Ruge-Kutta approximation
    {
        new_odometry_data.x = last_odometry_data.x + linear_velocity * delta_time * cos(last_odometry_data.theta + delta_theta / 2);
        new_odometry_data.y = last_odometry_data.y + linear_velocity * delta_time * sin(last_odometry_data.theta + delta_theta / 2);
    }
    new_odometry_data.linear_velocity_x = linear_velocity * cos(new_odometry_data.theta);
    new_odometry_data.linear_velocity_y = linear_velocity * sin(new_odometry_data.theta);
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

void subCallback(const odometry::floatStamped::ConstPtr& left,
                 const odometry::floatStamped::ConstPtr& right, 
                 const odometry::floatStamped::ConstPtr& steer,
                 tf::TransformBroadcaster& broadcaster,
                 ros::Publisher& publisher){

    double time = (left->header.stamp.toSec() + right->header.stamp.toSec() + steer->header.stamp.toSec()) / 3;
    double delta_time = time - last_msg_time;
    last_msg_time = time;

    OdometryData new_odometry_data;
    if (last_config.odometry_model_mode == DIFFERENTIAL_MODEL_MODE) {
        differenrialDriveOdometry(delta_time, left->data, right->data, new_odometry_data);
    } else if (last_config.odometry_model_mode == ACKERMAN_MODEL_MODE) {
        ackermanDriveOdometry(delta_time, left->data, right->data, steer->data, new_odometry_data);
    } else {
        ROS_INFO("!!!!!!!!!!! CRITICAL CONFIG ERROR - ODOMETRY MODEL NOT SET!!!!!!!!!!!");
    }
    

    double deltax = new_odometry_data.x - last_odometry_data.x;
    double deltay = new_odometry_data.y - last_odometry_data.y;
    double speed = (right->data + left->data) / 2;

    last_odometry_data = new_odometry_data;

    ROS_INFO("[\nX COORDINATE: %f\nY COORDINATE: %f\nORIENTATION: %f\ndelta_time: %f\nSPEED: %f\nDELTA: %f   %f", last_odometry_data.x, last_odometry_data.y, last_odometry_data.theta * 360/ (2*PI), delta_time, speed, deltax, deltay);
    
    // publishing and broadcasting
    nav_msgs::Odometry odometry = populateOdometry();
    publisher.publish(odometry);

    geometry_msgs::TransformStamped transform = populateTransform();
    broadcaster.sendTransform(transform);
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