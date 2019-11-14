#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

geometry_msgs::Twist vel;
geometry_msgs::Twist vel_linear;
geometry_msgs::Twist vel_angular;
void split(const string& s,vector<int>& sv,const char flag = ' ') {
    sv.clear();
    istringstream iss(s);
    string temp;
    while (getline(iss, temp, flag)) {
        sv.push_back(stoi(temp));
    }
    return;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //pixel_distance_x,pixel_distance_y
    vector<int> sv;
    int pixel_distance_x,pixel_distance_y,angularz;

    split(msg, sv, ',');
    pixel_distance_x = sv[0];
    pixel_distance_y = sv[1];

    // linearx
    if (pixel_distance_y > 5)
        vel.linear.x = 0.3;
    else if (pixel_distance_y < -5)
        vel.linear.x = -0.3;
    else 
        vel.linear.x = 0.0;

    //angularz
    if (pixel_distance_x >-5 && pixel_distance_x < 5)
        angularz = 0;
    else 
        angularz = atan2(pixel_distance_y/pixel_distance_x);
    vel.angular.z = angularz;

    cmd_pub.publish(vel);
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"motion");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/pixel_distance",1,chatterCallback);
    ros::Publisher cmd_pub = nhandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher pub_angular = nhandle.advertise<geometry_msgs::Twist>("/pixel_distance/steering", 1);
    ros::Publisher pub_linear = nhandle.advertise<geometry_msgs::Twist>("/pixel_distance/steering", 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        cmd_pub.publish(vel);
        pub_angular.publish(vel);
        pub_linear.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}