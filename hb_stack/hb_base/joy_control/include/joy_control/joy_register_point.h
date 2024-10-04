#ifndef JOY_REGISTER_POINT_H
#define JOY_REGISTER_POINT_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <stdio.h> // for getchar
#include <math.h>
#include <string>
#include <iostream>

class JoyRegisterPoint {

  private:

    ros::Publisher  _vis_pub;
    ros::Subscriber _joy_sub;
    ros::Subscriber _location;
 
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    void getParams() ;
    void JoyCb(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& joy_msg);
    void PublishJoy();
    void DecodeValues(const sensor_msgs::Joy::ConstPtr& _jmsg);
    int _axes_size, _button_size;
    geometry_msgs::PoseStamped _current_pose;

  public:

    JoyRegisterPoint();
    ~JoyRegisterPoint();
    void Execute();



};
#endif
