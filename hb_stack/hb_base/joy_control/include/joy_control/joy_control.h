

#ifndef JOY_CONTROL_H
#define JOY_CONTROL_H


#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <stdio.h> // for getchar
#include <math.h>
#include <string>
#include <iostream>

class JoyDecode {

  private:

    ros::Publisher  _cmd_vel_pub;
    ros::Subscriber _joy_sub;
 
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    void getParams() ;
    void JoyCb(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void PublishJoy();
    void DecodeValues(const sensor_msgs::Joy::ConstPtr& _jmsg);
    int _axes_size, _button_size;
    geometry_msgs::Twist cmd_vel_msg;

    std::string _cmd_vel;
    std::string _joy;

    int j=0,k=0;
    bool _start_flag;
    bool _mode_xinput;
    bool _mode_normal;
    bool _reset_flag;
    double _speed_factor;
    double _fd_spd = 0.0;
    double _rt_spd = 0.0;
    int _loop_rate;



  public:

    JoyDecode();
    ~JoyDecode();
    void Execute();



};
#endif
