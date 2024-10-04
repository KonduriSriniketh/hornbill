#include "joy_control/joy_register_point.h"


void JoyRegisterPoint::JoyCb(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  _axes_size = joy_msg->axes.size();
  _button_size = joy_msg->buttons.size();
  int _button_values[_button_size-1];
  double _axes_values[_axes_size-1];

  if (_axes_size == 8 &&_button_size == 11){
    _mode_normal = false;
    _mode_xinput = true;

  }
  else {
    _mode_normal = true;
    _mode_xinput = false;
  }
  if (_mode_xinput){
    if (joy_msg->buttons[7] == 1){
      _start_flag = true;
      ROS_INFO("Control sequence initiated...");
    }

    DecodeValues(joy_msg);
  }
  else {
    ROS_WARN("Change the mode to xinput!");
  }
}


JoyDecode::JoyDecode(): nh_private("~")
{

  getParams();


  _cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(_cmd_vel, 10);
  _joy_sub = nh.subscribe(_joy, 1 , &JoyDecode::JoyCb, this);


}

JoyDecode::~JoyDecode(){

}

void JoyDecode::DecodeValues(const sensor_msgs::Joy::ConstPtr& _jmsg){

  if (!_reset_flag) {
    if (_jmsg->axes[7] == 1.0 ){
      if (_fd_spd < 1.0) {
        _fd_spd += _speed_factor;
      }
      else { ROS_INFO("Maximun Speed reached!!");}
    }
    if (_jmsg->axes[7] == -1.0){
      if (-1.0 < _fd_spd && _fd_spd  > 0 ) {
        _fd_spd -= _speed_factor;
      }
      else {
        _fd_spd = 0.0;
        ROS_INFO("Speed is zero!!");}
    }
    if (_jmsg->axes[6] == 1.0 ){
      if (_rt_spd < 1.0) {
      _rt_spd += _speed_factor;
      }
      else { ROS_INFO("Maximun Speed reached!!");}
    }
    if (_jmsg->axes[6] == -1.0){
      if (-1.0 < _rt_spd && _rt_spd > 0) {
        _rt_spd -= _speed_factor;
      }
      else {
        _rt_spd = 0.0;
        ROS_INFO("Speed is zero!!");}
    }
    if (_jmsg->axes[1] > 0 || _jmsg->axes[1] < 0 ){
      cmd_vel_msg.linear.x = _jmsg->axes[1]*abs(_fd_spd);
    }

    if (_jmsg->axes[0] > 0 || _jmsg->axes[0] < 0 ){
      cmd_vel_msg.angular.z = _jmsg->axes[0]*abs(_rt_spd);
    }
    if(_jmsg->buttons[1] == 1){
      _fd_spd = 0.0;
      _rt_spd = 0.0;
      ROS_INFO("Speed reset successfull!");
    }

    _reset_flag = true;
    PublishJoy();
  }
  if (_reset_flag){
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
    _reset_flag = false;
  }
}
void JoyDecode::PublishJoy() {
  if (_start_flag){
    _cmd_vel_pub.publish(cmd_vel_msg);
    ROS_INFO("Current speed :- linear %f   Angular %f", _fd_spd, _rt_spd);
  }
  else {ROS_INFO("Press Start Button to begin the JoyStick control");}

}
void JoyDecode::getParams(){

  nh_private.param("speed_factor"   ,  _speed_factor, double(0.025));
  nh_private.param("reset_flag"     ,  _reset_flag  ,  false);
  nh_private.param("start_flag"     ,  _start_flag  ,  false);
  nh_private.param("mode_xinput"     ,  _mode_xinput  ,  false);
  nh_private.param("mode_normal"    ,  _mode_normal ,  false);
  nh_private.param("loop_rate"      ,  _loop_rate   ,  int(10));
  nh_private.param("cmd_vel_topic"  ,  _cmd_vel     ,  std::string("cmd_vel"));
  nh_private.param("joy_topic"      ,  _joy         ,   std::string("joy"));

}


void JoyDecode::Execute(){
  ros::Rate loop_rate(_loop_rate);

  while	(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

}
