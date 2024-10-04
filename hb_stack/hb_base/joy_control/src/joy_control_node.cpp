#include "joy_control/joy_control.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "joy_control");
  JoyDecode joy;
  joy.Execute();
  

  return 0;
}
