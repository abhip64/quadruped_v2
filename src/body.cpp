#include "body.h"

float BODY::body_len = 0;
float BODY::body_width = 0;
float BODY::body_height = 0;

void BODY::init_body(ros::NodeHandle &node)
{
  if((!node.getParam("body_length",body_len))||(!node.getParam("body_width",body_width))||
  (!node.getParam("body_height",body_height)))
  {
    ROS_FATAL("COULD NOT LOAD BODY PARAMETERS FROM THE PARAMETER SERVER");
  }

}

void BODY::set_initial_cog_pos(float x, float y)
{
  x_cog = x;
  y_cog = y;
  alpha = 0;

  roll = 0;
  pitch = 0;
  yaw = 0;

  x_disp = 0;
  y_disp = 0;
  z_disp = 0;

}

float BODY::get_body_len()
{
  return BODY::body_len;
}

float BODY::get_body_width()
{
  return BODY::body_width;
}
