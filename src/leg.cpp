#include "leg.h"

float LEG::l1_len = 0;
float LEG::l2_len = 0;
float LEG::l3_len = 0;

void LEG::init_leg(ros::NodeHandle &node)
{
  if((!node.getParam("link1_length",LEG::l1_len))||
  (!node.getParam("link2_length",LEG::l2_len))||
  (!node.getParam("link3_length",LEG::l3_len)))
  {
    ROS_FATAL("COULD NOT LOAD LEG PARAMETERS FROM THE PARAMETER SERVER. PLEASE CHECK WHETHER THE PARAMETRS HAVE BEEN DEFINED IN A .yaml FILE");
  }

}


float* LEG::convert_to_leg_frame(float *des_end_pos,int &index,float Rx,float Rz)
{

float x_ = *(des_end_pos);
float y_ = *(des_end_pos + 1);
float z_ = *(des_end_pos + 2);

/////////////////////////////////////////////////////////////////////////////
//IMPORTANT -
//THE ROTATION MATRICES WHICH HAVE BEEN USED BELOW ARE CALCULATED FOR
//FRAMES OF REFERENCES WHOSE Z-AXIS POINTS DOWNWARD. SO CW AND CCW
//ANGLES SEEN FROM TOP ARE INTERCHANGED.
/////////////////////////////////////////////////////////////////////////////

  switch(index)
  {
  case 1:
  result[0] = ( x_ - y_ + Rx)/pow(2,0.5);
  result[1] = ( x_ + y_)/pow(2,0.5);
  result[2] = z_ + Rz;
  break;

  case 2:
  result[0] = ( x_ + y_ + Rx)/pow(2,0.5);
  result[1] = (-x_ + y_)/pow(2,0.5);
  result[2] = z_ + Rz;
  break;

  case 3:
  result[0] = (-x_ - y_ + Rx)/pow(2,0.5);
  result[1] = ( x_ - y_)/pow(2,0.5);
  result[2] = z_ + Rz;
  break;

  case 4:
  result[0] = (-x_ + y_ + Rx)/pow(2,0.5);
  result[1] = (-x_ - y_)/pow(2,0.5);
  result[2] = z_ + Rz;
  break;
  default:
  ROS_FATAL("INVALID LEG INDEX PASSED FOR LEG IK CALCULATION");

}

return result;
}

float* LEG::ik_calc_leg(float *des_end_pos)
{

  float x = *(des_end_pos);
  float y = *(des_end_pos + 1);
  float z = -*(des_end_pos + 2);

  result[0] = atan2(y,x);

  float x_ = l1_len*cos(result[0]);
  float y_ = l1_len*sin(result[0]);

  float hf = sqrt((x_-x)*(x_-x) + (y_-y)*(y_-y) + z*z);

  float k = (hf*hf - l2_len*l2_len - l3_len*l3_len)/(2*l2_len*l3_len);

  if(k<1.01&&k>-1.01)
  {
    if(k>1)
    k=1;
    else if (k<-1)
    k = -1;

    result[2] =  -acos(k);
    result[1] =  atan2(z,sqrt(x*x + y*y)-l1_len) - atan2(l3_len*sin(result[2]),l2_len+l3_len*cos(result[2]));

    result[2] += M_PI/2;

    result[1] = -result[1];
    result[2] = -result[2];

    return result;

  }
  else
  {
  }
}


bool LEG::get_leg_state()
{
  return leg_state;
}

void LEG::update_leg_state()
{
  leg_state = !leg_state;
}

int LEG::get_leg_substate()
{
  return leg_substate;
}

void LEG::set_leg_substate(int state)
{
  leg_substate = state;
}

float LEG::get_link1_len()
{
  return LEG::l1_len;
}

float LEG::get_link2_len()
{
  return LEG::l2_len;
}

float LEG::get_link3_len()
{
  return LEG::l3_len;
}
