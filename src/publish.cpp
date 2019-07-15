#include "publish.h"

void PUB::publish_init(ros::NodeHandle& node)
{

  j1_lf = node.advertise<std_msgs::Float64>("/joint1_lf_controller/command", 1000);
  j2_lf = node.advertise<std_msgs::Float64>("/joint2_lf_controller/command", 1000);
  j3_lf = node.advertise<std_msgs::Float64>("/joint3_lf_controller/command", 1000);

  j1_lr = node.advertise<std_msgs::Float64>("/joint1_lr_controller/command", 1000);
  j2_lr = node.advertise<std_msgs::Float64>("/joint2_lr_controller/command", 1000);
  j3_lr = node.advertise<std_msgs::Float64>("/joint3_lr_controller/command", 1000);

  j1_rf = node.advertise<std_msgs::Float64>("/joint1_rf_controller/command", 1000);
  j2_rf = node.advertise<std_msgs::Float64>("/joint2_rf_controller/command", 1000);
  j3_rf = node.advertise<std_msgs::Float64>("/joint3_rf_controller/command", 1000);

  j1_rr = node.advertise<std_msgs::Float64>("/joint1_rr_controller/command", 1000);
  j2_rr = node.advertise<std_msgs::Float64>("/joint2_rr_controller/command", 1000);
  j3_rr = node.advertise<std_msgs::Float64>("/joint3_rr_controller/command", 1000);

}

void PUB::publish_leg(float* fpos,int index)
{
std_msgs::Float64 leg_pos[3];

leg_pos[0].data = *fpos;
leg_pos[1].data = *(fpos+1);
leg_pos[2].data = *(fpos+2);

switch (index)
{
case 1:
j1_lf.publish(leg_pos[0]);
j2_lf.publish(leg_pos[1]);
j3_lf.publish(leg_pos[2]);
break;

case 2:
j1_rf.publish(leg_pos[0]);
j2_rf.publish(leg_pos[1]);
j3_rf.publish(leg_pos[2]);
break;

case 3:
j1_lr.publish(leg_pos[0]);
j2_lr.publish(leg_pos[1]);
j3_lr.publish(leg_pos[2]);
break;

case 4:
j1_rr.publish(leg_pos[0]);
j2_rr.publish(leg_pos[1]);
j3_rr.publish(leg_pos[2]);
break;

default:
ROS_FATAL("INVALID INDEX FOR LEG PASSED FOR PUBLISHING");
}

}
