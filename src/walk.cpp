/*! \file Test
    \brief A test class.

    A more detailed class description.
*/

#include "leg.h"
#include "publish.h"
#include "gait.h"
#include "body.h"
#include "path.h"
#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <termios.h>


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc,char** argv)
{
ros::init(argc, argv, "walk");

ros::NodeHandle node;

float x_start = 0;
float y_start = 0;


//INTIAL COG POSITION IN THE PATH IS SPECIFIED BY x_start AND y_start
GAIT crawl_gait(node,x_start,y_start);
//ros::Duration(2).sleep();
//crawl_gait.moveto(1,0,-0.1,0);

ros::Subscriber jnt_state = node.subscribe ("/joint_states", 100, update_legpos);


ros::Duration(2).sleep();


ROS_INFO("QUADRUPED INITIATED");


crawl_gait.goto_home_pos();

/*
ROS_INFO("QUADRUPED MOVED TO START POSITION");


ros::Duration(2).sleep();

crawl_gait.leg_phasing();

ROS_INFO("LEG PHASING MANOUEVER FOR CRAWL GAIT COMPLETED ");

ros::Duration(2).sleep();

ROS_INFO("STARTING PATH FOLLOWING ALGORITHM");

set_endpoint(2,0);

crawl_gait.follow_2d_path();

ROS_INFO("SUCCESSFULLY REACHED END OF PATH");

*/
ROS_INFO("LEG MANIPULATION STARTED");

while(ros::ok)
{


  int c = getch();
  char k = c;
//  ROS_INFO("%d",c);
    crawl_gait.body_manip(c);


}

return 0;
}
