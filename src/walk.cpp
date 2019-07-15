/** @file
* \brief Initiates the primary node "walk"
*
* This file initiates the node "walk" which starts the quadruped path
* tracking algorithm. The path tracking is achieved using periodic
* crab-turning gait. Once the path is traversed, in-place body manipulations
* can be carried out using the keyboard(by typing into the terminal).
* Detailed instructions for carrying out body manipulations can be
* found in the file @body_manip.cpp
*/

///////////////////////////////////////////////////////////////////////////////////////

#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <termios.h>
#include "leg.h"
#include "publish.h"
#include "gait.h"
#include "body.h"
#include "path.h"

///////////////////////////////////////////////////////////////////////////////////////
/**
*Code to take keyboard input from the terminal once the path tracking is completed.\n
*For more info check out -  \n
*https://answers.ros.org/question/63491/keyboard-key-pressed/
*/

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
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
  * \brief Main function
  *
  * Calls all the necessary functions for initialization, path tracking
  * and body manipulations.
 */
int main(int argc,char** argv)
{
  ros::init(argc, argv, "walk");

  ros::NodeHandle node;

  //Initial x-position of the COG of the quadruped. Note that
  //it should lie on the path
  float x_start = 0;

  //Initial y-position of the COG of the quadruped. Note that
  //it should lie on the path
  float y_start = 0;

  //Initialise GAIT class with the node-handle and starting coordinates
  //of the quadruped
  GAIT crawl_gait(node,x_start,y_start);

  ROS_INFO("QUADRUPED INITIATED");

  //Move the quadruped to the home position. Home position of the joints
  //are defined in @leg.h
  crawl_gait.goto_home_pos();

  ROS_INFO("QUADRUPED MOVED TO HOME POSITION");

  //Wait for 2 seconds
  ros::Duration(2).sleep();

  //Carry out the leg phasing manouever to start path tracking. Leg phasing
  //is done to ensure enough stability margin exists before starting locomotion.
  //It is done when the quadruped is not on ground to ensure it doesnt topple
  //when the first leg is lifted
  crawl_gait.leg_phasing();

  ROS_INFO("LEG PHASING MANOUEVER FOR CRAWL GAIT COMPLETED ");

  //Wait for 2 seconds
  ros::Duration(2).sleep();

  ROS_INFO("STARTING PATH FOLLOWING ALGORITHM");

  //End coordinates of the COG of the quadruped that it needs to
  //achieve on the path
  set_endpoint(2,0);

  //Starting the path tracking algorithm. Path of the quadruped is
  //defined in @path.cpp
  crawl_gait.follow_2d_path();

  ROS_INFO("SUCCESSFULLY REACHED END OF PATH");

  ROS_INFO("LEG MANIPULATION STARTED");

  while(ros::ok)
  {
  //The code waits until an input from keyboard is received. Once
  //an input is captured the function is called to achieve the
  //particular configuration
    int c = getch();
    char k = c;
    ROS_INFO("");
    crawl_gait.body_manip(c);
  }

  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////
