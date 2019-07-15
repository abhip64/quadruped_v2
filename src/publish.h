////////////////////////////////////////////////////////////////////////////////
#ifndef PUBLISH_H_
#define PUBLISH_H_
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include "std_msgs/Float64.h"
////////////////////////////////////////////////////////////////////////////////

class PUB
{
public:

  /**
    *Definition for all the joint controller publishers
    *@param node - NodeHandle of primary node (walk)
   */
  void publish_init(ros::NodeHandle&);

  /**
    *Publish joint state of a given leg
    *@param index - Leg index for which publishing needs to be done
    *@param fpos  - Joint states of a leg which needs to be achieved
   */
  void publish_leg(float*, int );

  PUB()
  {

  }

private:


  ros::Publisher j1_lf,j2_lf,j3_lf; /**< Publisher definition for leg 1*/
  ros::Publisher j1_lr,j2_lr,j3_lr; /**< Publisher definition for leg 2*/
  ros::Publisher j1_rf,j2_rf,j3_rf; /**< Publisher definition for leg 3*/
  ros::Publisher j1_rr,j2_rr,j3_rr; /**< Publisher definition for leg 4*/

protected:

};
////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
