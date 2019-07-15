/////////////////////////////////////////////////////////////////////////////////
/**@file
  *Associated functions for calculating the STATIC LONGITUDINAL STABILTY
  *MARGIN(SLSM) when a leg is lifted. If SLSM is positive then the gait
  *can continue otherwise it has to be halted. Calculation of SLSM will
  *detailed in the associated wiki page.
 */
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
#include "gait.h"
/////////////////////////////////////////////////////////////////////////////////

bool GAIT::stability_check(int current_leg)
{
switch (current_leg)
{
case 1:
{
/*Gets the coordinates of collateral and contralateral legs*/
float x2 = leg_rf.next_legtip_body_frame[0];
float y2 = leg_rf.next_legtip_body_frame[1];

float x3 = leg_lr.next_legtip_body_frame[0];
float y3 = leg_lr.next_legtip_body_frame[1];

float m = (y3-y2)/(x3-x2);

/*Calculates the distance between the line connecting collateral and
 *contralateral legs from the origin along the axis*/
if((y2 - m*x2)<0)
return 1;
else
return 0;
}
break;

case 2:
{
/*Gets the coordinates of collateral and contralateral legs*/
float x1 = leg_lf.next_legtip_body_frame[0];
float y1 = leg_lf.next_legtip_body_frame[1];

float x4 = leg_rr.next_legtip_body_frame[0];
float y4 = leg_rr.next_legtip_body_frame[1];

float m = (y4-y1)/(x4-x1);

/*Calculates the distance between the line connecting collateral and
 *contralateral legs from the origin along the axis*/
if((y1 - m*x1)>0)
return 1;
else
return 0;
}

break;

case 3:
{
/*Gets the coordinates of collateral and contralateral legs*/
float x1 = leg_lf.next_legtip_body_frame[0];
float y1 = leg_lf.next_legtip_body_frame[1];

float x4 = leg_rr.next_legtip_body_frame[0];
float y4 = leg_rr.next_legtip_body_frame[1];

float m = (y4-y1)/(x4-x1);

/*Calculates the distance between the line connecting collateral and
 *contralateral legs from the origin along the axis*/
if((y1 - m*x1)<0)
return 1;
else
return 0;
}

break;

case 4:
{
/*Gets the coordinates of collateral and contralateral legs*/
float x2 = leg_rf.next_legtip_body_frame[0];
float y2 = leg_rf.next_legtip_body_frame[1];

float x3 = leg_lr.next_legtip_body_frame[0];
float y3 = leg_lr.next_legtip_body_frame[1];

float m = (y3-y2)/(x3-x2);

/*Calculates the distance between the line connecting collateral and
 *contralateral legs from the origin along the axis*/
if((y2 - m*x2)>0)
return 1;
else
return 0;
}
break;

default:
ROS_FATAL("INVALID LEG INDEX");

}
return 0;
}
/////////////////////////////////////////////////////////////////////////////////
