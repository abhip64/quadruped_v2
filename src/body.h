////////////////////////////////////////////////////////////////////////////////
#ifndef BODY_H_
#define BODY_H_

////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include "leg.h"
////////////////////////////////////////////////////////////////////////////////

class BODY
{
public:
  /**
    *To get the body dimensions from the parameter server
   */
  static void init_body(ros::NodeHandle &);

  /**
    *To set the initial COG position along the path and intialise all variables
    *@param x - x coordinate of COG wrt global frame
    *@param y - y coordinate of COG wrt global frame
   */
  void set_initial_cog_pos(float,float);

  /**
    *Returns the body length
    *@return body_len
   */
  static float get_body_len();

  /**
    *Returns the body width
    *@return body_width
   */
  static float get_body_width();

////////////////////////////////////////////////////////////////////////////////
/*Variables relevant for in-place body manipulation*/

  float roll;        /**<Current roll angle(x axis) of the body of quadruped */
  float yaw;         /**<Current yaw angle(z axis) of the body of quadruped */
  float pitch;       /**<Current pitch angle(y axis) of the body of quadruped */

  float x_disp;      /**<x displacement of COG wrt previous body frame*/
  float y_disp;      /**<y displacement of COG wrt previous body frame*/
  float z_disp;      /**<z displacement of COG wrt previous body frame*/
////////////////////////////////////////////////////////////////////////////////

  float x_cog;       /**<x position of COG wrt global frame*/
  float y_cog;       /**<y position of COG wrt global frame*/

  float alpha;       /**<Current body orientation wrt global frame*/

  BODY()
  {

  }

private:
  static float body_len;     /**< Body length*/
  static float body_width;   /**< Body width*/
  static float body_height;  /**< Thickness of body*/

protected:

};
////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
