/**@file
  *\brief Associated functions for carrying out in-place body manipulations
  *In-place body manipulation refers to the action of the quadruped where its
  *legs are planted firmly on the ground and its body is moved around. This is
  *done for fine adjustments of the body position.
  *Body manipulations start when path tracking is completed. Keyboard inputs
  *have to be provided to the terminal in which the walk node is initiated.
  *Keyboard combinations for movement -
  *
  *Translation - t
  *   X-Direction - x
  *   Y-Direction - y
  *   Z-Direction - z
  *       Positive movement - +
  *       Negative movement - -
  *Rotation - t
  *   Roll  - x
  *   Pitch - y
  *   Yaw   - z
  *       Positive movement - +
  *       Negative movement - -
  *
  *Step size for each of the movement is defined below.
 */

#include "gait.h"

///////////////////////////////////////////////////////////////////////////////
#define step_rot   1              /**< Step size for rotation in degrees*/
#define step_trans 0.01           /**< Step size for translation in m*/
#define exec_time  1              /**<Time for one body manipulation execution*/
///////////////////////////////////////////////////////////////////////////////

int prev = 0;
int aprev = 0;

void GAIT::body_manip(int k)
{
///////////////////////////////////////////////////////////////////////////////
  b.roll = 0;
  b.pitch = 0;
  b.yaw = 0;
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
  if(k==114)          //rotation
  {
    prev = 114;
    aprev=0;
  }
  else if(k==116)     //translation
  {
    prev = 116;
    aprev=0;}

    if(prev==114||prev==116)
    {
      if(k==120)      //x
      aprev=120;
      else if(k==121) //y
      aprev=121;
      else if(k==122) //z
      aprev=122;
    }

    if(prev==114&&aprev==120)
    {
      if(k==61)
      b.roll = step_rot;
      else if(k==45)
      b.roll = -step_rot;

      move_body();

    }

    else if(prev==114&&aprev==121)
    {
      if(k==61)
      b.pitch = step_rot;
      else if(k==45)
      b.pitch = -step_rot;

      move_body();

    }

    else if(prev==114&&aprev==122)
    {
      if(k==61)
      b.yaw = step_rot;
      else if(k==45)
      b.yaw =-step_rot;

      move_body();
    }
///////////////////////////////////////////////////////////////////////////////
    if(prev==116&&aprev==120)
    {
      if(k==61)
      b.x_disp = step_trans;
      else if(k==45)
      b.x_disp =-step_trans;

      move_body();

    }

    else if(prev==116&&aprev==121)
    {
      if(k==61)
      b.y_disp = step_trans;
      else if(k==45)
      b.y_disp =-step_trans;

      move_body();

    }

    else if(prev==116&&aprev==122)
    {
      if(k==61)
      b.z_disp = step_trans;
      else if(k==45)
      b.z_disp =-step_trans;

      move_body();
    }
///////////////////////////////////////////////////////////////////////////////
  }

  void GAIT::move_body()
  {
    Eigen::Matrix4d T;

    float a = b.yaw  *convert_to_rad;
    float p = b.pitch*convert_to_rad;
    float y = b.roll *convert_to_rad;

    /*Tait-Brian convention is used to represnt the Euler angles*/
    T  << (cos(a)*cos(p)),   (cos(a)*sin(p)*sin(y)-sin(a)*cos(y)), (cos(a)*sin(p)*cos(y)+sin(a)*sin(y)), b.x_disp,
          (sin(a)*cos(p)),   (sin(a)*sin(p)*sin(y)+cos(a)*cos(y)), (sin(a)*sin(p)*cos(y)-cos(a)*sin(y)), b.y_disp,
          (-sin(p)),                        (cos(p)*sin(y)),                      (cos(p)*cos(y)), b.z_disp,
          0,                                      0,                                    0,      1;

    Eigen::MatrixXd p_vec(4,1);

    for(int leg_index=1;leg_index<=4;leg_index++)
    {
      LEG &cur_leg = find_leg(leg_index);

      p_vec << cur_leg.cur_legtip_body_frame[0],
               cur_leg.cur_legtip_body_frame[1],
               cur_leg.cur_legtip_body_frame[2],
               1;

      p_vec = T*p_vec;

      cur_leg.next_legtip_body_frame[0] = p_vec(0,0);
      cur_leg.next_legtip_body_frame[1] = p_vec(1,0);
      cur_leg.next_legtip_body_frame[2] = p_vec(2,0);

      transform_to_work_frame(leg_index);

    }

    //Time allotted for one body manipulation execution
    time_exec.push(exec_time);
    execute_body_motion();

    /**
      *@todo Fix the issue of body orientation being reset everytime
      *      after a body manipulation
     */
    b.x_disp = 0;
    b.y_disp = 0;
    b.z_disp = 0;
    b.roll = 0;
    b.pitch = 0;
    b.yaw = 0;

  }
