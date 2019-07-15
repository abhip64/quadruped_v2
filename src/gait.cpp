#include "gait.h"

void GAIT::init_quad(ros::NodeHandle& node,float x,float y)
{
  /*Initialisation of link lengths from the parameter server*/
  LEG::init_leg(node);

  /*Initialisation of body dimensions from the parameter server*/
  BODY::init_body(node);

  /*Setting the initial position of the COG of the quadruped in the
  path*/
  b.set_initial_cog_pos(x,y);

  /*For initialising publisher for 12 joint controllers*/
  publish_pos.publish_init(node);

  float l1_len = LEG::get_link1_len();
  float l2_len = LEG::get_link2_len();
  float l3_len = LEG::get_link3_len();

  /*Calculation of workspace dimensions for periodic gait. It is based on the
  home position of the quadruped and on the assumption that the height of
  the body doesnt change during motion i.e. the z coordinate of the starting
  and ending points of the leg tip are same.
  */
  GAIT::Rx = (l1_len + l2_len*cos(LEG::home_pos[1]) + l3_len*cos(LEG::home_pos[1] + LEG::home_pos[2] + M_PI/2))*pow(2,0.5);
  GAIT::Ry = GAIT::Rx;
  GAIT::Rz = (l2_len*sin(LEG::home_pos[1]) + l3_len*sin(LEG::home_pos[1] + LEG::home_pos[2] + M_PI/2));

  /*Setting the body stride based on the leg workspace dimension*/
  body_stride = GAIT::Rx/2;
}

void GAIT::moveto(int index,float q1,float q2,float q3)
{
  float pos[3] = {q1,q2,q3};

  /*Publish required joint states*/
  publish_pos.publish_leg(pos,index);

}

void GAIT::goto_home_pos()
{
  double begin = ros::Time::now().toSec();

  while(ros::Time::now().toSec()-begin<=1)
  /*Publish joint positions corresponding to home position*/
{
  publish_pos.publish_leg(LEG::home_pos,1);
  publish_pos.publish_leg(LEG::home_pos,2);
  publish_pos.publish_leg(LEG::home_pos,3);
  publish_pos.publish_leg(LEG::home_pos,4);
}

  /*Set the current position as zero position of the leg tip in leg
    reference frame*/
  reset_legtip_pos();

}

void GAIT::leg_phasing()
{
  /*Set the leg as being in swing phase*/
  leg_rf.set_leg_substate(0);
  leg_rr.set_leg_substate(0);

  /*Push in the legs for execution*/
  leg_exec_order.push(2);
  leg_exec_order.push(4);

  /*Assign 2 seconds for each leg phasing manouever*/
  time_exec.push(2);
  time_exec.push(2);

  /*Execute leg motion*/
  execute_gait();

  /*Store the current position of the leg tip for further calculation.
    Check out the wiki for further info.
  */
  store_start_pos();
}

void GAIT::follow_2d_path()
{
  /*Leg tip positions in body frame*/
  Eigen::MatrixXd leg_tip_pose_cog(4,1);
  Eigen::MatrixXd next_leg_tip_pose_cog(4,1);

  /*Transformation Matrices for carrying out path tracking*/
  Eigen::Matrix4d T,T1,T2;

  int cur_leg_index;

  float xcg_dummy,ycg_dummy,alpha_dummy;

  while(calc_next_move(b.x_cog,b.y_cog,body_stride))
  {
    /*There are two phases for each locomtion cycle. The function
      calc_next_move() calculates the COG position and the rotation
      angle for the two phases wrt the global frame which is located
      at the beginning of the path. For achieving the
      transition, the COG position and rotation angle of the body has to
      be determined wrt the previous reference frame which is done below.
    */
    xcg_dummy   = x_cog2;
    ycg_dummy   = y_cog2;
    alpha_dummy = alpha2;

    /*Relative angle change wrt the first phase of locomtion*/
    alpha2 -= alpha2 - alpha1;

    /*Relative angle change wrt the previous frame at the start of
     locomtion.*/
    alpha1 -= b.alpha;

    /*Relative position change wrt the first phase of locomotion*/
    x_cog2 -= x_cog1;
    y_cog2 -= y_cog1;

    /*Relative position change wrt the previous frame at the start of
     locomtion.*/
    x_cog1 -= b.x_cog;
    y_cog1 -= b.y_cog;

    /*Transformation matrix associated with first locomotion phase*/
    T1 <<  cos(alpha1), -sin(alpha1), 0, x_cog1,
           sin(alpha1),  cos(alpha1), 0, y_cog1,
           0,            0,           1, 0,
           0,            0,           0, 1;

    /*Transformation matrix associated with second locomotion phase*/
    T2 <<  cos(alpha2), -sin(alpha2), 0, x_cog2,
           sin(alpha2),  cos(alpha2), 0, y_cog2,
           0,            0,           1, 0,
           0,            0,           0, 1;


    T = T1;

    /*Determination of leg execution order by traversing the gait matrix.
      Leg index is pushed into the execution queue if lifting of the leg doesnt
      destabilize the body. The exact method of calculation of foot hold
      positions will be discussed in detail in the github woko page.
    */
    for(int i=0;i<row;i++)
    {
      for(int j=0;j<column;j++)
      {
        if(crawl_gait_matrix[i][j] == 1)
        {
          cur_leg_index = j + 1;

          LEG &cur_leg = find_leg(cur_leg_index);

          /*Stability check*/
          if(stability_check(cur_leg_index))
          {
            cur_leg.set_leg_substate(3);

            leg_exec_order.push(cur_leg_index);

            time_exec.push(duration_vector[i]);

            leg_tip_pose_cog << cur_leg.leg_start_pos[0],
                                cur_leg.leg_start_pos[1],
                                cur_leg.leg_start_pos[2],
                                1;

            leg_tip_pose_cog = T1*T2*leg_tip_pose_cog;

            cur_leg.next_legtip_body_frame[0] = leg_tip_pose_cog(0,0);
            cur_leg.next_legtip_body_frame[1] = leg_tip_pose_cog(1,0);
            cur_leg.next_legtip_body_frame[2] = leg_tip_pose_cog(2,0);

            transform_to_work_frame(cur_leg_index);

          }
          else
          {
            ROS_FATAL("LOCOMOTION CANNOT BE CONTINUED AS STABILITY CONDITION IS VIOLATED");
          }

        }
        else if(crawl_gait_matrix[i][j] == 2)
        {
          execute_gait();

          update_leg_for_body_motion(T);

          time_exec.push(duration_vector[i]);

          execute_body_motion();

          T1 << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

          T = T2;
        }
      }
    }

    b.x_cog = xcg_dummy;
    b.y_cog = ycg_dummy;
    b.alpha = alpha_dummy;

  }

}

void GAIT::update_leg_for_body_motion(Eigen::Matrix4d &T)
{
  Eigen::MatrixXd p(4,1);
  Eigen::Matrix4d T_inv;

  T_inv = T.inverse();

  for(int leg_index=1;leg_index<=4;leg_index++)
  {
    LEG &cur_leg = find_leg(leg_index);

    p << cur_leg.cur_legtip_body_frame[0],
         cur_leg.cur_legtip_body_frame[1],
         cur_leg.cur_legtip_body_frame[2],
         1;

    p = T_inv*p;

    cur_leg.next_legtip_body_frame[0] = p(0,0);
    cur_leg.next_legtip_body_frame[1] = p(1,0);
    cur_leg.next_legtip_body_frame[2] = p(2,0);

    transform_to_work_frame(leg_index);

  }


}

void GAIT::execute_gait()
{
  float des_end_pos[3];
  float *result;
  int current_leg_index;

  double begin;
  double t;
  float T;


  while(!leg_exec_order.empty())
  {
    current_leg_index = leg_exec_order.front();
    leg_exec_order.pop();

    T = time_exec.front();
    time_exec.pop();

    if(current_leg_index)
    {
      LEG &current_leg = find_leg(current_leg_index);

      /*Assigning leg tip end position based on leg sub state. Only useful for
      phasing. Not used during the locomotion cycle*/
      set_next_pos(current_leg);

      begin = ros::Time::now().toSec();
      t = 0;

      while(t<=T)
      {
        for(int i=0;i<3;i++)
        {
          /*Start position*/
          float A = current_leg.cur_legtip_pos[i];
          /*End position*/
          float B = current_leg.next_legtip_pos[i];

          /*To interpolate between the start and end positions of the leg tip.
            Interpolation for x and y axis is done using a general 5th order
            polynomial. Whereas z axis interpolation is done using a 3rd
            order polynomial. The reason is because for the 5th order polynomial
            the second derivative being zero the curve remains almost parallel
            near the end position. Thus it has high chance of colliding with
            ground due to small instabilities.
          */
          if(i!=2)
          {
            des_end_pos[i] = A + (10*(B - A)/pow(T,3))*pow(t,3) +
                                 (15*(A - B)/pow(T,4))*pow(t,4) +
                                 (6* (B - A)/pow(T,5))*pow(t,5);
          }

          else
          {
            /*For the z axis start and end positions are zero. If interpolation
              is applied directly then it will interpolate linearly. Thus it is
              necessary to divide the path into two equal halves and then
              interpolate twice.
            */
            if(t<=(T/2))
            {
              /*Maximum z-height used for interpolation.*/
              B = -0.07;

              des_end_pos[i] = A + (3*(B - A)/pow((T/2),2))*pow(t,2) +
                                   (2*(A - B)/pow((T/2),3))*pow(t,3);

            }
            else
            {
              /*Maximum z-height used for interpolation.*/
              A = -0.07;

              des_end_pos[i] = (5*B-4*A) + (24*(A - B)/pow(T,1))*pow(t,1) +
                                           (36*(B - A)/pow(T,2))*pow(t,2) +
                                           (16*(A - B)/pow(T,3))*pow(t,3);
            }
          }
        }

        /*Joint positions for achieving the given leg tip position*/
        result = current_leg.ik_calc_leg(current_leg.convert_to_leg_frame(des_end_pos,current_leg_index,GAIT::Rx,GAIT::Rz));

        publish_pos.publish_leg(result,current_leg_index);

        ros::Duration(sleep_time).sleep();

        t = ros::Time::now().toSec() - begin;
      }

      transform_to_body_frame(current_leg_index);

      current_leg.update_leg_state();
    }
  }
  update_legtip_pos();
}

void GAIT::execute_body_motion()
{
  float des_end_pos[3];
  float *result;

  double begin = ros::Time::now().toSec();
  double t = 0;

  float T = time_exec.front();
  time_exec.pop();

  while(t<=T)
  {

    for(int leg_index=1;leg_index<=4;leg_index++)
    {

      LEG &current_leg =find_leg(leg_index);

      for(int i=0;i<3;i++)
      {
        /*Start position*/
        float A = current_leg.cur_legtip_pos[i];
        /*End position*/
        float B = current_leg.next_legtip_pos[i];

        des_end_pos[i] = A + (10*(B - A)/pow(T,3))*pow(t,3) +
                             (15*(A - B)/pow(T,4))*pow(t,4) +
                             (6* (B - A)/pow(T,5))*pow(t,5);

      }
      /*Joint positions for achieving the final leg tip position*/
      result = current_leg.ik_calc_leg(current_leg.convert_to_leg_frame(des_end_pos,leg_index,GAIT::Rx,GAIT::Rz));

      publish_pos.publish_leg(result,leg_index);

      ros::Duration(sleep_time).sleep();

      t = ros::Time::now().toSec() - begin;
    }

  }

  update_legtip_pos();
  transform_to_body_frame(1);
  transform_to_body_frame(2);
  transform_to_body_frame(3);
  transform_to_body_frame(4);

}
