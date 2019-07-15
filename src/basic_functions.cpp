#include "gait.h"


void GAIT::update_legtip_pos()
{

  leg_lf.cur_legtip_pos[0] = leg_lf.next_legtip_pos[0];
  leg_lf.cur_legtip_pos[1] = leg_lf.next_legtip_pos[1];
  leg_lf.cur_legtip_pos[2] = leg_lf.next_legtip_pos[2];

  leg_rf.cur_legtip_pos[0] = leg_rf.next_legtip_pos[0];
  leg_rf.cur_legtip_pos[1] = leg_rf.next_legtip_pos[1];
  leg_rf.cur_legtip_pos[2] = leg_rf.next_legtip_pos[2];

  leg_lr.cur_legtip_pos[0] = leg_lr.next_legtip_pos[0];
  leg_lr.cur_legtip_pos[1] = leg_lr.next_legtip_pos[1];
  leg_lr.cur_legtip_pos[2] = leg_lr.next_legtip_pos[2];

  leg_rr.cur_legtip_pos[0] = leg_rr.next_legtip_pos[0];
  leg_rr.cur_legtip_pos[1] = leg_rr.next_legtip_pos[1];
  leg_rr.cur_legtip_pos[2] = leg_rr.next_legtip_pos[2];

  transform_to_body_frame(1);
  transform_to_body_frame(2);
  transform_to_body_frame(3);
  transform_to_body_frame(4);

}

void GAIT::reset_legtip_pos()
{
  float body_len   = BODY::get_body_len();
  float body_width = BODY::get_body_width();

  leg_lf.cur_legtip_pos[0] = 0;
  leg_lf.cur_legtip_pos[1] = 0;
  leg_lf.cur_legtip_pos[2] = 0;

  leg_rf.cur_legtip_pos[0] = 0;
  leg_rf.cur_legtip_pos[1] = 0;
  leg_rf.cur_legtip_pos[2] = 0;

  leg_lr.cur_legtip_pos[0] = 0;
  leg_lr.cur_legtip_pos[1] = 0;
  leg_lr.cur_legtip_pos[2] = 0;

  leg_rr.cur_legtip_pos[0] = 0;
  leg_rr.cur_legtip_pos[1] = 0;
  leg_rr.cur_legtip_pos[2] = 0;

  leg_lf.next_legtip_pos[0] =  0;
  leg_lf.next_legtip_pos[1] =  0;
  leg_lf.next_legtip_pos[2] =  0;

  leg_rf.next_legtip_pos[0] =  0;
  leg_rf.next_legtip_pos[1] =  0;
  leg_rf.next_legtip_pos[2] =  0;

  leg_lr.next_legtip_pos[0] = 0;
  leg_lr.next_legtip_pos[1] = 0;
  leg_lr.next_legtip_pos[2] = 0;

  leg_rr.next_legtip_pos[0] = 0;
  leg_rr.next_legtip_pos[1] = 0;
  leg_rr.next_legtip_pos[2] = 0;

  transform_to_body_frame(1);
  transform_to_body_frame(2);
  transform_to_body_frame(3);
  transform_to_body_frame(4);

}


void GAIT::store_start_pos()
{
leg_lf.leg_nominal_pos[0] = leg_lf.cur_legtip_body_frame[0];
leg_lf.leg_nominal_pos[1] = leg_lf.cur_legtip_body_frame[1];
leg_lf.leg_nominal_pos[2] = leg_lf.cur_legtip_body_frame[2];

leg_rf.leg_nominal_pos[0] = leg_rf.cur_legtip_body_frame[0];
leg_rf.leg_nominal_pos[1] = leg_rf.cur_legtip_body_frame[1];
leg_rf.leg_nominal_pos[2] = leg_rf.cur_legtip_body_frame[2];

leg_lr.leg_nominal_pos[0] = leg_lr.cur_legtip_body_frame[0];
leg_lr.leg_nominal_pos[1] = leg_lr.cur_legtip_body_frame[1];
leg_lr.leg_nominal_pos[2] = leg_lr.cur_legtip_body_frame[2];

leg_rr.leg_nominal_pos[0] = leg_rr.cur_legtip_body_frame[0];
leg_rr.leg_nominal_pos[1] = leg_rr.cur_legtip_body_frame[1];
leg_rr.leg_nominal_pos[2] = leg_rr.cur_legtip_body_frame[2];

}

LEG& GAIT::find_leg(int index)
{
  switch(index)
  {
    case 1:
    {
      return leg_lf;
    }
    break;
    case 2:
    {
      return leg_rf;
    }
    break;
    case 3:
    {
      return leg_lr;
    }
    break;
    case 4:
    {
      return leg_rr;
    }
    break;
    default:
    ROS_FATAL("INVALID LEG INDEX");
  }

}


void GAIT::transform_to_body_frame(int index)
{
  float body_len   = BODY::get_body_len();
  float body_width = BODY::get_body_width();

  switch(index)
  {
    case 1:
    leg_lf.cur_legtip_body_frame[0] = leg_lf.cur_legtip_pos[0] + (body_len   + GAIT::Rx)/2;
    leg_lf.cur_legtip_body_frame[1] = leg_lf.cur_legtip_pos[1] - (body_width  + GAIT::Rx)/2;
    leg_lf.cur_legtip_body_frame[2] = leg_lf.cur_legtip_pos[2] + GAIT::Rz;

    leg_lf.next_legtip_body_frame[0] = leg_lf.next_legtip_pos[0] + (body_len   + GAIT::Rx)/2;
    leg_lf.next_legtip_body_frame[1] = leg_lf.next_legtip_pos[1] - (body_width  + GAIT::Rx)/2;
    leg_lf.next_legtip_body_frame[2] = leg_lf.next_legtip_pos[2] + GAIT::Rz;

    break;

    case 2:
    leg_rf.cur_legtip_body_frame[0] = leg_rf.cur_legtip_pos[0] + (body_len + GAIT::Rx)/2;
    leg_rf.cur_legtip_body_frame[1] = leg_rf.cur_legtip_pos[1] + (body_width  + GAIT::Rx)/2;
    leg_rf.cur_legtip_body_frame[2] = leg_rf.cur_legtip_pos[2] + GAIT::Rz;

    leg_rf.next_legtip_body_frame[0] = leg_rf.next_legtip_pos[0] + (body_len + GAIT::Rx)/2;
    leg_rf.next_legtip_body_frame[1] = leg_rf.next_legtip_pos[1] + (body_width  + GAIT::Rx)/2;
    leg_rf.next_legtip_body_frame[2] = leg_rf.next_legtip_pos[2] + GAIT::Rz;

    break;

    case 3:
    leg_lr.cur_legtip_body_frame[0] = leg_lr.cur_legtip_pos[0] - (body_len + GAIT::Rx)/2;
    leg_lr.cur_legtip_body_frame[1] = leg_lr.cur_legtip_pos[1] - (body_width  + GAIT::Rx)/2;
    leg_lr.cur_legtip_body_frame[2] = leg_lr.cur_legtip_pos[2] + GAIT::Rz;

    leg_lr.next_legtip_body_frame[0] = leg_lr.next_legtip_pos[0] - (body_len + GAIT::Rx)/2;
    leg_lr.next_legtip_body_frame[1] = leg_lr.next_legtip_pos[1] - (body_width  + GAIT::Rx)/2;
    leg_lr.next_legtip_body_frame[2] = leg_lr.next_legtip_pos[2] + GAIT::Rz;

    break;

    case 4:
    leg_rr.cur_legtip_body_frame[0] = leg_rr.cur_legtip_pos[0] - (body_len + GAIT::Rx)/2;
    leg_rr.cur_legtip_body_frame[1] = leg_rr.cur_legtip_pos[1] + (body_width  + GAIT::Rx)/2;
    leg_rr.cur_legtip_body_frame[2] = leg_rr.cur_legtip_pos[2] + GAIT::Rz;

    leg_rr.next_legtip_body_frame[0] = leg_rr.next_legtip_pos[0] - (body_len + GAIT::Rx)/2;
    leg_rr.next_legtip_body_frame[1] = leg_rr.next_legtip_pos[1] + (body_width  + GAIT::Rx)/2;
    leg_rr.next_legtip_body_frame[2] = leg_rr.next_legtip_pos[2] + GAIT::Rz;

    break;

    default:
    ROS_FATAL("WRONG LEG INDEX PASSED FOR TRANSFORMATION TO BODY FRAME");
  }

}

void GAIT::transform_to_leg_frame(int index)
{
  float body_len   = BODY::get_body_len();
  float body_width = BODY::get_body_width();

  Eigen::MatrixXd p_vec(4,1);

  switch(index)
  {
    case 1:

    p_vec << -x_disp,
              y_disp,
             -z_disp,
              1;

    leg_lf.next_legtip_pos[0] = leg_lf.next_legtip_body_frame[0] + p_vec(0,0);
    leg_lf.next_legtip_pos[1] = leg_lf.next_legtip_body_frame[1] + p_vec(1,0);
    leg_lf.next_legtip_pos[2] = leg_lf.next_legtip_body_frame[2] + p_vec(2,0);

    break;

    case 2:

    p_vec << -x_disp,
             -y_disp,
             -z_disp,
              1;

    leg_rf.next_legtip_pos[0] = leg_rf.next_legtip_body_frame[0] + p_vec(0,0);
    leg_rf.next_legtip_pos[1] = leg_rf.next_legtip_body_frame[1] + p_vec(1,0);
    leg_rf.next_legtip_pos[2] = leg_rf.next_legtip_body_frame[2] + p_vec(2,0);

    break;

    case 3:

    p_vec <<  x_disp,
              y_disp,
             -z_disp,
              1;

    leg_lr.next_legtip_pos[0] = leg_lr.next_legtip_body_frame[0] + p_vec(0,0);
    leg_lr.next_legtip_pos[1] = leg_lr.next_legtip_body_frame[1] + p_vec(1,0);
    leg_lr.next_legtip_pos[2] = leg_lr.next_legtip_body_frame[2] + p_vec(2,0);

    break;

    case 4:

    p_vec <<  x_disp,
             -y_disp,
             -z_disp,
              1;

    leg_rr.next_legtip_pos[0] = leg_rr.next_legtip_body_frame[0] + p_vec(0,0);
    leg_rr.next_legtip_pos[1] = leg_rr.next_legtip_body_frame[1] + p_vec(1,0);
    leg_rr.next_legtip_pos[2] = leg_rr.next_legtip_body_frame[2] + p_vec(2,0);

    break;

    default:
    ROS_FATAL("WRONG LEG INDEX PASSED FOR TRANSFORMATION TO LEG FRAME");
  }
}


void GAIT::set_next_pos(LEG& cur_leg)
{
  /*Leg phasing manouever*/
  if(cur_leg.get_leg_substate()==0)
  {
    cur_leg.next_legtip_pos[0] = -GAIT::body_stride/4;
    cur_leg.next_legtip_pos[1] =  0.0;
    cur_leg.next_legtip_pos[2] =  0.0;
  }
  /*Other two not really required but just kept for completeness*/
  else if(cur_leg.get_leg_substate()==1)
  {
    cur_leg.next_legtip_pos[0] =  -GAIT::body_stride/2;
    cur_leg.next_legtip_pos[1] =  0.0;
    cur_leg.next_legtip_pos[2] =  0.0;
  }
  else if(cur_leg.get_leg_substate()==2)
  {
    cur_leg.next_legtip_pos[0] =  GAIT::body_stride/2;
    cur_leg.next_legtip_pos[1] =  0.0;
    cur_leg.next_legtip_pos[2] =  0.0;
  }
  else
  {

  }
}
