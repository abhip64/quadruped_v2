///////////////////////////////////////////////////////////////////////////
//LEG CLASS IS INTENDED TO DEFINE ALL THE PROPERTIES RELATING TO A LEG
//WHICH CAN BE USED BY THE GAIT CLASS FOR LOCOMOTION. ALL THE LEGS ARE ASSUMED
//TO HAVE THE SAME PARAMETERS WHICH ARE EITHER OBTAINED FROM THE PARAMETER
//SERVER OR IS EXPLICITLY DEFINED IN THE CODE
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
#ifndef LEG_H_
#define LEG_H_
///////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <math.h>
///////////////////////////////////////////////////////////////////////////
#define convert_to_deg (180.0/M_PI)
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

class LEG
{
public:

  /**
    *Initialise the leg with the length of the joints obtained from
    *parameter server
    *@param node - Nodehandle corresponding to the primary node (walk)
   */
  static void init_leg(ros::NodeHandle&);


  ///////////////////////////////////////////////////////////////////////////
  //FINITE STATE ASSUMPTION IS USED FOR THE AUTOMATION WHERE THE STATE OF THE
  //LEG IN STANCE PHASE IS REPRESENTED BY 0 AND STATE OF LEG IN SWING PHASE
  //IS DENOTED BY 1. INTERNAL STATES CORRESPONDING TO THE MOTION OF THE LEG
  //IS ACHIEVED VIA CONTINOUS APPROACH.(IT CAN ALSO BE DONE VIA FINITE STATE
  //APPROACH)
  ///////////////////////////////////////////////////////////////////////////

  /**
    *To update the state of the leg (whether in swing or stance)
   */
  void update_leg_state();

  /**
    *To update the sub-state of leg in swing (moving forward, backward)
   */
  void set_leg_substate(int);

  /**
    *To get the state of the leg (whether in swing or stance)
   */
  bool get_leg_state();

  /**
    *To get the sub-state of leg in swing (moving forward, backward)
   */
  int get_leg_substate();

  /**
    *Converts the leg tip position to a coordinate with its x axis along
    *the leg at its 45 deg position and z axis pointing downward. IK
    *calcualtions are easier in this frame.
    *@param des_end_pos - 3D Leg tip position in workspace frame
    *@param index       - Index of leg for which transformation is to be done
    *@param Rx          - Workspace dimension along x axis
    *@param Rz          - Workspace dimension along z axis
    *@return 3D Leg tip position in leg frame
   */
  float* convert_to_leg_frame(float*, int&,float,float);

  /**
    *To carry out leg inverse kinematics given desired leg tip position.
    *Details regarding procedure and formulae for ik will be discussed
    *in the github wiki page.
    *@param des_end_pos - 3D point expressed in leg frame
    *@todo Check the ik calculation equations
   */
  float* ik_calc_leg(float*);

  /**
    *Return link1 length
    *@return l1_len
   */
  static float get_link1_len();

  /**
    *Return link2 length
    *@return l2_len
   */
  static float get_link2_len();

  /**
    *Return link3 length
    *@return l3_len
   */
  static float get_link3_len();

  float result[3];                 /**< Used to store the final joint state
                                        of a leg */

  static float home_pos[3];        /**< Joint angles corresponding to the
                                        home position of the legs.
                                        Initialisation of the values are
                                        done in gait.h */

  float cur_legtip_pos[3];         /**< Current tip position of the leg in
                                        leg frame.*/
  float cur_legtip_body_frame[3];  /**< Current tip position of the leg in
                                        body frame.*/

  float next_legtip_pos[3];        /**< Next tip position of the leg in
                                        leg frame.*/
  float next_legtip_body_frame[3]; /**< Next tip position of the leg in
                                        body frame.*/

  float leg_start_pos[3];          /**< Position of the leg after leg
                                        phasing*/

  LEG()
  {

  }

private:

  int leg_index;               /**
                                 *LEG NUMBERING -
                                 *LEFT FRONT  - 1
                                 *RIGHT FRONT - 2
                                 *LEFT REAR   - 3
                                 *RIGHT REAR  - 4
                                */


  bool leg_state;            /**< State of leg depicting whether leg is in
                                  swing or stance phase*/
  int leg_substate;          /**< Sub-state of leg depicting whether leg
                                  is in forqard or backward motion*/

  static float l1_len;       /**< Length of link 1 (FEMUR)*/
  static float l2_len;       /**< Length of link 2 (TIBIA)*/
  static float l3_len;       /**< Length of link 3 (COXA)*/

protected:

};

///////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////
