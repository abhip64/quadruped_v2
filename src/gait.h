////////////////////////////////////////////////////////////////////////////////////
#ifndef GAIT_H_
#define GAIT_H_

#include <ros/ros.h>
#include <queue>
#include <boost/date_time.hpp>
#include <math.h>
#include <Eigen/Dense>
#include "leg.h"
#include "body.h"
#include "publish.h"
#include "path.h"

////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//Conversion factor for converting from degree to radian
#define convert_to_rad 0.0174444
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
class GAIT
{
public:

////////////////////////////////////////////////////////////////////////////////////
//CORE FUNCTIONS

  /**
    *Initialises the objects of the class LEG, BODY and PUBLISH. Workspace of
    *the leg is also calculated based on the dimensions of the leg provided
    *in the param.yaml file. The initial COG position of the quadruped
    *is also set.
    *@param node - Nodehandle corresponding to the primary node(walk)
    *@param x    - Initial x position of COG of the quadruped
    *@param y    - Initial y position of COG of the quadruped
    */
  void init_quad(ros::NodeHandle&,float,float);

  /**
    *To move the legs to the home position as defined in the LEG class
    */
  void goto_home_pos();

  /**
    *Initial leg movements required before the start of gait. This is to
    *ensure stability is ensured when the actual locomtion is started.
    *This is considered as the start position of the locomotion.
    *The nature of leg movement has been described in the github wiki
    *page associated with the code.
    */

  void leg_phasing();

  /**
    *Function to determine the footholds to achieve a particular COG position
    *along the path. Leg transferance position for each leg is calculated
    *and is pushed into the stack for execution only if the leg can be lifted
    *without causing a body destabilization. Stability check is carried out
    *in the function @stability_check()
    */
  void follow_2d_path();

  /**
    *Computes the static longitudinal stability margin(SLSM) associated with
    *the lifting of a leg. Leg is pushed into the stack for execution if SLSM
    *is positive. Otherwise execution is halted.
    *@param current_leg - Leg that is to be lifted
    *@return 0 for failure, 1 for success
    *@todo If SLSM proves to be negative then halting path tracking may not
    *      be a good idea. Criterions such as Criterion-N, Criterion-K can be
    *      applied to determine an appropriate foot hold position and foot
    *      lifting order which would help to achieve the required SLSM
    */
  bool stability_check(int);

  /**To execute the leg motions after they have been checked for stability.
    *It executes half of a locomtion cycle i.e movement of two legs
    */
  void execute_gait();

  /**
    *Executes body motion which comes after two leg motions
    */
  void execute_body_motion();

  /**
    *Carry out body in place body manipulations based on keyboard input.
    *The required sequence of keyboard inputs for achieving a particular
    *manipulation has been described in @body_manip.cpp
    *@param k - Keyboard input
    */
  void body_manip(int);

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//AUXILLIARY FUNCTIONS

  /**
    *Setting all the current and next leg tip positions to nil. This is done
    *in the beginning of code to prevent garbage values being used for
    *calculations
    */
  void reset_legtip_pos();

  /**
    *Once the leg motion has moved from current to next position, the
    *values of current position variable is updated with the next position
    *variable
    */
  void update_legtip_pos();

  /**
    *Transform the current leg positions stored wrt leg reference
    *frame to the body frame. Current leg position in body frame
    *are required for stability calculations
    *@param index - Index of the leg for which the transformation is
    *               to be done
    */
  void transform_to_body_frame(int);

  /**
    *Transform the next leg positions stored wrt body reference frame
    *to the leg frame. It is easier to calculate next leg positions in the
    *body reference frame. Transformation to leg frame is necessary
    *as the inverse kinematics calculations for determining the joint
    *angles are easier in the leg frame.
    *@param index - Index of the leg for which the transformation is
    *               to be done
    */
  void transform_to_leg_frame(int);

  /**
    *Update the position that the leg would achieve wrt body frame after
    *one body motion. This would be the next leg tip position with respect
    *to the previous body frame. More details on the implementation can
    *be found on the associated github wiki page.
    *@param T - Transformation matrix associated with body motion
    */
  void update_leg_for_body_motion(Eigen::Matrix4d&);

  /**
    *Store the next position to which the the leg tip has to be moved.
    *It is represneted in the leg reference frame.
    *@param cur_leg - Leg whose next position is to be stored
    */
  void set_next_pos(LEG&);

  /**
    *Store the position of the tip of each leg during the start of locomotion
    *so that it can be later used in calculations. The data is in the body
    *reference frame
    */
  void store_start_pos();

  /**
    *Returns the LEG class object associated with a particular index
    *@param leg_index - Index number of leg
    *@return Reference of LEG object associated with a particular index
    */
  LEG& find_leg(int);

  /**
    *Move the leg to achieve a particular joint angle set
    *@param index - Index of leg to be moved
    *@param q1    - Joint angle corresponding to first joint
    *@param q2    - Joint angle corresponding to second joint
    *@param q3    - Joint angle corresponding to third joint
    */
  void moveto(int,float,float,float);
////////////////////////////////////////////////////////////////////////////////////

  /*
    *Leg workspace dimensions along the x,y and z axes. It defines
    *a cuboidal region within which all the leg motions must occur.
    *Rx also defines the maximum achievable stride. These are defined
    *as static as these values remain same for all legs.
    */
  static float Rx;                /**< Leg workspace dimension along x axis.
                                       It also is the maximum achievable
                                       stride*/
  static float Ry;                /**< Leg workspace dimension along y axis*/
  static float Rz;                /**< Leg workspace dimension along z axis*/


  std::queue<int> leg_exec_order; /**< When a leg is to be moved it's index
                                       is pushed into the queue. It stores
                                       the leg index until a body motion after
                                       which it is cleared.
                                    */

  std::queue<float> time_exec;    /**< It is populated alongwith leg execution
                                       queue. It denotes the time in which the
                                       leg/body motion has to be completed
                                    */


  GAIT(ros::NodeHandle& node,float x_start_path,float y_start_path)
  {
    init_quad(node,x_start_path,y_start_path);
  }

private:
  BODY b;
  ////////////////////////////////////////////////////////////////////////////
  //LEG NUMBERING -
  //LEFT FRONT  - 1
  //RIGHT FRONT - 2
  //LEFT REAR   - 3
  //RIGHT REAR  - 4
  ////////////////////////////////////////////////////////////////////////////
  LEG leg_lf;
  LEG leg_lr;
  LEG leg_rf;
  LEG leg_rr;



  PUB publish_pos;              /**< To publish joint positions to gazebo
                                     controllers */

  ////////////////////////////////////////////////////////////////////////////
  /*GAIT DEFINITIONS -
   *GAIT DEFINITION OF A QUADRUPED CAN BE ACHIEVED USING A COMBINATION OF
   *GAIT MATRIX AND DURATION VECTOR, GAIT FORMULA AND TOTAL TIME FOR ONE
   *LOCOMTION CYCLE OR BY MAKING USE OF GAIT DIAGRAMS. HERE THE APPROACH OF
   *DEFINNING THE GAIT USING GAIT MATRIX AND DURATION MATRIX IS ADOPTED.
   */
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////
  //                             CRAWL GAIT
  /////////////////////////////////////////////////////////////////////////////
  /*CRAWL GAIT PARAMTERS DEFINITION USING GAIT MATRIX AND DURATION VECTOR -
   *EACH ROW OF THE GAIT MATRIX HAS FOUR COLUMNS AND HAS A CORRESPONDING
   *VALUE OF DURATION. THE COLUMN DEFINES THE STATE OF THE ROBOT LEGS
   *FOR A PERIOD GIVEN IN THE DURATION VECTOR.
   */
  ////////////////////////////////////////////////////////////////////////////

  static int const crawl_gait_matrix[8][4];   /**< Gait matrix that defines
                                                   the leg lifting sequence.
                                                   @todo Check if static
                                                   definition is necessary */


  static int row,column;                  /**< Dimensions of the gait matrix */


  static float duration_vector[8];        /**< Each element defines the time
                                               for which a particular phase
                                               of the gait will continue. The
                                               time is expressed in seconds.
                                               adding all the elements of the
                                               vector gives the total time taken
                                               for one locomotion cycle using
                                               the given gait.
                                               */

  static float sleep_time;                /**< Sleep time to ensure simulation
                                               catches up with the running code
                                            */



  ////////////////////////////////////////////////////////////////////////////

protected:

};
////////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////////
