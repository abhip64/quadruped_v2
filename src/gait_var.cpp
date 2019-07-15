#include "gait.h"

float GAIT::Rx = 0;
float GAIT::Ry = 0;
float GAIT::Rz = 0;
float GAIT::body_stride = 0;
float LEG::Rx_max = 0;
float LEG::Ry_max = 0;
////////////////////////////////////////////////////////////////////////////
/*Initialisation of home joint position of leg joints*/
float LEG::home_pos[3] = {0, -M_PI/6, M_PI/6};
////////////////////////////////////////////////////////////////////////////
//                         CRAWL GAIT INITIALISATION
/////////////////////////////////////////////////////////////////////////////

/*Gait matrix that defines the leg lifting sequence
  Rows depict the number of stages in the locomtion cycle. Each stage allows
  only one change. The columns depict the leg index.
  Note : 1 denotes the leg is in swing phase
         0 denotes the leg is in stance phase
         2 in the beginning indicates a body motion*/
int const GAIT::crawl_gait_matrix[8][4] =  {
                                            {0,0,0,1},
                                            {0,0,0,0},
                                            {0,1,0,0},
                                            {2,0,0,0},
                                            {0,0,1,0},
                                            {0,0,0,0},
                                            {1,0,0,0},
                                            {2,0,0,0}
                                          };
int GAIT::row = 8;
int GAIT::column = 4;

/*Each element defines the time for which a particular phase of the gait will
  continue. The time is expressed in seconds. Adding all the elements of the
  vector gives the total time taken one locomotion cycle using the given gait.
*/
float GAIT::duration_vector[8] = {1,2,1,2,1,2,1,2};

/*Sleep time to ensure simulation catches up with the running code*/
float GAIT::sleep_time = 0.1;
////////////////////////////////////////////////////////////////////////////
