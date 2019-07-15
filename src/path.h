////////////////////////////////////////////////////////////////////////////////
#ifndef PATH_H_
#define PATH_H_
////////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Dense>
////////////////////////////////////////////////////////////////////////////////
/*Only extern variable definitions can come in the header files.
Otherwise whichever file uses this header will have a
redefinition of the same variables leading to multiple
definition error. Functions are by default extern.*/

/*Variables denoting the next COG translation and body rotation
  for the locomotion cycle*/
extern float x_cog1,y_cog1,x_cog2,y_cog2;
extern float alpha1,alpha2;

/**
*Defintion of the path that the c.g of the quadruped has to follow.
*The path has to be a 2D path.
*@param x - x coordinate at which function is to be evaluated
*@param y - y coordinate at which function is to be evaluated
*@return Value of the function at the given x and y coordinate
*@todo Add the capability to define a 3D path
*/
float path_define(float,float);

/**
  *Calculates the value of the horizon function which defines the
  *reachable horizon of the COG of the quadruped in one body stride
  *@param x - x coordinate
  *@param y - y coordinate
  *@retun Value of horizon function at the given coordinates
 */
float calc_horizon(float,float);

/**
  *Calculate the next position of the COG along the path given the
  *current position of the COG of the quadruped. This calculation
  *is done using newton's solution of non-linear equations
*/
void next_pos_in_path();

/**Newton's method of solving a system of non-linear equations
  *https://www.youtube.com/watch?v=-Ws7cEH7w_U
 */
bool newton_solution();

/**
  *Determine the next point in the path that the cog of the quadruped
  *has to move to. Procedure of determining the solution will be explained
  *in the associated github wiki page.
  *@param xcog - x coordinate of COG
  *@param ycog - y coordinate of COG
  *@param L    - body stride length
  *@return 0 denotes path completion and 1 denotes one cycle of locomotion
  *is completed.
 */
bool calc_next_move(float,float,float);

/**
  *Calculates the Jacobian inverse for newton's algorithm
  *@param x - x coordinate
  *@param y - y coordinate
 */
void calc_J_inv(float&,float&);

/**
  *Set the end point of COG on the path. Differentials are calculated
  *using central difference method
  *@param x - x coordinate of end point
  *@param y - y coordinate of end point
 */
void set_endpoint(float,float);

////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
