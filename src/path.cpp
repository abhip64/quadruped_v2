/**@file
  *Associated functions for path defintion and tracking
 */
////////////////////////////////////////////////////////////////////////////////
#include "path.h"
////////////////////////////////////////////////////////////////////////////////
#define TOL    0.01            /*Acceptable error for convergence*/
#define dx     0.001           /*Step size for deteriminning differentials*/
#define offset 0.1             /*Maximum allowable error form the end position*/
////////////////////////////////////////////////////////////////////////////////

/*Variable defining the next point in path that the COG of the body should
*reach. It is defined as extern as it should be possible for any
*external source file to use it directly
*https://stackoverflow.com/questions/1433204/how-do-i-use-extern-to-share-variables-between-source-files
*/

/*COG position after first phase of locomotion*/
float x_cog1 = 0;
float y_cog1 = 0;

/*COG position after second phase of locomotion*/
float x_cog2 = 0;
float y_cog2 = 0;

/*Body rotation after first phase of locomotion*/
float alpha1 = 0;

/*Body rotation after second phase of locomotion*/
float alpha2 = 0;

/*Starting point of the path. By default it is 0*/
float x_start = 0,y_start = 0;

/*End point of the path. Can be changed from main node*/
float x_end,y_end;

/*Current COG position*/
float x_cog = 0,y_cog = 0;

/*Body body_stride*/
float stride = 0;

/*Inverse jacobian matrix*/
double J_inv[2][2];

///////////////////////////////////////////////////////////////////////////////

float path_define(float x,float y)
{
  /*Give the value of the function as the return statement*/
  return 0;
}

float calc_horizon(float x,float y)
{
  /*Gives the value of the horizon function as the return statement*/
  return pow((x-x_cog),2) + pow((y-y_cog),2) - pow(stride,2);
}

void set_endpoint(float x,float y)
{
  x_end = x;
  y_end = y;
}

bool calc_next_move(float xcog, float ycog,float L)
{
  x_cog = xcog;
  y_cog = ycog;

  stride = L;

  return newton_solution();

}

void calc_J_inv(float& x,float& y)
{
  /*Calculating central diffrence differential*/
  J_inv[0][0] =  (calc_horizon(x,y+dx) - calc_horizon(x,y-dx))/(2*dx);
  J_inv[1][1] =  (path_define(x+dx,y)  - path_define(x-dx,y))/(2*dx);
  J_inv[0][1] = -(path_define(x,y+dx)  - path_define(x,y-dx))/(2*dx);
  J_inv[1][0] = -(calc_horizon(x+dx,y) - calc_horizon(x-dx,y))/(2*dx);

}

bool newton_solution()
{
  float x_prev = 0;
  float y_prev = 0;

  /*Initialising for faster convergence*/
  float alpha_t = atan((path_define(x_cog+dx,y_cog) - path_define(x_cog-dx,y_cog))/(2*dx));
  float x_next = x_cog + (stride/2)*cos(alpha_t);
  float y_next = y_cog + (stride/2)*sin(alpha_t);

  /*Function values*/
  float f[2];

  do
  {
    x_prev = x_next;
    y_prev = y_next;

    calc_J_inv(x_prev,y_prev);

    f[0] = path_define(x_prev,y_prev);
    f[1] = calc_horizon(x_prev,y_prev);

    x_next = x_prev - (J_inv[0][0]*f[0] + J_inv[0][1]*f[1]);
    y_next = y_prev - (J_inv[1][0]*f[0] + J_inv[1][1]*f[1]);

  }while((fabs(x_next-x_prev)>TOL)||(fabs(y_next-y_prev)>TOL));

  alpha1 = atan((path_define(x_next+dx,y_next) - path_define(x_next-dx,y_next))/(2*dx));

  if(fabs(x_end - x_next)<offset&&fabs(y_end - y_next)<offset)
  return 0;

  x_cog1 = x_next;
  y_cog1 = y_next;

  x_cog = x_cog1;
  y_cog = y_cog1;

  /*Initialising for second phase of locomotion using the first solution*/
  alpha_t = atan((path_define(x_cog+dx,y_cog) - path_define(x_cog-dx,y_cog))/(2*dx));
  x_next = x_cog + (stride/2)*cos(alpha_t);
  y_next = y_cog + (stride/2)*sin(alpha_t);

  do
  {
    x_prev = x_next;
    y_prev = y_next;

    calc_J_inv(x_prev,y_prev);

    f[0] = path_define(x_prev,y_prev);
    f[1] = calc_horizon(x_prev,y_prev);

    x_next = x_prev - (J_inv[0][0]*f[0] + J_inv[0][1]*f[1]);
    y_next = y_prev - (J_inv[1][0]*f[0] + J_inv[1][1]*f[1]);

  }while((fabs(x_next-x_prev)>TOL)||(fabs(y_next-y_prev)>TOL));

  alpha2 = atan((path_define(x_next+dx,y_next) - path_define(x_next-dx,y_next))/(2*dx));

  if(fabs(x_end - x_next)<0.1&&fabs(y_end - y_next)<0.1)
  return 0;

  x_cog2 = x_next;
  y_cog2 = y_next;

  return 1;

}
