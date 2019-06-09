#include <cassert>
#include <cmath>
#include "parkingpathsolver.h"
#include "circle.h"

void ParkingPathSolver::solve() {
  debug_circles.clear();
  debug_circles.reserve(3);

  //TODO calculate from car parameters / set as class members

  VehicleState p;

  Circle s_circle(starting_state.x - max_turn_radius_l * cos(starting_state.angle),
                  starting_state.y + max_turn_radius_l * sin(starting_state.angle),
                  max_turn_radius_l);

  Circle g_circle(goal_pre_end_state.x - max_turn_radius_l * cos(goal_pre_end_state.angle),
                  goal_pre_end_state.y + max_turn_radius_l * sin(goal_pre_end_state.angle),
                  max_turn_radius_l);

  // now searching for the center c_p of circle p.
  // c_p is one of the two intersection points of
  // the circles s and g with radius (max_turn_radius_r + max_turn_radius_l)

  double circle_center_x = (g_circle.x + s_circle.x) / 2;
  double circle_center_y = (g_circle.y + s_circle.y) / 2;
  double dX = (s_circle.x - g_circle.x);
  double dY = (s_circle.y - g_circle.y);
  double len = sqrt(dX * dX + dY * dY);

  double big_radius = max_turn_radius_l + max_turn_radius_r;

  double h = sqrt(big_radius * big_radius - (len / 2) * (len / 2));

  Circle p_circle(circle_center_x + (h * dY / len),
                  circle_center_y - (h * dX / len),
                  max_turn_radius_r);

  assert(p_circle.x > s_circle.x);
  assert(p_circle.x > g_circle.x);
  assert(p_circle.y >= s_circle.y);


  //find the "middle" point with barycentric coordinates (due to different turn_radius')
  p.x = (max_turn_radius_l * p_circle.x + max_turn_radius_r * g_circle.x) / big_radius;
  p.y = (max_turn_radius_l * p_circle.y + max_turn_radius_r * g_circle.y) / big_radius;


  //calculate the angle of the car when it reaches the turning point (p)
  double norm_x = g_circle.y - p_circle.y;
  double norm_y = p_circle.x - g_circle.x;

  len = sqrt(norm_x * norm_x + norm_y * norm_y);

  p.angle = asin(norm_x / len);

  assert(p.angle < 0);
  assert(p.angle > -M_PI_2);

  turning_point_angle = p.angle;

  //check how much we have to drive ahead (steering left) to reach the contact point with the first circle
  norm_x = s_circle.y - p_circle.y;
  norm_y = p_circle.x - s_circle.x;

  assert(norm_x < 0);

  len = sqrt(norm_x * norm_x + norm_y * norm_y);
  pre_begin_angle = asin(norm_x / len);
  assert(pre_begin_angle <= starting_state.angle);

  debug_circles.push_back(s_circle);
  debug_circles.push_back(p_circle);
  debug_circles.push_back(g_circle);
}
