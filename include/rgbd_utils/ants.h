/*
 * ants.h
 *
 *  Created on: Sep 27, 2012
 *      Author: lengelhan
 */

#ifndef ANTS_H_
#define ANTS_H_


#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, boost::no_property, boost::property < boost::edge_weight_t, float > > graph_t;
typedef boost::graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits < graph_t >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;



struct Edge_cmp
{
 bool operator()(const Edge& e1,const Edge& e2) const
 {
  if (e1.first == e2.first)
   return e1.second < e2.second;
  else
   return e1.first < e2.first;
 }
};

enum EDGE_TYPE {EDGE_NORMAL, EDGE_WATER, EDGE_TOO_STEEP, EDGE_NONE};

typedef std::pair<float, EDGE_TYPE> Edge_info;


typedef std::map<Edge, Edge_info,Edge_cmp> Edge_map;



class Path_planner {

 // TODO: parameter
 /// maximal depth that can be traversed
 float allowed_water_depth;

 /// cost to go to a cell with more than C_allowed_water_depth water
 float untraversable_cost;

 /// information on each edge
 Edge_map edge_map;

 /// height of each point in the grid
 cv::Mat height_map;
 cv::Mat water_map;


 /// policy as CV_32FC2-image. For every point the direction towards the goal is stored
 cv::Mat policy;

 /// position in map to which the current policy is directing
 cv::Point goal_;


 std::vector<Edge> edges;

 std::vector<float> edge_costs;

 /// maximal steepness that is still traversable
 float max_angle_deg;

 float height_cost_factor;

 float uphill_factor;

 cv::Mat normed;

 /// true if only four neighbours should be used
 bool use_four_neighbours;

 EDGE_TYPE addEdges(const int current_id, const cv::Point& neighbour, float current_height);

 float cost_height(float current, float neighbour);
 float cost_hillside(int x, int y);



 bool apply_smoothing;


 float hillside_cost_factor, path_length_factor;

 std::vector<cv::Point> path;
 std::vector<cv::Vec3b> path_colors;
 std::vector<Edge_info> used_edges;

public:


 bool policy_computed;

 void saveHillSideCostImage();


 Path_planner(): use_four_neighbours(false){
  allowed_water_depth = 0.001;
  untraversable_cost = 100000;
  policy_computed = false;
 }

 /**
 * @param height height map as CV_32FC1 image
 */
 void setHeightMap(const cv::Mat& height){
  assert(height.type() == CV_32FC1);
  height.copyTo(height_map);
  policy_computed = false;
 }

 /**
 * @param depth depth map as CV_64FC1 image
 */
 void setWaterDepthMap(const cv::Mat& depth){
  assert(depth.type() == CV_64FC1);
  depth.copyTo(water_map);
  policy_computed = false;
 }



 /// returns pointer to the path
 std::vector<cv::Point> getPath(){return path;}
 std::vector<cv::Vec3b> getPathColor(){return path_colors;}
 std::vector<Edge_info> getPathEdges(){return used_edges;}


 Edge_map getEdgeInformation(){return edge_map;}


 /// Setting the new maximal steepness
 void setMaxSteepness(float angle_deg){ max_angle_deg = angle_deg;}

 /// setting cost factor for cost to climb or descend to different altitude
 void setHeightCostFactor(float factor) { height_cost_factor = factor;}

 /// cost factor for walking on a hillside
 void setHillSideCostFactor(float factor) { hillside_cost_factor = factor;}

 /// setter or enabling smooting function on height- and water-depth map
 void setSmoothing(bool do_smoothing){ apply_smoothing = do_smoothing;}

 /// setter for uphill factor (factor of 2 means that going up costs twice the amount of going down)
 void setUphillfactor(float factor){ uphill_factor = factor;}

 /// cost for distance is comuted as path_length_factor*(1 for direct neighbour or sqrt(2) for diagonal neighbour)
 void setPathLengthFactor(float factor) { path_length_factor = factor;}


 /**
 * @param goal   goal position withing height map to which the routes are computed
 * @param cell_size_m  edge_length in m of a cell
 */
 /// Computation of a policy for the whole heightmap towards the goal point
 void computePolicy(cv::Point goal, float cell_size_m = 1);

 cv::Mat getPolicy(){ return policy; }

 /**
 * @param start  position in grid from which the path is planned
 * @return true if there is a way from the start to the goal
 */
 /// Path from the start-position to the goal position is printed
 bool computePath(cv::Point start);


 /// setter to chose between four and eight neighbours in search graph
 void setFourNeighbours(bool four_neighbours){use_four_neighbours = four_neighbours;}

};



/**
* Possible states of the ant
* @see Ant, Ant_2
*/
enum Ant_state {ANT_AT_GOAL, ANT_IN_FRONT_OF_OBSTACLE, ANT_MOVING, ANT_NOT_INITIALIZED};


class Ant {
 int id;

 bool pendulum;

 /// current position on path
 uint pos;

 std::vector<cv::Point> path;
 std::vector<Edge_info> used_edges;

 float bank_account;

 Ant_state state;

public:

 Ant(){
  pendulum = false;
  state = ANT_NOT_INITIALIZED;
  bank_account = 0;
 }

 int getId(){return id;}
 void setId(int id){this->id = id;}


 int getPosInPath(){return pos;}

 uint getPathLenth(){return path.size();}

 Ant_state getState(){return state;}

 void setPath(const std::vector<cv::Point>& path, const std::vector<Edge_info>& edges){
  assert(path.size() == edges.size()+1);
  this->path = path;
  this->used_edges = edges;
  pos = 0;
  state = ANT_MOVING;
 }


 cv::Point getPositionAt(uint step){assert(step < path.size()); return path[step];}
 cv::Point getGoal(){return path[path.size()-1];}
 cv::Point getStart(){return path[0];}
 cv::Point getPosition(){
  if (state == ANT_AT_GOAL)
   return getGoal();
  else{
   assert(pos < path.size());
   return path[pos];
  }
 }

 void walk(float new_funding);

};




//struct Ant_Controller {
//
// cv::Point start;
// cv::Point goal;
//
// std::map<int,Ant> ants;
//
//
//
//
//
//};


#endif /* ANTS_H_ */
