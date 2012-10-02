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


class Path_planner {

 /// height of each point in the grid
 cv::Mat height_map;

 /// policy as CV_32FC2-image. For every point the direction towards the goal is stored
 cv::Mat policy;

 /// position in map to which the current policy is directing
 cv::Point goal_;


 /// maximal steepness that is still traversable
 float max_angle_deg;

 float cost_factor;

 float uphill_factor;

 /// true if only four neighbours should be used
 bool use_four_neighbours;

 bool addEdges(const int current_id, const cv::Point& neighbour, float current_height, cv::Mat* map, std::vector<Edge>* edges,std::vector<float>* edge_costs);

 float cost(float current, float neighbour);

public:


 Path_planner(): use_four_neighbours(true){}

 /**
  * @param height height map as CV_32FC1 image
  */
 void setHeightMap(const cv::Mat& height){
  height.copyTo(height_map);
 }


 /// Setting the new maximal steepness
 void setMaxSteepness(float angle_deg){  max_angle_deg = angle_deg; }

 void setCostFactor(float factor) {cost_factor = factor;}

 /**
  * @param goal   goal position withing height map to which the routes are computed
  * @param cell_size_m  edge_length in m of a cell
  */
 /// Computation of a policy for the whole heightmap towards the goal point
 void computePolicy(cv::Point goal, float cell_size_m = 1);

 cv::Mat getPolicy(){ return policy; }

 /**
  * @param start  position in grid from which the path is planned
  * @return true iff there is a way from the start to the goal
  */
 /// Path from the start-position to the goal position is printed
 bool printPath(cv::Point start);


 void setFourNeighbours(bool four_neighbours){use_four_neighbours = four_neighbours;}
 void setUphillfactor(float factor){ uphill_factor = factor;}
};



//cv::Mat getPolicy(const cv::Mat& height_map, cv::Point goal, float cell_size = 1);

//inline cv::Point id2pos(int id, int width);
//inline int pos2id(int x, int y, int width);
//inline int pos2id(const cv::Point& p, int width);
//void testBoost();


#endif /* ANTS_H_ */
