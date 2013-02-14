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
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include "math.h"

#include <pcl_ros/point_cloud.h>
#include "type_definitions.h"  // only for timing functions timing_end, timing_start


typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, boost::no_property, boost::property < boost::edge_weight_t, float > > graph_t;
typedef boost::graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits < graph_t >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

typedef pcl::PointXYZRGB pcl_Point;
typedef pcl::PointCloud<pcl_Point> Cloud;




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


typedef std::map<Edge, Edge_info, Edge_cmp> Edge_map;


struct Enemy {
  float strength;
  float radius;
  float x,y;

  void scale(float s){
    strength*=s;
    radius*=s;
    x*=s;
    y*=s;
  }
};


struct Path_planner {

  // TODO: parameter
  /// maximal depth that can be traversed
  float allowed_water_depth;

  /// cost to go to a cell with more than C_allowed_water_depth water
  float untraversable_cost;

  /// information on each edge
  Edge_map edge_map;

  /// distance to the goal for every vertex in the graph
  std::vector<float> distance_map_;

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

  bool with_water;






  float hillside_cost_factor, path_length_factor;

  std::vector<cv::Point> path;
  std::vector<cv::Vec3b> path_colors;
  std::vector<Edge_info> used_edges;

  std::vector<Enemy> enemies;
  cv::Mat enemy_cost;

public:


  void getDistanceImage(cv::Mat & img, float *threshold = NULL);

  float scale;

  void setScale(float new_scale){scale = new_scale;}

  void printParameters();


  void getRangeOfMotion(const cv::Mat& distanceMap, float threshold, std::vector<cv::Point>& contour);
  void getDistanceMap(cv::Mat& map, bool normed = true);

  /// height of each point in the grid
  cv::Mat height_map;

  float enemy_factor;

  void addEnemy(float strength, int radius, int x, int y);

  void removeEnemies(){
    enemies.clear();
    enemy_cost.setTo(0);
  }


  bool apply_smoothing;

  bool policy_computed;

  void saveHillSideCostImage();


  Path_planner(): use_four_neighbours(false){
    allowed_water_depth = 0.001;
    untraversable_cost = 100000;
    policy_computed = false;
    apply_smoothing = false;
    with_water = false;


    max_angle_deg = 45;
    height_cost_factor = 1;
    hillside_cost_factor = 1;
    uphill_factor = 1;
    path_length_factor = 3;
    enemy_factor = 1;
    scale = 1;
  }

  /**
 * @param height height map as CV_32FC1 image
 */
  void setHeightMap(const cv::Mat& height, float cell_size_m = 0.05, bool compute_model = true){
    assert(height.type() == CV_32FC1);
    height.copyTo(height_map);

    if (compute_model)
      model = createModel();



    // if (abs(scale-1) > 0.001){
    ROS_INFO("SCaling height map: %f", scale);
    cv::resize(height_map, height_map,cv::Size(), scale, scale, CV_INTER_CUBIC);
    // }


    policy_computed = false;
    cell_size_m_ = cell_size_m;

    enemy_cost = cv::Mat(height_map.size(),CV_32FC1);
    enemy_cost.setTo(0);


  }

  /**
 * @param depth depth map as CV_32FC1 image
 */
  void setWaterDepthMap(const cv::Mat& depth){
    assert(depth.type() == CV_32FC1);
    depth.copyTo(water_map);
    policy_computed = false;
    with_water = true;
  }

  Cloud model;

  float cell_size_m_;

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
 */
  /// Computation of a policy for the whole heightmap towards the goal point
  void computePolicy(cv::Point goal);

  cv::Mat getPolicy(){ return policy; }

  /**
 * @param start  position in grid from which the path is planned
 * @return true if there is a way from the start to the goal
 */
  /// Path from the start-position to the goal position is printed
  bool computePath(cv::Point start);

  /// returns pointer to the path
  std::vector<cv::Point> getPath(){return path;}
  std::vector<cv::Vec3b> getPathColor(){return path_colors;}
  std::vector<Edge_info> getPathEdges(){return used_edges;}


  /// setter to chose between four and eight neighbours in search graph
  void setFourNeighbours(bool four_neighbours){use_four_neighbours = four_neighbours;}


  Cloud createModel(){
    Cloud cloud;
    cloud.reserve(height_map.cols*height_map.rows);

    pcl_Point p;
    p.r = 0;
    p.g = 0;
    p.b = 255;


    pcl_Point nap; nap.x = nap.y = nap.z = std::numeric_limits<float>::quiet_NaN();

    for (int y=0; y<height_map.rows; ++y){
      p.y = y*cell_size_m_;
      for (int x=0; x<height_map.cols; ++x){
        p.x = x*cell_size_m_;
        p.z = height_map.at<float>(y,x);
        cloud.push_back(p);

      }
    }

    cloud.width = height_map.cols;
    cloud.height = height_map.rows;

    return cloud;

  }

  void createEnemyMarker(visualization_msgs::Marker& marker, int id);
  void createPathMarker(visualization_msgs::Marker& marker);
  void createRangeMarker(visualization_msgs::Marker& marker, const std::vector<cv::Point>& contour, int id = 0);


  void publishPath(ros::Publisher & pub_marker);


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



#endif /* ANTS_H_ */
