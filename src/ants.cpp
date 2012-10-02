/*
 * ants.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: lengelhan
 */

#include "rgbd_utils/ants.h"


using namespace boost;
using namespace std;



/**
*
* Assumes that neighbouring position have a horizontal distance of 1 and that the
* height is also given in this dimension s.t. a vertical distance of 1 corresponds to an 45deg ascend
*
* @param current    height of current position
* @param neighbour  height of neighbouring position
* @return           cost to go to the neighbouring cell, -1 if ascend/descend is untraversable
* @todo parameterize (max ascend,...)
*/
/// computation of weight to go from current height to neighbouring height
float Path_planner::cost(float current, float neighbour){

 float diff = neighbour-current;

 float angle = atan(diff)/M_PI*180;

 //  float max_angle_deg = 45;

 //  ROS_INFO("heights: %f %f Angle: %f",current, neighbour, angle);

 // not traversable (to steep to ascend or descend)
 if (abs(angle) > max_angle_deg)
  return -1;

 // simple linear interpolation for cost definition

 //     angle     | cost
 // - max_angle:  |   0.5
 //      0        |    1
 //   max_angle   |   1.5

 float cost = (angle+max_angle_deg)/(2*max_angle_deg)+0.5;


 /*
  * Asymetric cost. Otherwise, climbing a hill and going down again is like walking through a plane
  */
  if (neighbour > current > 0) cost *= uphill_factor;

 // assert(0.5 <= cost && cost <= 1.5);

 return cost;

}





inline cv::Point id2pos(int id, int width){
 cv::Point p;
 p.x = id%width;
 p.y = id/width;
 return p;
}

inline int pos2id(int x, int y, int width){
 return y*width+x;
}

inline int pos2id(const cv::Point& p, int width){
 return pos2id(p.x,p.y,width);
}


/**
*
* Accepts wrong values for neighbour (like (-1,-1)),
* @param current_id
* @param neighbour
* @param current_height
* @param map
* @param edges
* @param edge_costs
* @return if edges were added (neighbour within height map and relative height not to large)
*/
inline bool Path_planner::addEdges(const int current_id, const cv::Point& neighbour, float current_height, cv::Mat* map, vector<Edge>* edges,vector<float>* edge_costs){

 if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map->cols || neighbour.y >= map->rows)
  return false;

 //assert(neighbour.x < map->cols && neighbour.y < map->rows);

 float neighbour_height = map->at<float>(neighbour.y,neighbour.x);

 int neighbour_id = pos2id(neighbour, map->cols);


 cv::Point current = id2pos(current_id, map->cols);



 float dist_cost = sqrt(pow(neighbour.x - current.x,2)+pow(neighbour.y - current.y,2));


 float height_cost = cost(current_height, neighbour_height);

 // if (height_cost!=100)
 //  ROS_INFO("dist: %f, height: %f", dist_cost, height_cost);

 // cout << height_cost << endl;
 if (height_cost >= 0) {
  edges->push_back(Edge(current_id, neighbour_id));
  edge_costs->push_back(height_cost+dist_cost);
 }

 height_cost = cost(neighbour_height, current_height);
 if (height_cost >= 0) {
  edges->push_back(Edge(neighbour_id, current_id));
  edge_costs->push_back(height_cost+dist_cost);
 }

 return true;
}





/**
*
* @param goal   goal position withing height map to which the routes are computed
* @param cell_size_m  edge_length in m of a cell
*/
void Path_planner::computePolicy(cv::Point goal, float cell_size_m){

 goal_ = goal;

 // ros::Time start_time = ros::Time::now();

 assert(height_map.rows > 0 && height_map.cols > 0);
 assert(goal_.x < height_map.cols && goal_.y < height_map.rows);


 // normalize so that points with the same height have a distance of 1 and heights
 // are measured in multiples of the cell size
 cv::Mat normed = height_map/cell_size_m;


 vector<Edge> edges;
 vector<float> edge_costs;

 int width = height_map.cols;

 // number of nodes
 int num_nodes = height_map.cols*height_map.rows;

 cv::Point current;

 uint valid_transitions = 0;


 for (int x=0; x<normed.cols; ++x){
  current.x = x;
  for (int y=0; y<normed.rows; ++y){
   current.y = y;
   uint valid_current = 0;

   float current_height = normed.at<float>(y,x);
   int current_id = pos2id(x,y, width);


   if (use_four_neighbours){
    // upwards (addEdges returns false if neighboring pixel is not within image
    if (addEdges(current_id, cv::Point(x,y-1), current_height, &normed, &edges, &edge_costs)){ valid_transitions++; valid_current++;}

    // downwards
    if (addEdges(current_id, cv::Point(x,y+1), current_height, &normed, &edges, &edge_costs)){ valid_transitions++; valid_current++;}

    // left
    if (addEdges(current_id, cv::Point(x-1,y), current_height, &normed, &edges, &edge_costs)){ valid_transitions++; valid_current++;}

    // right
    if (addEdges(current_id, cv::Point(x+1,y), current_height, &normed, &edges, &edge_costs)){ valid_transitions++; valid_current++;}
   }else{

    // all eight neighbors
    for (int dx = -1; dx <= 1; dx++)
     for (int dy = -1; dy <= 1; dy++){
      if (dx == 0 && dy == 0) continue;

      //     ROS_INFO("dx,dy: %i %i", dx, dy);

      if (addEdges(current_id, cv::Point(x+dx,y+dy), current_height, &normed, &edges, &edge_costs)){
       valid_transitions++; valid_current++;
      }
     }
   }
   //   ROS_INFO("%i %i has %i edges", x,y,valid_current);

  }

 }

 // ROS_INFO("Valids: %f")
 assert(edges.size() == edge_costs.size());

 ROS_INFO("Created grid with %zu edges", edges.size());


 graph_t g(&edges[0], &edges[0] + edges.size(), &edge_costs[0], num_nodes);

 std::vector<vertex_descriptor> p(num_vertices(g));
 std::vector<int> d(num_vertices(g));
 vertex_descriptor s = vertex(pos2id(goal_.x, goal_.y,width), g);

 dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));

 // ros::Time end_time = ros::Time::now();

 // TODO: print time


 // resulting policy: in each pixel, the relative coordinate of the neighbouring pixel in the direction to the goal is inserted
 policy = cv::Mat(normed.rows, normed.cols, CV_32FC2);


 graph_traits < graph_t >::vertex_iterator vi, vend;
 for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
  int current = *vi;
  int best_neighbour = p[*vi];

  cv::Point cur = id2pos(current,width);
  cv::Point best = id2pos(best_neighbour,width);

  cv::Vec2f dir(best.x-cur.x,best.y-cur.y);

  policy.at<cv::Vec2f>(cur.y,cur.x) = dir;

  //  if (dir.val[0] == dir.val[1] && dir.val[0] == 0){
  //   ROS_INFO("Single node: %i %i", cur.x,cur.y);
  //  }

  //  std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
  //  std::cout << "parent(" << name[*vi] << ") = " << name[ p[*vi]] << std::
 }

 // TODO: store pathlength to goal for each cell


 // return policy;

}


bool Path_planner::printPath(cv::Point start){
 assert(start.x >=0 && start.x < policy.cols);

 cv::Point current = start;

 ROS_INFO("start: %i %i, end: %i %i", start.x, start.y, goal_.x, goal_.y);

 int length = 0;


 cv::Mat img = cv::Mat(height_map.rows, height_map.cols, CV_8UC3);
 for (int x = 0; x<height_map.cols; ++x)
  for (int y = 0; y<height_map.rows; ++y){

   float h = height_map.at<float>(y,x)*255;
   img.at<cv::Vec3b>(y,x) = cv::Vec3b(h,h,h);

  }

 cv::imwrite("height_map.png",img*255);



 while (current.x != goal_.x || current.y != goal_.y){

  cv::Vec2f dir = policy.at<cv::Vec2f>(current.y,current.x);

  if (dir.val[0] == 0 && dir.val[1] == 0){
   ROS_INFO("Path stopped at %i %i", current.x, current.y);
   cv::imwrite("path_partial.png",img);
   return false;
  }

  current.x += dir.val[0];
  current.y += dir.val[1];

  // map.at<float>(current.y, current.x) = 125;

  img.at<cv::Vec3b>(current.y, current.x) = cv::Vec3b(255,0,0);

  length++;

  // ROS_INFO("New pos: %i %i", current.x, current.y);
 }

 ROS_INFO("Path length: %i", length);

 cv::imwrite("path.png",img);

 return true;

}



void testDijkstra(int x, int y){


 Path_planner planner;

 cv::Mat height(100,100, CV_32FC1);
 height.setTo(0);


 cv::Point goal(height.cols-1,height.rows-1);
 // cv::Mat policy = getPolicy(height, goal);
 // printPath(cv::Point(0,0), goal, policy,height);

 cv::line(height, cv::Point(0,20), cv::Point(20,20), cv::Scalar::all(1),1);
 cv::line(height, cv::Point(0,60), cv::Point(70,60), cv::Scalar::all(1),1);

 cv::line(height, cv::Point(100,30), cv::Point(20,30), cv::Scalar::all(1),1);
 cv::line(height, cv::Point(100,80), cv::Point(20,80), cv::Scalar::all(1),1);
 cv::circle(height, cv::Point(50,50),5,cv::Scalar::all(1),-1);

 // cv::blur(height, height,cv::Size(2,2),cv::Point(-1,-1));
 // cv::GaussianBlur(height, height, cv::Size(11,11),1,1);

 cv::imwrite("foo.png", height);

 planner.setHeightMap(height);
 planner.computePolicy(goal,1);

 // cv::Mat policy = planner.getPolicy();
 planner.printPath(cv::Point(x,y));
 // printPath(cv::Point(x,y), goal, policy, height);
 // cv::imwrite("after.png", height);


}




//void testBoost(){
//
// const int num_nodes = 5;
// enum nodes { A, B, C, D, E };
// char name[] = "ABCDE";
// Edge edge_array[] = { Edge(A, C), /*Edge(B, B), Edge(B, D), Edge(B, E),  Edge(C, B),*/ Edge(C, D), Edge(D, E), Edge(E, A)};//, Edge(E, B)};
// float weights[] = { 1,/* 2, 1, 2, 7,*/ 3, 1, 1 /*, 1*/ };
// int num_arcs = sizeof(edge_array) / sizeof(Edge);
//
// graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
// // property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
//
// std::vector<vertex_descriptor> p(num_vertices(g));
// std::vector<int> d(num_vertices(g));
// vertex_descriptor s = vertex(A, g);
//
// dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
//
// std::cout << "distances and parents:" << std::endl;
// graph_traits < graph_t >::vertex_iterator vi, vend;
// for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
//  std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
//  std::cout << "parent(" << name[*vi] << ") = " << name[ p[*vi]] << std::
//    endl;
// }
// std::cout << std::endl;
//
//}


int main(int argc, char ** argv){

 testDijkstra(atoi(argv[1]),atoi(argv[2]));
 return 1;

}
