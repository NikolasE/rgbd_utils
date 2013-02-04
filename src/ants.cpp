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
* cost depending on local shape.
*
* Walking in a plane is more expensive than walking on an equipotential line around a mountain
*
* @param x
* @param y
* @return local_cost_factor*(mean of absolute height differences to neighbouring cells)
*/
float Path_planner::cost_hillside(int x, int y){

  float h = normed.at<float>(y,x);


  // neighbours with manhattan distance not larger than l are taken into account
  int l = 1;

  int valid_cell_cnt = 0;
  float height_abs_diff_sum = 0;

  for (int dx = -l; dx <= l; ++dx)
    for (int dy = -l; dy <= l; ++dy){

      if (dx == 0 && y == 0) continue;

      int x_ = x+dx; int y_ = y+dy;

      if (x_<0 || y_<0 || x_ >= normed.cols || y_ >= normed.rows)
        continue;

      float h_ = normed.at<float>(y_,x_);

      if (h_ == h_){
        valid_cell_cnt++;
        height_abs_diff_sum += abs(h_-h);
      }

    }

  float cost = 0;

  // there will be no edge to this position
  if (valid_cell_cnt == 0)
    return cost;


  cost = height_abs_diff_sum/valid_cell_cnt;


  return hillside_cost_factor*cost;


}


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
float Path_planner::cost_height(float current, float neighbour){


  if (abs(max_angle_deg) < 1e-3){
    // ROS_WARN("max_angle_deg is zero!");
    return 0;
  }

  //ROS_INFO("Height: %f %f", uphill_factor,height_cost_factor);

  float diff = neighbour-current;

  float angle = atan(diff)/M_PI*180;

  // not traversable (to steep to ascend or descend)
  if (abs(angle) > max_angle_deg)
    return -1;

  // ROS_INFO("heights: %f %f Angle: %f",current, neighbour, angle);


  // simple linear interpolation for cost definition

  //     angle     | cost
  // - max_angle   |   0.5
  //      0        |    1
  //   max_angle   |   1.5

  float cost = (angle+max_angle_deg)/(2*max_angle_deg)+0.5;


  /*
  * Asymetric cost. Otherwise, climbing a hill and going down again is like walking through a plane
  */
  // if (neighbour > current) cost *= uphill_factor;

  // assert(0.5 <= cost && cost <= 1.5);

  return cost*height_cost_factor;

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
* @return if edges were added (neighbour within height map and relative height not to large)
*/
inline EDGE_TYPE Path_planner::addEdges(const int current_id, const cv::Point& neighbour, float current_height){

  if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= normed.cols || neighbour.y >= normed.rows)
    return EDGE_NONE;

  //assert(neighbour.x < normed.cols && neighbour.y < normed.rows);

  float neighbour_height = normed.at<float>(neighbour.y,neighbour.x);

  int neighbour_id = pos2id(neighbour, normed.cols);

  cv::Point current = id2pos(current_id, normed.cols);


  Edge edge(current_id, neighbour_id);


  if (with_water){
    double water_depth = water_map.at<double>(neighbour.y, neighbour.x);

    if (water_depth > allowed_water_depth){
      edges.push_back(edge);
      edge_costs.push_back(untraversable_cost);
      edge_map[edge] = Edge_info(untraversable_cost,EDGE_WATER);
      return EDGE_WATER;
    }

  }



  float height_cost = cost_height(current_height, neighbour_height);

  // height_cost is negative if the height difference is too large
  if (height_cost < 0){
    edges.push_back(edge);
    edge_costs.push_back(untraversable_cost);
    edge_map[edge] = Edge_info(untraversable_cost,EDGE_TOO_STEEP);
    return EDGE_TOO_STEEP;
  }


  float hillside_cost = cost_hillside(current.x, current.y);


  float enemy_costs = enemy_cost.at<float>(neighbour.x,neighbour.y)*enemy_factor;


  // ROS_INFO("enemz: %f", enemy_costs);

  float dist_cost = path_length_factor*sqrt(pow(neighbour.x - current.x,2)+pow(neighbour.y - current.y,2));
  float total_cost = height_cost+dist_cost+hillside_cost+enemy_costs*enemy_factor;


  // ROS_INFO("Height: %f, dist: %f, hill: %f", height_cost, dist_cost, hillside_cost);

  edges.push_back(edge);
  edge_costs.push_back(total_cost);
  edge_map[edge] = Edge_info(total_cost,EDGE_NORMAL);

  return EDGE_NORMAL;

}


void Path_planner::addEnemy(float strength, int radius, int x, int y){
  //Enemy e; e.strength = strength; e.x = x; e.y = y;


  cv::Mat img = cv::Mat(height_map.size(), CV_32FC1);
  img.setTo(0);

  //  img.at<float>(y,x) = strength;


  int r = radius/2;
  if (r%2==1)
    r+=1;


  cv::circle(img,cv::Point(x,y),r,CV_RGB(strength,strength,strength),-1);

  // cv::GaussianBlur(img,img,cv::Size(radius,radius),radius);

  cv::blur(img,img,cv::Size(r,r));

  enemy_cost += img;

  double min_, max_;
  cv::minMaxLoc(img, &min_,&max_);
  // cv::Mat scaled = (h2-min_)/(max_-min_);

  ROS_INFO("min: %f %f", min_, max_);

  cv::namedWindow("enemy");

  cv::Mat scaled = (img-min_)/(max_-min_);

  cv::resize(scaled, scaled, cv::Size(),3,3);

  cv::imshow("enemy",scaled);





}


void Path_planner::saveHillSideCostImage(){


  cv::Mat img(normed.rows, normed.cols, CV_8UC3);

  float max_cost = 1; // given in cell_distances

  // ROS_INFO("img: %i %i", normed.rows, normed.cols);

  for (int x = 0; x<normed.cols; ++x)
    for (int y = 0; y<normed.rows; ++y){
      float c = cost_hillside(x,y)/hillside_cost_factor;

      //   ROS_INFO("x,y: %i %i: %f", x,y,c);

      img.at<cv::Vec3b>(y,x) = cv::Vec3b(c/max_cost*255,c/max_cost*255,c/max_cost*255);
    }

  cv::imwrite("hillside_cost.png", img);

}

void Path_planner::createPathMarker(visualization_msgs::Marker& marker){


  marker.points.clear();
  marker.colors.clear();

  marker.header.frame_id = "/fixed_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "points_and_lines";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = cell_size_m_/2;

  marker.color.r = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::Point p;



  std_msgs::ColorRGBA col;
  col.a = 1;

  for (uint i=0; i<path.size(); ++i){
    pcl_Point  P = model.at(path[i].x,path[i].y);

    col.r = path_colors[i].val[0]/255;
    col.g = path_colors[i].val[1]/255;
    col.b = path_colors[i].val[2]/255;

    marker.colors.push_back(col);

    p.x = P.x;
    p.y = P.y;
    p.z = P.z;

    marker.points.push_back(p);
  }

}




/**
*
* @param goal   goal position withing height map to which the routes are computed
* @param cell_size_m  edge_length in m of a cell
* @todo separate graph construction from path planning
*/
void Path_planner::computePolicy(cv::Point goal){

  // ROS_INFO("computePolicy: %i %i", goal_.x, goal_.y);

  double min_, max_;
  cv::minMaxLoc(enemy_cost, &min_,&max_);


  ROS_INFO("enemy: %f %f", min_, max_);

  goal_ = goal;

  // ros::Time start_time = ros::Time::now();

  assert(height_map.rows > 0 && height_map.cols > 0);
  assert(goal_.x < height_map.cols && goal_.y < height_map.rows);


  edge_costs.clear();
  edges.clear();
  edge_map.clear();

  // normalize so that points with the same height have a distance of 1 and heights
  // are measured in multiples of the cell size
  normed = height_map/cell_size_m_;


  if (apply_smoothing){
    cv::GaussianBlur(normed, normed, cv::Size(3,3),1,1);
    ROS_INFO("With Smoothing");
  }


  int width = height_map.cols;

  // number of nodes
  int num_nodes = height_map.cols*height_map.rows;

  cv::Point current;

  uint valid_transitions = 0;


  // // number of normal edges
  // int edge_normal_cnt = 0;
  // // number of water edges
  // int edge_water_cnt = 0;
  // // number of edges that were not added due to
  // int edge_none_cnt = 0;

  for (int x=0; x<normed.cols; ++x){
    current.x = x;
    for (int y=0; y<normed.rows; ++y){
      current.y = y;
      uint valid_current = 0;

      float current_height = normed.at<float>(y,x);
      int current_id = pos2id(x,y, width);


      //   if (use_four_neighbours){
      //    // upwards (addEdges returns false if neighboring pixel is not within image
      //    if (addEdges(current_id, cv::Point(x,y-1), current_height) == EDGE_NORMAL){ valid_transitions++; valid_current++;}

      //    // downwards
      //    if (addEdges(current_id, cv::Point(x,y+1), current_height) == EDGE_NORMAL){ valid_transitions++; valid_current++;}

      //    // left
      //    if (addEdges(current_id, cv::Point(x-1,y), current_height) == EDGE_NORMAL){ valid_transitions++; valid_current++;}

      //    // right
      //    if (addEdges(current_id, cv::Point(x+1,y), current_height) == EDGE_NORMAL){ valid_transitions++; valid_current++;}
      //   }else{

      // all neighbors
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++){
          if (dx == 0 && dy == 0) continue;

          if (use_four_neighbours && (abs(dx)+abs(dy)) > 1)
            continue;


          if (addEdges(current_id, cv::Point(x+dx,y+dy), current_height) == EDGE_NORMAL){
            valid_transitions++; valid_current++;
          }
          //     }
        }

    }

  }

  // ROS_INFO("Valids: %f")
  assert(edges.size() == edge_costs.size());

  // ROS_INFO("Created grid with %zu edges", edges.size());

  graph_t g(&edges[0], &edges[0] + edges.size(), &edge_costs[0], num_nodes);

  // computation of path
  std::vector<vertex_descriptor> p(num_vertices(g));
  std::vector<int> d(num_vertices(g));
  vertex_descriptor s = vertex(pos2id(goal_.x, goal_.y,width), g);


  dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));

  ROS_INFO("called dijkstra");


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



  policy_computed = true;
  // return policy;

}


bool Path_planner::computePath(cv::Point start){
  assert(start.x >=0 && start.x < policy.cols);

  if (!policy_computed){
    ROS_FATAL("Trying to compute Path without policy");
    assert(policy_computed);
  }

  // ROS_INFO("start: %i %i, end: %i %i", start.x, start.y, goal_.x, goal_.y);

  //#define SAVE_PATH

#ifdef SAVE_PATH
  // draw heightmap as 8UC3 image
  cv::Mat img = cv::Mat(height_map.rows, height_map.cols, CV_8UC3);
  for (int x = 0; x<height_map.cols; ++x)
    for (int y = 0; y<height_map.rows; ++y){

      float h = height_map.at<float>(y,x)*255;
      img.at<cv::Vec3b>(y,x) = cv::Vec3b(h,h,h);

    }

  cv::imwrite("height_map.png",img*255);
#endif


  path.clear();
  path_colors.clear();
  used_edges.clear();


  float path_costs = 0;

  path.push_back(cv::Point(start.x,start.y));
  path_colors.push_back(cv::Vec3b(255,0,0));

  cv::Point current = start;
  while (current.x != goal_.x || current.y != goal_.y){

    cv::Vec2f dir = policy.at<cv::Vec2f>(current.y,current.x);

    if (dir.val[0] == 0 && dir.val[1] == 0){
      ROS_INFO("Path stopped at %i %i", current.x, current.y);

#ifdef SAVE_PATH
      cv::imwrite("path_partial.png",img);
#endif

      return false;
    }

    int current_id = pos2id(current, policy.cols);

    current.x += dir.val[0];
    current.y += dir.val[1];

    int next_id = pos2id(current, policy.cols);

    Edge used_edge(current_id, next_id);

    assert(edge_map.find(used_edge) != edge_map.end());

    Edge_info edge_info = edge_map.at(used_edge);

    path_costs += edge_info.first;

    cv::Vec3b color;


    // show different edge types with different colors
    if (edge_info.second == EDGE_NORMAL)
      color = cv::Vec3b(0,255,0); // green

    if (edge_info.second == EDGE_WATER)
      color = cv::Vec3b(0,0,255); // blue

    if (edge_info.second == EDGE_TOO_STEEP)
      color = cv::Vec3b(255,0,0); // red

#ifdef SAVE_PATH
    img.at<cv::Vec3b>(current.y, current.x) = color;
#endif

    path_colors.push_back(color);
    path.push_back(cv::Point(current.x,current.y));
    used_edges.push_back(edge_info);
  }

  ROS_INFO("Path length: %zu, col length: %zu, cost: %f", path.size(), path_colors.size(), path_costs);

  assert(path.size() == path_colors.size() && used_edges.size() == path.size()-1);

#ifdef SAVE_PATH
  cv::imwrite("path.png",img);
#endif


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
  planner.computePolicy(goal);

  // cv::Mat policy = planner.getPolicy();
  planner.computePath(cv::Point(x,y));
  // printPath(cv::Point(x,y), goal, policy, height);
  // cv::imwrite("after.png", height);


}


void Ant::walk(float new_funding){
  assert(new_funding >=0);

  if (state == ANT_NOT_INITIALIZED){
    ROS_INFO("Trying to move an uninitialized Ant");
    return;
  }

  if (state != ANT_MOVING)
    return;


  assert(pos < used_edges.size());
  Edge_info edge = used_edges[pos];

  float cost = edge.first;

  // check if edge is traversable
  if (edge.second != EDGE_NORMAL){
    state = ANT_IN_FRONT_OF_OBSTACLE;
    return;
  }

  // new funding for next step (only added if next edge is traversable)
  bank_account += new_funding;


  // check if there is enough money to pay for this step
  if (cost > bank_account){
    //  ROS_INFO("Next step would cost %f, but I only have %f", cost, bank_account);
    return;
  }


  // next edge is traversable and also affordable

  pos++;
  bank_account -= cost; // pay for step
  assert(bank_account>=0);
  // ROS_INFO("Just payed %f for one step, I still have %f", cost, bank_account);

  if (pos == path.size()-1){

    if (pendulum){

      std::reverse(path.begin(), path.end());
      std::reverse(used_edges.begin(), used_edges.end());

      pos = 0;
      bank_account = 0;

    }else{
      state = ANT_AT_GOAL;
    }

    return;
  }


  // take next step (without new funding)
  walk(0);


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


//int main(int argc, char ** argv){

// testDijkstra(atoi(argv[1]),atoi(argv[2]));
// return 1;

//}
