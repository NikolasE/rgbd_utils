/*
 * meshing.cpp
 *
 *  Created on: Apr 20, 2012
 *      Author: Nikolas Engelhard
 */

#include "rgbd_utils/meshing.h"

using namespace std;



#define MANUAL_METHOD
/**
 *
 * @param p0 first point of triangle
 * @param p1 second point
 * @param p2 third point
 * @param max_length maximal length of edge
 * @return true iff no edge of the triangle is longer than max_length
 */
bool validTriangle(const pcl_Point& p0, const pcl_Point& p1, const pcl_Point& p2, float max_length){

 if (p0.x != p0.x) return false;
 if (p1.x != p1.x) return false;
 if (p2.x != p2.x) return false;

 if (max_length < 0) return true;

 if (dist(p0,p1) > max_length) return false;
 if (dist(p0,p2) > max_length) return false;
 if (dist(p1,p2) > max_length) return false;

 return true;
}


#ifdef MANUAL_METHOD

/**
 * @param cloud input cloud that will be meshed. Cloud is assumed to be organized
 * @param max_length maximal length of edge in mesh
 * @return mesh simple mesh creation. Cloud is organized so that neighbours are well defined
 *
 */
pcl::PolygonMesh Mesh_visualizer::createMesh(const Cloud& cloud, float max_length){
 pcl::PolygonMesh mesh;


// ROS_INFO("Creating mesh for %i %i grid", cloud.width, cloud.height);

 int width = cloud.width;

 sensor_msgs::PointCloud2 msg;
 pcl::toROSMsg( cloud, msg);
 mesh.cloud = msg;

 pcl::Vertices vertices;


 for (uint x = 0; x<cloud.width-1; ++x)
  for (uint y = 0; y<cloud.height-1; ++y){

   vertices.vertices.clear();
   vertices.vertices.push_back(y*width+x);
   vertices.vertices.push_back((y+1)*width+x);
   vertices.vertices.push_back(y*width+x+1);

   // check for nan in points and max length of lines
   if (validTriangle(cloud.at(vertices.vertices[0]),cloud.at(vertices.vertices[1]),cloud.at(vertices.vertices[2]),max_length))
    mesh.polygons.push_back(vertices);


   vertices.vertices.clear();

   vertices.vertices.push_back(y*width+x+1);
   vertices.vertices.push_back((y+1)*width+x);
   vertices.vertices.push_back((y+1)*width+x+1);

   if (validTriangle(cloud.at(vertices.vertices[0]),cloud.at(vertices.vertices[1]),cloud.at(vertices.vertices[2]),max_length))
    mesh.polygons.push_back(vertices);

  }

// ROS_INFO("Mesh hast %zu triangles", mesh.polygons.size());

 return mesh;
}

#else

/**
 * @param cloud organized input cloud
 * @param mesh mesh for cloud, computed with the pcl::OrganizedFastMesh algorithm
 */
pcl::PolygonMesh Mesh_visualizer::createMesh(const Cloud& cloud){
 pcl::PolygonMesh mesh;
 if (cloud.size() == 0) {ROS_WARN("createMesh: Size 0"); return mesh;}

 ROS_INFO("Cloud has %zu points", cloud.size());

 pcl::OrganizedFastMesh<pcl_Point> mesher;

 Cloud clone = cloud;

 mesher.setInputCloud(clone.makeShared());

 // std::vector<pcl::Vertices>& polygons

 // ROS_INFO("reconstruct started");
 // ROS_INFO("%i %i", cloud.height, cloud.width);

 assert(cloud.isOrganized());

 // mesher.setMaxEdgeLength(0); // not implemented...

 mesher.reconstruct(mesh);
 ROS_INFO("Mesh has %zu triangles",mesh.polygons.size());

 return mesh;
}
#endif


/**
 *
 * @param mesh Input mesh. All triangle edges are send as red lines to RVIZ using the pub_lines_-Publisher on the "mesh_lines"-topic
 */
void Mesh_visualizer::visualizeMeshLines(const pcl::PolygonMesh& mesh){

 if (pub_.getNumSubscribers() == 0) { return;}

 Cloud cloud;
 pcl::fromROSMsg(mesh.cloud, cloud);

 visualization_msgs::Marker marker;

 marker.header.frame_id = "/openni_rgb_optical_frame";
 marker.header.stamp = ros::Time::now();

 marker.type = visualization_msgs::Marker::LINE_LIST;
 marker.action = visualization_msgs::Marker::ADD;

 marker.id = 0;

 geometry_msgs::Point p;
 std_msgs::ColorRGBA col;
 col.a = 1.0;
 col.r = 1.0;
 col.g = 0.0;
 col.b = 0.0;

 marker.scale.x = 0.02;

 marker.color = col;

 marker.lifetime = ros::Duration();

 //  ROS_INFO("Mesh has %zu triangles",mesh.polygons.size());
 for (uint i=0; i<mesh.polygons.size(); ++i){
  pcl::Vertices vtc = mesh.polygons[i];
  assert(vtc.vertices.size() == 3);

  for (uint j=0; j<=3; ++j){

   pcl_Point cp = cloud.points[vtc.vertices[j%3]];

   p.x = cp.x;
   p.y = cp.y;
   p.z = cp.z;

   marker.points.push_back(p);
   marker.colors.push_back(col);

  }

 }

 pub_lines_.publish(marker);

}


/**
 *
 * @param height_lines Heightlines that will be visualized
 * @param img Output: Each heightline is projected into the image with the given (3x4) Projectionmatrix and shown as white lines
 * @param P Projection Matrix
 */
void Mesh_visualizer::visualizeHeightLinesOnImage(const std::vector<Line_collection>& height_lines, cv::Mat& img, const cv::Mat& P){


 assert(P.rows == 3 && P.cols == 4);

 for (uint h=0; h<height_lines.size(); ++h){
  for (uint i=0; i<height_lines[h].size(); ++i){

   cv::Point2f px = applyPerspectiveTrafo(height_lines[h][i].first, P);
   cv::Point2f px2 = applyPerspectiveTrafo(height_lines[h][i].second, P);

   cv::line(img, px, px2, CV_RGB(255,255,255),3);
  }
 }
}


/**
 *
 * @param cloud input cloud
 * @param mask all points with mask(x,y)==0 are ignored
 * @param min_z Output: minimal z-value within cloud
 * @param max_z Output: maximal z-value within cloud
 */
void Mesh_visualizer::getZRangeWithinMaskArea(const Cloud& cloud, const cv::Mat& mask, float& min_z, float& max_z){

 min_z = 1e6;
 max_z = -1e6;

 for (uint x=0; x<cloud.width; ++x)
  for (uint y=0; y<cloud.height; ++y){
   if (mask.at<uchar>(y,x) == 0) continue;
   float z = cloud.at(x,y).z;
   min_z = min(min_z,z); max_z = max(max_z,z);
  }
}


/**
 *
 * @param lc All lines are visualized in RVIZ as red lines (published by pub_height_lines_ on "pub_height_lines_"-topic)
 */
void Mesh_visualizer::visualizeHeightLines(const std::vector<Line_collection>& lc){

 // don't publish if no one listens to this topic
 if (pub_height_lines_.getNumSubscribers() == 0)
  return;

 visualization_msgs::Marker marker;

 marker.header.frame_id = "/openni_rgb_optical_frame";
 marker.header.stamp = ros::Time::now();

 marker.type = visualization_msgs::Marker::LINE_LIST;
 marker.action = visualization_msgs::Marker::ADD;

 marker.id = 0;

 geometry_msgs::Point p;
 std_msgs::ColorRGBA col;
 col.a = 1.0;
 col.r = 1.0;
 col.g = 0.0;
 col.b = 0.0;

 marker.scale.x = 0.01;

 marker.color = col;
 marker.lifetime = ros::Duration();

 for (uint h=0; h<lc.size(); ++h){

  // todo change color for different height
  for (uint i=0; i<lc[h].size(); ++i){
   geometry_msgs::Point p,q;


   p.x = lc[h][i].first.x();
   p.y = lc[h][i].first.y();
   p.z = lc[h][i].first.z();

   q.x = lc[h][i].second.x();
   q.y = lc[h][i].second.y();
   q.z = lc[h][i].second.z();
   marker.points.push_back(p);
   marker.points.push_back(q);

   //   ROS_INFO("Adding point to visualizer: %f %f %f", p.x,p.y,p.z);
   //   ROS_INFO("Adding point to visualizer: %f %f %f", q.x,q.y,q.z);

  }
 }

 pub_height_lines_.publish(marker);

}


/**
 * @param mesh published Mesh. A colored TRIANGLE_LIST marker is generated and then send with the pub_-Publisher to RVIZ ("mesh"-topic)
 */
void Mesh_visualizer::visualizeMesh(const pcl::PolygonMesh& mesh){


 if (pub_.getNumSubscribers() == 0) {
   return;
  }

 Cloud cloud;
 pcl::fromROSMsg(mesh.cloud, cloud);

 visualization_msgs::Marker marker;

 marker.header.frame_id = "/openni_rgb_optical_frame";
 marker.header.stamp = ros::Time::now();

 marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

 marker.action = visualization_msgs::Marker::ADD;

 marker.id = 0;

 geometry_msgs::Point p;
 std_msgs::ColorRGBA col;
 col.a = 1.0;
 col.r = 1.0;
 col.g = 1.0;
 col.b = 0.0;

 marker.scale.x = 1;
 marker.scale.y = 1;
 marker.scale.z = 1;

 marker.color = col;

 marker.lifetime = ros::Duration();

 for (uint i=0; i<mesh.polygons.size(); ++i){
  pcl::Vertices vtc = mesh.polygons[i];

  assert(vtc.vertices.size() == 3);

  std_msgs::ColorRGBA cols[3];
  geometry_msgs::Point pts[3];

  for (uint j=0; j<3; ++j){

   pcl_Point cp = cloud.points[vtc.vertices[j]];

   p.x = cp.x;
   p.y = cp.y;
   p.z = cp.z;

   pts[j] = p;

   col.r = cp.r/255.0;
   col.g = cp.g/255.0;
   col.b = cp.b/255.0;
   col.a = 1;

   cols[j] = col;
  }

   marker.points.push_back(pts[0]);
   marker.points.push_back(pts[1]);
   marker.points.push_back(pts[2]);

   marker.colors.push_back(cols[0]);
   marker.colors.push_back(cols[1]);
   marker.colors.push_back(cols[2]);

 }

// ROS_INFO("sending mesh with %zu points", marker.points.size());

 pub_.publish(marker);

}



// returns false if both points are on same side of height (depending on z-axis)
/**
 *
 * @param a start point of line
 * @param b end point of line
 * @param height z-value of intersection plane (parallel to x-y plane)
 * @param interpolated Output: Point on the line between a and b with z=height
 * @return true iff the line has an intersection with the plane
 */
bool checkPair(const pcl_Point& a, const pcl_Point& b, float height, Eigen::Vector3f& interpolated){

 if ((a.z <= height && b.z <= height) || (a.z >= height && b.z >= height)) {
  return false;
 }
 // if (a.z == height || b.z == height) return false;


 pcl_Point sub = (a.z<b.z)?a:b;
 pcl_Point top = (a.z<b.z)?b:a;

 // ROS_INFO("sub: %f %f %f, top: %f %f %f, height: %f:", sub.z,sub.y,sub.z, top.z,top.y,top.z, height);
 assert(sub.z < height && top.z > height);

 float coeff = (height-sub.z)/(top.z-sub.z);


 assert(coeff>=0);
 interpolated.x() = sub.x+coeff*(top.x-sub.x);
 interpolated.y() = sub.y+coeff*(top.y-sub.y);
 interpolated.z() = sub.z+coeff*(top.z-sub.z);
 // ROS_INFO("P1: %f %f %f, P2: %f %f %f, height: %f:  IP:%f %f %f", a.x,a.y,a.z, b.x,b.y,b.z, height, interpolated.x(), interpolated.y(),interpolated.z());


 // if (abs(interpolated.x()) < 1e-10){
 //  assert(1==0);
 // }

 assert(interpolated.x() == interpolated.x());
 assert( abs(interpolated.x()) < 10);

 return true;

}


/**
 *
 * @param a first point of triangle
 * @param b second point of triangle
 * @param c third point of triangle
 * @param height z-value of intersection plane (parallel to x-y-plane)
 * @param pp Output: starting and end point of intersection line of triangle with given plane
 * @return true iff triangle has intersection points with the plane
 */
bool addLine(const pcl_Point& a, const pcl_Point& b, const pcl_Point& c, float height, PointPair& pp){


 Eigen::Vector3f interpolated;
 vector<Eigen::Vector3f> points;

 if (checkPair(a,b,height,interpolated)) points.push_back(interpolated);
 if (checkPair(a,c,height,interpolated)) points.push_back(interpolated);
 if (checkPair(b,c,height,interpolated)) points.push_back(interpolated);

 if (points.size() == 2){
  pp.first = points[0]; pp.second = points[1];
  return true;
 }

 return false;


}


/**
 *
 * @param mesh Input mesh for which the heightlines are generated
 * @param height_lines Output: Set of heightlines
 * @param min_z lowest intersection plane
 * @param max_z highes intersection line
 * @param height_step distance between adjacent intersection planes
 */
void Mesh_visualizer::findHeightLines(const pcl::PolygonMesh& mesh, std::vector<Line_collection>& height_lines, float min_z, float max_z, float height_step){

 Cloud cloud;
 pcl::fromROSMsg(mesh.cloud, cloud);

 // visualize x-direction
 //  float x_min = 1e6;
 //  float x_max = -1e6;
 //
 ////  find range of mesh in x-direction
 //  for (uint i=0; i<mesh.polygons.size(); ++i){
 //   pcl::Vertices vtc = mesh.polygons[i]; assert(vtc.vertices.size() == 3);
 //   for (uint j=0; j<3; ++j){
 //    float x = cloud.points[vtc.vertices[j]].z;
 //    x_min = std::min(x,x_min); x_max = max(x,x_max);
 //   }
 //  }

 //  ROS_INFO("CLoud: min: %f, max: %f", x_min, x_max);

 // float z_min = -0.3;
 // float z_max = 0;

 height_lines.clear();
 Line_collection current_lines;
 // loop over heights (else: start at given height)
 for (float height = max_z; height >= min_z; height-=height_step){

  //  ROS_INFO("Checking %f", height);


  current_lines.clear();
  PointPair pp;

  // loop over all triangles
  for (uint i=0; i<mesh.polygons.size(); ++i){
   pcl::Vertices vtc = mesh.polygons[i]; assert(vtc.vertices.size() == 3);


   bool line_found = addLine(cloud.points[vtc.vertices[0]],cloud.points[vtc.vertices[1]],cloud.points[vtc.vertices[2]], height,pp);


   if (line_found){
    //    ROS_INFO("ndx: %i %i %i",vtc.vertices[0], vtc.vertices[1], vtc.vertices[2]);
    //    ROS_INFO("pp: %f %f", pp.first.y(), pp.second.y());
    current_lines.push_back(pp);
   }

  }

  if (current_lines.size() > 0){
   ROS_INFO("Found %zu Pointpairs for height %f", current_lines.size(), height);
   height_lines.push_back(current_lines);
  }else{
   break;
  }

 }




 ROS_INFO("Found heightlines for %zu different heights", height_lines.size());






}

