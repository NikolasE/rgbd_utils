/*
 * meshing.h
 *
 *  Created on: Apr 20, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef MESHING_H_
#define MESHING_H_

//#include "type_definitions.h"
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/vtk_io.h>
#include "visualization_msgs/MarkerArray.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_greedy.h>
#include <pcl/features/normal_3d.h>
#include "rgbd_utils/calibration_utils.h"

//#include "type_definitions.h"
//#include "calibration.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

typedef std::pair<Eigen::Vector3f,Eigen::Vector3f> PointPair;
typedef std::vector<PointPair>  Line_collection;



struct Mesh_visualizer {

 ros::Publisher pub_, pub_lines_, pub_height_lines_;

 Mesh_visualizer(ros::NodeHandle& nh){
  pub_ = nh.advertise<visualization_msgs::Marker>( "mesh", 0 );
  pub_lines_ = nh.advertise<visualization_msgs::Marker>( "mesh_lines", 0 );
  pub_height_lines_ = nh.advertise<visualization_msgs::Marker>( "height_lines", 0 );
 }

 Mesh_visualizer(){}


 void visualizeMesh(const pcl::PolygonMesh& mesh);
 void visualizeMeshLines(const pcl::PolygonMesh& mesh);
 void visualizeHeightLines(const std::vector<Line_collection>& lc);

 void findHeightLines(const pcl::PolygonMesh& mesh,const Cloud& cloud, std::vector<Line_collection>& height_lines, float min_z, float max_z, float height_step);


 void createMesh(const Cloud& cloud, pcl::PolygonMesh& mesh,float max_length = -1, bool do_checks = false);

 void visualizeHeightLinesOnImage(const std::vector<Line_collection>& height_lines, cv::Mat& img, const cv::Mat& P);


 void getZRange(const Cloud& cloud, float& min_z, float& max_z, const cv::Mat* mask = NULL);


};



#endif /* MESHING_H_ */
