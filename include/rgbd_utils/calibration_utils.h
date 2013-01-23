/*
 * calibration.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "rgbd_utils/type_definitions.h"
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <limits>

#include <pcl/common/transform.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/voxel_grid.h"


void applyMaskInPlace(cv::Mat& mat, const cv::Mat mask);
cv::Mat applyMask(cv::Mat& mat, const cv::Mat mask);

void applyMaskOnCloudNaN(const cv::Mat& mask,const Cloud& cloud, Cloud& out);

void heightVisualization(cv::Mat& img, const cv::Mat height, float z_min, float z_max, float color_height,const cv::Mat* mask = NULL);
void waterVisualization(cv::Mat& img, const cv::Mat& water_depth, float min_water_depth, float max_water_depth,const cv::Mat* mask = NULL);
void waterVisualizationAlpha(cv::Mat& img, cv::Mat& alpha_channel, const cv::Mat& water_depth, float min_water_depth, float max_water_depth,const cv::Mat* mask = NULL);

Cloud transferColorToMesh(const cv::Mat& color, Cloud& mesh, const cv::Mat* mask = NULL);


void computeNormals(const Cloud& input, Cloud_n& with_normals);


void getTriangles(const cv::Mat& img, std::vector<cv::Vec3i>& triangles);

bool isSimilar(const cv::Mat& depth_1, const cv::Mat& depth_2, const cv::Mat* mask = NULL, const float dist_threshold = 0.05, const int outlier_threshold = 100);


Cloud applyMask(const Cloud& current, const cv::Mat& mask);




pcl_Point getTransformedPoint(pcl_Point p, const Eigen::Affine3f& trafo);

cv::Mat visualizeMask(const cv::Mat& img, const cv::Mat& mask);

Cloud computeMean(const std::vector<Cloud>& clouds);

Cloud removeMean(const Cloud& reference, const Cloud cloud, float max_dist, std::vector<cv::Point2i>* valids = NULL);


float dist(const cv::Point& a, const cv::Point& b);
cv::Point2f mean(const cv::Point& a, const cv::Point& b);
void add(pcl_Point& a,const pcl_Point& b);
void div(pcl_Point& a, float d);
pcl_Point sub(const pcl_Point& a,const pcl_Point& b);
float dist(const pcl_Point& a,const pcl_Point& b);
float dist_sq(const pcl_Point& a,const pcl_Point& b);
float norm(const pcl_Point& p);


pcl_Point setLength(const pcl_Point& p, float s);

void computeTransformationFromYZVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
  const Eigen::Vector3f& origin, Eigen::Affine3f& transformation);



void showPath(cv::Mat& img, const pcl::PolygonMesh& mesh,const cv::Mat& proj_matrix, const std::vector<cv::Point>* path,  const std::vector<cv::Vec3b>* colors = NULL);


void project3D(const cv::Point2f px, const cv::Mat P, float W,  cv::Point3f& out);



void projectCloudIntoImage(const Cloud& cloud, const cv::Mat& P, cv::Mat& img, float z_max, float z_min, float color_height = 0.1);


Cloud createCloud(const cv::Mat& depth, float fx, float fy, float cx, float cy);

Cloud colorizeCloud(const Cloud& cloud, float z_max, float z_min, float color_height, const cv::Mat* color = NULL, double max_water_height = 0.2, float min_water_height = 0.01);

//Cloud colorizeCloud(const Cloud& cloud, float min_height, const cv::Mat& color, float color_height);


bool loadMat(const std::string path, const std::string filename, cv::Mat& mat);
bool saveMat(const std::string path, const std::string filename, const cv::Mat& mat);

void applyMaskOnCloud(const cv::Mat& mask, const Cloud& in, Cloud& out);
void applyMaskOnCloud(const cv::Mat& mask, const pcl::PointCloud<pcl::Normal>& in, pcl::PointCloud<pcl::Normal>& out);


void sampleCloudWithNormals(const Cloud& points, const Cloud_n& normals, Cloud& points_out,Cloud_n& normals_out, uint step, cv::Mat* mask = NULL);


void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_);

void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_);

cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P);
cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P);
cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P);

void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed);
void scalePixels(const std::vector<cv::Point2f>& pxs,cv::Mat& T, std::vector<cv::Point2f>& transformed);

bool detectPlane(const Cloud& scene, pcl::ModelCoefficients::Ptr& coefficients,  Cloud& plane, Cloud& rest, float dist_threshold, const Eigen::Vector3f* normal = NULL, const float* eps = NULL);
void getClusters(const Cloud& scene, std::vector<pcl::PointIndices>& objects, float max_distance, int min_point_count = 0);
void computeDistanceToPlane(Cloud& scene, const pcl::ModelCoefficients::Ptr& coefficients, cv::Mat& dists, cv::Mat* mask = NULL);

// returns false if mean error after transformation is above max_dist
bool computeTransformationFromPointclouds(const Cloud& fixed, const Cloud& moved, Eigen::Affine3f& trafo, float max_dist = 0.05);


/**
* Compute center of pointcloud as column-vector
* if out!=NULL it contains the demeaned pointcloud
* @todo use this function in scaleCloud to prevent doubled code
*
*/
void centerPointCloud(const Cloud& in, cv::Mat& mean, Cloud* out = NULL);


bool saveAffineTrafo(const Eigen::Affine3f& M, const char* filename);
bool loadAffineTrafo(Eigen::Affine3f& M, const char* filename);

void printTrafo(const Eigen::Affine3f& M);

void update_min_filtered_cloud(Cloud& min_cloud, const Cloud& current);

#endif /* CALIBRATION_H_ */
