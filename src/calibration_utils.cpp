/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "rgbd_utils/calibration_utils.h"


#include "fstream"

using namespace std;

/**
*
* @param a
* @param b
* @return a-b
*/
/// result = a-b
pcl_Point sub(const pcl_Point& a,const pcl_Point& b){
  pcl_Point m;
  m.x = a.x-b.x; m.y = a.y-b.y; m.z = a.z-b.z;
  return m;
}

/**
*
* @param a will be a+b
* @param b
*/
/// a += b
void add(pcl_Point& a,const pcl_Point& b){
  a.x+=b.x; a.y+=b.y; a.z += b.z;
}

/**
*
* @param a will be a/d
* @param d
*/
/// a/=d
void div(pcl_Point& a, float d){
  assert(d != 0);
  a.x /= d; a.y /= d; a.z /= d;
}


/**
*
* @param p input point
* @return norm of p
*/
/// result = |p|
float norm(const pcl_Point& p){
  return sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
}

/**
*
* @return squared norm of (a-b)
*/
/// result = |a-b|^2
float dist_sq(const pcl_Point& a,const pcl_Point& b){
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}

/**
*
* @param a first input point
* @param b second input point
* @return norm of a-b
*/
/// result |a-b|
float dist(const pcl_Point& a,const pcl_Point& b){
  return norm(sub(a,b));
}

/**
*
* Sets length of input vector to given length
*
* @param p input vector
* @param s new length
* @return p/|p|*s
*/
pcl_Point setLength(const pcl_Point& p, float s){
  float l = norm(p);
  assert(l>0);
  pcl_Point out;

  out.x = p.x/l*s;
  out.y = p.y/l*s;
  out.z = p.z/l*s;

  // if (abs(norm(out)-s)> 0.01){
  //  ROS_INFO("old; %f %f %f", p.x,p.y,p.z);
  //  ROS_INFO("new; %f %f %f", out.x,out.y,out.z);
  //  ROS_INFO("dist: %f", abs(norm(out)-s));
  // }

  return out;
}

/**
* @param current input point cloud
* @param mask uchar image with same dimension as input cloud. Only white ( val = 255) pixels are considered foreground.
* @return Input cloud where non foreground points are replaced by NaN values. (Cloud is still organized)
*/
Cloud applyMask(const Cloud& current, const cv::Mat& mask){

  //cout << "foo" << endl;
  if (uint(mask.cols) != current.width){
    ROS_INFO("mask: %i, current: %i", mask.cols, current.width);
    return current;
  }
  // cout << "bar" << endl;
  Cloud result = current;
  result.reserve(current.size());

  pcl_Point nap; nap.x = nap.y = nap.z = 0;// std::numeric_limits<float>::quiet_NaN();

  // cout << "div" << endl;

  assert(nap.x != nap.x);
  // cout << "dd" << endl;

  int invalid = 0;

  for (uint x=0; x<current.width; ++x)
    for (uint y=0; y<current.height; ++y){
      //   cout << x << " " << y << endl;
      if (mask.at<uchar>(y,x) != 255){
        invalid++;
        //    cout << "invalid" << endl;
        result.at(x,y) = nap;
      }
    }

  ROS_INFO("removed %i points", invalid);

  return result;

}


void getClusters(const Cloud& scene, std::vector<pcl::PointIndices>& segments, float max_distance, int min_point_count){

  segments.clear();
  assert(max_distance > 0);
  assert(min_point_count >= 0);


  // find largest cluster in plane
  pcl::EuclideanClusterExtraction<pcl_Point> extractor;

// TODO: cloud is ordered!
  pcl::KdTree<pcl_Point>::Ptr clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl_Point> > ();  // CHANGED



  // pcl::search::Search<pcl_Point>::Ptr clusters_tree_ = boost::make_shared<pcl::search::KdTree<pcl_Point> > ();
  extractor.setMinClusterSize(min_point_count); // min number of points in a cluster
  extractor.setClusterTolerance(max_distance);  // min dist between clusters
  extractor.setSearchMethod (clusters_tree_);
  extractor.setInputCloud(scene.makeShared());

  //timing_start("extract");
  extractor.extract(segments);
  //timing_end("extract");

  // ROS_INFO("extracted %zu segments", segments.size());

//  objects.resize(segments.size());

// // timing_start("cpy");
//  for (uint i=0; i<segments.size(); ++i){
//    int n = segments[i].indices.size();
//    Cloud c; c.resize(n);
//    for (int p=0; p>n; ++p){  c.points[p] = scene.points[segments[i].indices[p]]; }
//    objects[i] = c;
//  }
  //timing_end("cpy");

}








void computeDistanceToPlane(Cloud& scene, const pcl::ModelCoefficients::Ptr& coefficients, cv::Mat& dists, cv::Mat* mask){

  assert(dists.rows == scene.height && dists.cols == scene.width);
  assert(dists.type() == CV_32FC1);
  dists.setTo(0);
  if (mask) assert(mask->size() == dists.size());


  float c_x,c_y,c_z,c_d;
  c_x = coefficients->values[0];
  c_y = coefficients->values[1];
  c_z = coefficients->values[2];
  c_d = coefficients->values[3];

  //  ROS_INFO("Plane: %f %f %f = %f",x,y,z,d);


  for (uint x=0; x<scene.width; ++x)
    for (uint y=0; y<scene.height; ++y)
      {

        pcl_Point P = scene.at(x,y); if(P.x != P.x) continue;

        if (mask != NULL && mask->at<uchar>(y,x) == 0)
          continue;

        float d_p = c_x*P.x+c_y*P.y+c_z*P.z+c_d;

        // if (d_p < 0) continue;

        dists.at<float>(y,x) = d_p;
        // cout << abs(d_p) << endl;
      }

  //    { // only visualisation:
  //        double maxVal=0.04;
  //        double minVal=0.04;

  //        cv::minMaxLoc(dists, &minVal, &maxVal, 0, 0);
  //        ROS_INFO("min %f, max: %f", minVal, maxVal);
  //        cv::Mat cp  = (dists+minVal)/(maxVal-minVal);

  //        //cv::threshold(cp, cp, 0.25, CV_THRESH_TOZERO, 0 );
  //        cv::namedWindow("depth above table", 1);
  //        cv::imshow("depth above table", cp);
  //        cv::waitKey(10);
  //    }

}




bool detectPlane(const Cloud& scene, pcl::ModelCoefficients::Ptr& coefficients,  Cloud& plane, Cloud& rest, float dist_threshold, const Eigen::Vector3f* normal, const float* eps){


  plane.clear();
  rest.clear();

  assert(dist_threshold>0);

  pcl::SACSegmentation<pcl_Point> seg;


  seg.setOptimizeCoefficients (true);

  // look only for tables (no walls")
  //  seg.setAxis(Eigen::Vector3f(0,0,1)); // upwards
  //  seg.setEpsAngle(20/180.0*M_PI); // accept planes with not more than 20deg deviation from given axis



  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dist_threshold); // max dist to be considered an inlier (in m)

  seg.setInputCloud(scene.makeShared());
  seg.segment(*inliers, *coefficients);

  if (coefficients->values[3] < 0){
    for (uint i=0; i<4; ++i) coefficients->values[i] *= -1;
  }

  assert(coefficients->values[3]>0);

  if (inliers->indices.size () == 0) {
    ROS_WARN("Could not estimate a planar model for the given scene");
    return false;
  }


  // Cloud::Ptr inlier_cloud ( new Cloud );

  // create cloud which contains the inliers from the RANSAC
  pcl::ExtractIndices<pcl_Point > extract;
  extract.setInputCloud(scene.makeShared());
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter (plane);

  extract.setNegative (true);
  extract.filter (rest);

  return true;


}


/// triangles given as index in row-major (id = x+y*img.cols)
void getTriangles(const cv::Mat& img, std::vector<cv::Vec3i>& triangles){

  assert(img.cols > 0 && img.rows > 0 && img.type() == CV_8UC1);
  triangles.clear();

  int w = img.cols;

  for (int x=0; x<img.cols-1; ++x)
    for (int y=0; y<img.rows-1; ++y){

      int valid_corners[4];
      int valid_cnt = 0;

      if(img.at<uchar>(y,x)     > 0){valid_corners[valid_cnt++] = x+y*w;}
      if(img.at<uchar>(y,x+1)   > 0){valid_corners[valid_cnt++] = (x+1)+y*w;}
      if(img.at<uchar>(y+1,x+1) > 0){valid_corners[valid_cnt++] = (x+1)+(y+1)*w;}
      if(img.at<uchar>(y+1,x)   > 0){valid_corners[valid_cnt++] = x+(y+1)*w;}



      if (valid_cnt < 3)
        continue;

      // triangle is always counter clock wise
      if (valid_cnt == 3){
        cv::Vec3i tri;
        for (uint i=0; i<3; ++i)
          tri[i] = valid_corners[i];
        triangles.push_back(tri);
        continue;
      }


      // valid_cnt == 4, all corners are marked,
      cv::Vec3i tri;
      tri[0] = valid_corners[0]; // corners a,b,c (top left, top right, lower right
      tri[1] = valid_corners[1];
      tri[2] = valid_corners[2];
      triangles.push_back(tri);

      tri[0] = valid_corners[2]; // corners c,d,a
      tri[1] = valid_corners[3];
      tri[2] = valid_corners[0];
      triangles.push_back(tri);
    }

}




/**
 *
 * untested
 *
 * @param img
 * @param mesh
 * @param proj_matrix
 * @param path
 * @param colors
 */
void showPath(cv::Mat& img, const pcl::PolygonMesh& mesh, const cv::Mat& proj_matrix, const std::vector<cv::Point>* path,  const std::vector<cv::Vec3b>* colors){

  Cloud cloud;
  pcl::fromROSMsg(mesh.cloud, cloud);

  assert(cloud.width > 1 && cloud.height > 1);

  // glColor3f(0,1,0);
  // glLineWidth(5.0);

  int line_width = 3;

  bool with_colors = (colors != NULL);

  // ROS_INFO("path: %zu, colors: %zu", path.size(), pathColors.size());

  // draw path in green if no colors are given
  // cv::Vec3b col_green = cv::Vec3b(0,255,0);

  cv::Scalar col(0,255,0);

  cv::Point2f last_pos,current;

  for (uint i=0; i<path->size(); ++i){

    pcl_Point P3 = cloud.at(path->at(i).x, path->at(i).y);

    // project 3D point into image
    current = applyPerspectiveTrafo(P3,proj_matrix);

    if (i==0){
      last_pos = current;
      continue;
    }

    if (with_colors){
      col = cv::Scalar(colors->at(i).val[0],colors->at(i).val[1],colors->at(i).val[2]);
    }

    cv::line(img, last_pos, current, col, line_width);
  }


}




/**
* @param depth_1   first depth image
* @param depth_2   second depth image
* @param mask      mask (if zero, pixel is ignored)
* @param dist_threshold  maximal absolute distance between pixel values so that the pair is considered similar
* @param outlier_threshold  maximal number of unsimilar pixel pairs
* @return  true if both images in the masked area are similar
*/
/// If less than  <B>outlier_threshold</B> pixels have an absolute difference larger than  <B>dist_threshold</B>, both images are considered similar.
bool isSimilar(const cv::Mat& depth_1, const cv::Mat& depth_2, const cv::Mat* mask, const float dist_threshold, const int outlier_threshold){

  cv::Mat thres;
  cv::threshold(cv::abs(depth_1-depth_2), thres, dist_threshold,1,CV_THRESH_BINARY);


  int outlier;

  // apply mask if given
  if (mask){
    cv::Mat masked;
    thres.copyTo(masked,*mask);
    outlier = cv::countNonZero(masked);
  }
  else{
    outlier = cv::countNonZero(thres);
  }

  ROS_INFO("XX Found %i outlier:",outlier);
  return outlier < outlier_threshold;
}



/**
*
* @param reference Reference depth for each pixel (background model)
* @param cloud current cloud from which the background should be removed
* @param max_dist if a pixel is more than max_dist closer to the cam than the corresponding mask pixel, it's considered foreground
* @param valids optionel: pixelpositions of all foreground pixels
* @return All points that are more than max_dist closer to the cam than the corresponding point in the reference cloud
* @todo rename function and max_dist
*/
Cloud removeMean(const Cloud& reference, const Cloud cloud, float max_dist, std::vector<cv::Point2i>* valids){

  Cloud result;
  assert(cloud.size() == reference.size());

  // for (uint i=0; i<cloud.size(); ++i){

  for (uint x=0; x<cloud.width; ++x)
    for (uint y=0; y<cloud.height; ++y){

      pcl_Point c = cloud.at(x,y);
      pcl_Point r = reference.at(x,y);

      if (c.x != c.x) continue;

      if (((r.x != r.x) || norm(c) < norm(r) - max_dist)){
        result.push_back(c);

        if (valids) valids->push_back(cv::Point2i(x,y));
      }

    }

  return result;

}




/**
* @param clouds array of input clouds (all of same size)
* @return Cloud where each point has the mean depth of the corresponding pixels in the input clouds.
*/
Cloud computeMean(const std::vector<Cloud>& clouds){

  assert(clouds.size() > 1);

  Cloud out = clouds[0];

  // ROS_INFO("W.H: %i %i", out.width, out.height);

  for (uint i=0; i<clouds.size(); ++i){
    //  assert(clouds[i].is_dense);
    assert(clouds[i].width == out.width && clouds[i].height == out.height);
    //  ROS_INFO("CLoud %i: w: %i, h: %i, s: %i", i, clouds[i].width, clouds[i].height, clouds[i].size());
  }


  uint total_valid = 0;

  // compute mean of distances in every pixel:
  // could also be done on the distance images
  for (uint x=0; x<out.width; ++x)
    for (uint y=0; y<out.height; ++y){
      pcl_Point mean; mean.x = mean.y = mean.z = 0;
      int cnt = 0;// number of clouds that have a measurement at this position


      for (uint i=0; i<clouds.size(); ++i){
        pcl_Point p = clouds[i].at(x,y);
        if (p.x != p.x) continue; // NaN value
        add(mean, p);
        cnt++;
      }


      if (cnt == 0){
        out.at(x,y) = clouds[0](x,y); // copy a nan-point at this position
      }else{

        div(mean,cnt);
        out.at(x,y) = mean;
        total_valid++;
      }
    }

  // ROS_INFO("Computed mean of %zu cloud, resulting cloud has %i valid points",clouds.size(),total_valid);

  return out;

}




void project3D(const cv::Point2f px, const cv::Mat P, float W,  cv::Point3f& out){

  /*
  * [px.x,px.y,1] = P [out.x,out.y,out.z,W]
  */

  float a = px.x;
  float b = px.y;


  double p[3][4];

  for (uint i=0; i<3; i++)
    for (uint j=0; j<4; j++)
      p[i][j] = P.at<double>(i,j);

  // in Maple:
  // solve({b = p[2, 1]*X+p[2, 2]*Y+p[2, 3]*Z+p[2, 4]*W, a = p[1, 1]*X+p[1, 2]*Y+p[1, 3]*Z+p[1, 4]*W, 1 = p[3, 1]*X+p[3, 2]*Y+p[3, 3]*Z+W}, [X, Y, Z])
  // language conversion to C

  double sub = p[0][1] * p[1][0] * p[2][2] - p[2][0] * p[0][1] * p[1][2] - p[1][0] * p[2][1] * p[0][2] - p[1][1] * p[0][0] * p[2][2] + p[2][0] * p[1][1] * p[0][2] + p[0][0] * p[2][1] * p[1][2];

  out.x =  ( p[1][1] * p[0][2] - p[2][1] * p[1][2] * p[0][3] * W + p[2][1] * p[1][2] * a - p[2][1] * b * p[0][2] + p[2][2] * p[0][1] * b - p[2][2] * p[1][1] * a + p[2][1] * p[1][3] * W * p[0][2] + p[2][2] * p[1][1] * p[0][3] * W - p[2][2] * p[0][1] * p[1][3] * W + W * p[0][1] * p[1][2] - W * p[1][1] * p[0][2] - p[0][1] * p[1][2])   / sub;
  out.y = -(-p[1][0] * p[2][2] * a - p[1][0] * W * p[0][2] + p[1][0] * p[2][2] * p[0][3] * W + p[1][0] * p[0][2] + p[2][2] * p[0][0] * b - p[2][0] * p[1][2] * p[0][3] * W + p[2][0] * p[1][2] * a - p[2][2] * p[0][0] * p[1][3] * W + p[2][0] * p[1][3] * W * p[0][2] + W * p[0][0] * p[1][2] - p[2][0] * b * p[0][2] - p[0][0] * p[1][2]) / sub;
  out.z = -( p[2][0] * p[1][1] * p[0][3] * W - p[2][0] * p[0][1] * p[1][3] * W + p[2][0] * p[0][1] * b - p[0][1] * p[1][0] - p[0][0] * p[2][1] * b - p[2][0] * p[1][1] * a - p[1][0] * p[2][1] * p[0][3] * W + p[0][1] * p[1][0] * W + p[1][0] * p[2][1] * a + p[1][1] * p[0][0] - p[1][1] * p[0][0] * W + p[0][0] * p[2][1] * p[1][3] * W)  / sub;


  // test: reproject into image
  cv::Point2f px_test = applyPerspectiveTrafo(out,P);
  if (abs(px.x-px_test.x) > 0.1 || abs(px.y-px_test.y) > 0.1 ){
    ROS_INFO("Input: %f %f, 3d: %f %f %f, projected: %f %f", px.x,px.y, out.x, out.y, out.z, px_test.x, px_test.y);
    assert(1==0);
  }

}



/**
*
* @param path foldername (with trailing /)
* @param name name of matrix
* @param mat output matrix
* @return true iff matrix was found at path/name.yml
*/
bool loadMat(const string path, const string name, cv::Mat& mat){
  char fn[100]; sprintf(fn,"%s%s.yml", path.c_str(), name.c_str());
  ROS_INFO("Reading %s from %s", name.c_str(),fn);

  cv::FileStorage fs(fn, cv::FileStorage::READ);
  if (!fs.isOpened()){
    ROS_WARN("Could not read %s", fn);
    return false;
  }

  fs[name] >> mat; fs.release();
  fs.release();
  return true;
}

/**
*
* @param path folder (name with trailing /)
* @param name title for matrix
* @param mat matrix that will be written to file
* @return true iff matrix could be stored at folder/name.yml
*/
bool saveMat(const string path, const string name, const cv::Mat& mat){
  char fn[100]; sprintf(fn,"%s%s.yml", path.c_str(), name.c_str());
  ROS_INFO("Saving %s to %s", name.c_str(),fn);

  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  if (!fs.isOpened()){
    ROS_WARN("Could not write to %s", fn);
    return false;
  }

  fs << name << mat;
  fs.release();
  return true;
}


/**
*
* @todo rename
*
* @param points
* @param normals
* @param points_out
* @param normals_out
* @param step
* @param mask
*/
void sampleCloudWithNormals(const Cloud& points, const Cloud_n& normals, Cloud& points_out,Cloud_n& normals_out, uint step, cv::Mat* mask){

  assert(points.size() == normals.size());
  if (mask) assert(mask->cols == int(points.width));
  points_out.clear();
  normals_out.clear();
  points_out.reserve(points.size());
  normals_out.reserve(normals.size());


  for (uint x=0; x<points.width; x += step)
    for (uint y=0; y<points.height; y += step){

      if (mask && mask->at<uchar>(y,x) == 0) continue;

      pcl_Point p = points.at(x,y);
      if (!(p.x == p.x)) continue;

      points_out.push_back(p);
      normals_out.push_back(normals.at(x,y));
    }


  assert(points_out.size() == normals_out.size());

  ROS_INFO("Downsampling of image: from %zu points and %zu normales to %zu points", points.size(), normals.size(), points_out.size() );


}

/**
* @param mask uchar matrix
* @param in input cloud (same size as mask)
* @param out all normals in input cloud where corresponding mask pixel has positive value (not organized)
*/
void applyMaskOnCloud(const cv::Mat& mask, const pcl::PointCloud<pcl::Normal>& in, pcl::PointCloud<pcl::Normal>& out){
  assert(mask.cols == int(in.width));
  out.clear();

  for (int x=0; x<mask.cols; ++x)
    for (int y=0; y<mask.rows; ++y){
      if (mask.at<uchar>(y,x) > 0){
        out.push_back(in.at(x,y));
      }
    }
}


void applyMaskOnCloudNaN(const cv::Mat& mask, const Cloud& cloud, Cloud& out){
  assert(mask.cols == int(cloud.width));
  assert(mask.type() == CV_32FC1);

  out.width = cloud.width;
  out.height = cloud.height;
  out.resize(cloud.size());

  pcl_Point nap; nap.x = nap.y = nap.z = std::numeric_limits<float>::quiet_NaN();

  for (uint x=0; x<cloud.width; ++x)
    for (uint y=0; y<cloud.height; ++y){
      if (mask.at<float>(y,x) == 0){
        out.points[y*cloud.width+x] = nap;
      }else{
        out.points[y*cloud.width+x] = cloud.at(x,y);
      }
    }
}


/**
* @param mask uchar matrix
* @param in input cloud (same size as mask)
* @param out all points in input cloud where corresponding mask pixel has positive value (not organized)
*
* @todo template on point Type of cloud
*/
void applyMaskOnCloud(const cv::Mat& mask, const Cloud& in, Cloud& out){

  assert(mask.cols == int(in.width));
  out.clear();

  for (int x=0; x<mask.cols; ++x)
    for (int y=0; y<mask.rows; ++y){
      if (mask.at<uchar>(y,x) > 0){
        pcl_Point p = in.at(x,y);
        if (p.x == p.x)
          out.push_back(p);
      }
    }

}


/**
 *
 * @param mat
 * @param mask  8UC1-mask
 */
/// if res(x,y) = (mask(x,y) == 0) ? mat(x,y) = 0 : 0
cv::Mat applyMask(cv::Mat& mat, const cv::Mat mask){

  // assert(mat.type() == CV_32FC1 || mat.type() == CV_64FC1);
  assert(mat.size() == mask.size());

  cv::Mat result(mat.size(), mat.type());

  result.setTo(0);

  mat.copyTo(result, mask);

  return result;
}

/**
 *
 * @param mat   CV_32FC1 or CV_64FC1 image
 * @param mask  8UC1-Image
 */
/// if (mask(x,y) == 0) mat(x,y) = 0
void applyMaskInPlace(cv::Mat& mat, const cv::Mat mask){
  assert(mat.type() == CV_32FC1 || mat.type() == CV_64FC1);
  assert(mask.type() == CV_8UC1);
  assert(mat.size() == mask.size());

  bool float_img = mat.type() == CV_32FC1;

  for (int x=0; x<mat.cols; ++x)
    for (int y=0; y<mat.rows; ++y){

      if (mask.at<uchar>(y,x) == 0){
        if (float_img)
          mat.at<float>(y,x) = 0;
        else
          mat.at<double>(y,x) = 0;
      }
    }

}





/**
* Takes around 1ms for an 45x50 grid
*
* @param input         organized pointcloud
* @param with_normals  corresponding normals
*/
/// computing normals for organized pointcloud with the   <a href="http://www.pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php">pcl::IntegralImageNormalEstimation</a>.
void computeNormals(const Cloud& input, Cloud_n& with_normals){


  int nan_point = 0;

  for (uint i=0; i<input.size(); ++i){
    if (input.at(i).x != input.at(i).x)
      nan_point++;
  }


  pcl::IntegralImageNormalEstimation<pcl_Point, pcl_Normal> ne;

  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX); //AVERAGE_3D_GRADIENT
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(2.0f);
  ne.setInputCloud(input.makeShared());
  ne.compute(with_normals);
  // ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  // normalize direction of normals

  int nan_cnt = 0;

  for (uint i=0; i<with_normals.size(); ++i){
    pcl_Normal *n = &with_normals.at(i);

    if (n->normal_x != n->normal_x)
      nan_cnt++;

    if  (n->normal_z < 0){
      n->normal_x *=-1;
      n->normal_y *=-1;
      n->normal_z *=-1;
    }


  }

  ROS_INFO("Normals: %i from %zu points without normal (%i pts without pos)", nan_cnt, with_normals.size(), nan_point);


  // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  // viewer.setBackgroundColor (0.0, 0.0, 0.5);
  //
  // // viewer.addPointCloud<pcl_Point>(input.makeShared());
  // viewer.addPointCloudNormals<pcl_Point,pcl_Normal>(input.makeShared(), with_normals.makeShared());
  //
  // while (!viewer.wasStopped ())
  //  {
  //  viewer.spinOnce ();
  //  }


}



/**
* @param fixed goal pointcloud
* @param moved moved pointcloud (same size as fixed cloud)
* @param trafo resulting transformation (fixed = trafo*moved)
* @param max_dist treshold for evaluation
* @return computes the affine transformation that minimizes the quadratic distance between cooresponding points. Return value is true iff
* mean error is smaller than max_dist
*/
bool computeTransformationFromPointclouds(const Cloud& fixed, const Cloud& moved, Eigen::Affine3f& trafo, float max_dist){


  assert(fixed.size() == moved.size());

  cv::Mat M(3,3,CV_32FC1);
  M.setTo(0);

  cv::Mat r(3,1,CV_32FC1);
  cv::Mat c(3,1,CV_32FC1);

  cv::Mat mean_fixed, mean_moved;
  Cloud fixed_centered, moved_centered;

  // compute center of point clouds and centered pointclouds
  centerPointCloud(fixed, mean_fixed, &fixed_centered);
  centerPointCloud(moved, mean_moved, &moved_centered);


  cout << "center fixed:" << endl << mean_fixed << endl << "center moved: " << endl << mean_moved << endl;


  for (uint i=0; i<fixed_centered.size(); ++i){

    //  pcl_Point a = fixed[i];
    //  pcl_Point b = moved[i];
    //
    //  ROS_INFO("fixed: %f %f %f", a.x,a.y,a.z);
    //  ROS_INFO("moved: %f %f %f", b.x,b.y,b.z);


    pcl_Point f = fixed_centered[i];
    pcl_Point m = moved_centered[i];

    // fixed cloud has column vector
    c.at<float>(0) = f.x; c.at<float>(1) = f.y; c.at<float>(2) = f.z;
    r.at<float>(0) = m.x; r.at<float>(1) = m.y; r.at<float>(2) = m.z;

    //  cout << "c "  << c << endl << "r "  << r << endl;
    //  cout << c*r.t() << endl;

    M += c*r.t();
  }




  // cout << "M " << endl << M << endl;


  cv::Mat W,U,Vt;

  cv::SVDecomp(M, W,U,Vt);

  cout << "W " << endl << W << endl;
  cout << "U " << endl << U << endl;
  cout << "Vt " << endl << Vt << endl;


  cv::Mat R = U*Vt;

  cout << "R " << endl << R << endl;


  cv::Mat t = mean_fixed - R*mean_moved;

  cout << "t " << endl << t << endl;


  // return false;


  for (uint i=0; i<3; ++i){
    for (uint j=0; j<3; ++j){
      trafo(i,j) = R.at<float>(i,j);
    }
    trafo(3,i) = 0; // lowest row
    trafo(i,3) = t.at<float>(i); // right colum
  }
  trafo(3,3) = 1;

  ROS_INFO("kinect trafo");
  printTrafo(trafo);


  // compute error before and after trafo:
  double error_before = 0;
  double error_after = 0;




  Cloud moved_final;
  pcl::getTransformedPointCloud(moved, trafo, moved_final);


  for (uint i=0; i<fixed.size(); ++i){
    pcl_Point f = fixed[i];
    pcl_Point m_orig = moved[i];
    pcl_Point m_trafoed = moved_final[i];

    error_before += dist(f, m_orig);
    error_before += dist(f, m_trafoed);
  }

  error_before /= fixed.size();
  error_after /= fixed.size();



  ROS_INFO("computeTransformationFromPointcloud: error before: %f cm, error after: %f cm", error_before*100, error_after*100);

  return (error_after < max_dist);

}



/**
* @param min_cloud reference cloud (pixel is replaced by corresponding entry in current if this point has a smaller z-value)
* @param current current measurement (same width and height as reference)
*/
void update_min_filtered_cloud(Cloud& min_cloud, const Cloud& current){
  for (uint x = 0; x<min_cloud.width; x++)
    for (uint y = 0; y<min_cloud.height; y++){
      pcl_Point old_point = min_cloud.at(x,y);
      pcl_Point new_point = current.at(x,y);

      // if point was nan, update always
      if (!(old_point.x == old_point.x)){
        min_cloud.at(x,y) = new_point;
        continue;
      }

      // update point if new point is valid
      if (!(new_point.x == new_point.x)) continue;

      // z-axis points into sand...
      if (new_point.z > old_point.z)
        min_cloud.at(x,y) = new_point;
    }

}


/**
* @param pxs input pixel positions
* @param T Transformation that sets the center of the pixels to (0,0) and the mean distance to sqrt(2)
* @param transformed transformed input pixels
*/
void scalePixels(const std::vector<cv::Point2f>& pxs, cv::Mat& T, std::vector<cv::Point2f>& transformed){

  uint c_cnt = pxs.size();

  Eigen::Vector2f mu(0,0);
  for (uint i=0; i<c_cnt; ++i){
    cv::Point2f px = pxs.at(i);
    mu[0] += px.x; mu[1] += px.y;
  }
  mu /= c_cnt;

  // get mean distance to center:
  double d = 0;
  for (uint i=0; i<c_cnt; ++i){
    cv::Point2f px = pxs.at(i);
    d += sqrt(pow(px.x-mu[0],2)+pow(px.y-mu[1],2));
  }
  d /= c_cnt;

  // ROS_INFO("2d: mean: %f %f, dist: %f", mu[0], mu[1], d);

  // mean distance should be sqrt(2)
  double s = sqrt(2)/d;

  T = cv::Mat::eye(3,3,CV_64FC1); T*=s;  T.at<double>(2,2) = 1;
  for (int i=0; i<2; ++i)
    T.at<double>(i,2) = -mu[i]*s;

  //	cout << "T" << endl << T << endl;

  // apply matrix on all points
  Cloud d3_scaled;

  cv::Mat P = cv::Mat(3,1,CV_64FC1);

  transformed.reserve(c_cnt);
  for (uint i=0; i<c_cnt; ++i){
    cv::Point2f p = pxs.at(i);

    P.at<double>(0,0) = p.x;
    P.at<double>(1,0) = p.y;
    P.at<double>(2,0) = 1;

    P = T*P;

    p.x = P.at<double>(0,0);
    p.y = P.at<double>(1,0);

    // ROS_INFO("centered: %f %f", p.x,p.y);

    transformed.push_back(p);
  }


}

/**
*
* @param in input point cloud
* @param mean center of gravity of the points
* @param out de-meaned pointcloud
*/
void centerPointCloud(const Cloud& in, cv::Mat& mean, Cloud* out){

  if (in.size() == 0){
    ROS_WARN("centerPointCloud called with empty inputcloud");
    return;
  }

  mean = cv::Mat(3,1,CV_32FC1);
  mean.setTo(0);

  for (uint i=0; i<in.size(); ++i){
    pcl_Point p = in[i];
    mean.at<float>(0) += p.x;
    mean.at<float>(1) += p.y;
    mean.at<float>(2) += p.z;
  }

  mean /= in.size();

  if (!out) return;

  // store so that the values aren"t read from mean for every point
  float x = mean.at<float>(0);
  float y = mean.at<float>(1);
  float z = mean.at<float>(2);

  out->reserve(in.size());

  for (uint i=0; i<in.size(); ++i){
    pcl_Point p = in[i];
    p.x -= x;   p.y -= y;  p.z -= z;
    out->push_back(p);
  }

}


/**
* @param pts input point cloud
* @param U transformation that sets the center of the point cloud to (0,0,0) and the mean norm to sqrt(3)
* @param transformed transformed pointcloud (center at Origo, mean norm of sqrt(3)
*/
void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed){

  uint c_cnt = pts.points.size();

  // normalize since metric coordinates are in [0,1]^3
  // and pixels in [0,640]x[0,480]
  Eigen::Vector3f mu(0,0,0);
  for (uint i=0; i<c_cnt; ++i){
    pcl_Point	p = pts.points.at(i);
    mu[0] += p.x; mu[1] += p.y; mu[2] += p.z;
  }
  mu /= c_cnt;
  // get mean distance to center:
  double d = 0;
  for (uint i=0; i<c_cnt; ++i){
    pcl_Point	p = pts.points.at(i);
    d += sqrt(pow(p.x-mu[0],2)+pow(p.y-mu[1],2)+pow(p.z-mu[2],2));
  }
  d /= c_cnt;
  // ROS_INFO("3d: mean: %f %f %f, dist: %f", mu[0], mu[1], mu[2], d);

  // mean distance should be sqrt(3)
  double s = sqrt(3)/d;


  U = cv::Mat::eye(4,4,CV_64FC1); U*=s;  U.at<double>(3,3) = 1;
  for (int i=0; i<3; ++i)
    U.at<double>(i,3) = -mu[i]*s;

  //	 cout << "U" << endl << U << endl;

  // apply matrix on all points
  Cloud d3_scaled;

  cv::Mat P = cv::Mat(4,1,CV_64FC1);

  transformed.reserve(c_cnt);
  for (uint i=0; i<c_cnt; ++i){
    pcl_Point		p = pts.points.at(i);

    P.at<double>(0,0) = p.x;
    P.at<double>(1,0) = p.y;
    P.at<double>(2,0) = p.z;
    P.at<double>(3,0) = 1;

    P = U*P;

    p.x = P.at<double>(0,0);
    p.y = P.at<double>(1,0);
    p.z = P.at<double>(2,0);

    transformed.push_back(p);
  }


}

/**
* @param p Point in R^3
* @param P projective Transformation (3x4 matrix in homogeneous coordinates)
* @return Pixel that corresponds to the given world point.
*/
cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P){
  cv::Point3f p3; p3.x = p.x(); p3.y = p.y(); p3.z = p.z();
  return applyPerspectiveTrafo(p3,P);
}

/**
* @param p Point in R^3
* @param P projective Transformation (3x4 matrix in homogeneous coordinates)
* @param p_ (output) Pixel that corresponds to the given world point.
*/
void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_){
  cv::Mat P4 = cv::Mat(4,1,CV_64FC1);
  cv::Mat P3 = cv::Mat(3,1,CV_64FC1);

  P4.at<double>(0,0) = p.x;
  P4.at<double>(1,0) = p.y;
  P4.at<double>(2,0) = p.z;
  P4.at<double>(3,0) = 1;

  P3 = P*P4;

  double z = P3.at<double>(2);

  p_.x = P3.at<double>(0)/z;
  p_.y = P3.at<double>(1)/z;

}

/**
* @param p Point in R^3
* @param P projective Transformation (3x4 matrix in homogeneous coordinates)
* @return Pixel that corresponds to the given world point.
*/
cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P){
  cv::Point2f px;
  applyPerspectiveTrafo(p,P,px);
  return px;
}

/**
* @param p Point in R^3
* @param P projective Transformation (3x4 matrix in homogeneous coordinates)
* @return Pixel that corresponds to the given world point.
*/
cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P){
  cv::Point3f p3; p3.x = p.x; p3.y = p.y; p3.z = p.z;
  return applyPerspectiveTrafo(p3,P);
}

/**
*
* @param p Point in R^2
* @param H Homography (homogeneous coordinates, 3x3 CV_64FC1 matrix)
* @param p_  result of application of H to p
*/
void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_){

  assert(H.cols == 3 && H.rows == 3);

  cv::Mat pc (3,1,CV_64FC1);
  pc.at<double>(0) = p.x;
  pc.at<double>(1) = p.y;
  pc.at<double>(2) = 1;

  pc = H*pc;

  double z = pc.at<double>(2);

  p_.x = pc.at<double>(0)/z;
  p_.y = pc.at<double>(1)/z;
}

/**
* @param M Affine Transformation that will be printed to stdout
*/
void printTrafo(const Eigen::Affine3f& M){
  for (uint i=0;i<4; ++i){
    for (uint j=0;j<4; ++j)
      cout << M(i,j) << " ";
    cout << endl;
  }
}

/**
* @param M Matrix to be written to file
* @param filename filename
* @return true iff file could opened
*/
bool saveAffineTrafo(const Eigen::Affine3f& M, const char* filename){
  ofstream off(filename);
  if (!off.is_open()){ ROS_WARN("Could not write to %s", filename); return false;}
  for (uint i=0;i<4; ++i){
    for (uint j=0;j<4; ++j)
      off << M(i,j) << " ";
    off << endl;
  }

  return true;
}

/**
* @param M (output) Affine Matrix that will be loaded fromfile
* @param filename filename
* @return true iff file was found
*/
bool loadAffineTrafo(Eigen::Affine3f& M, const char* filename){
  ifstream iff(filename);

  if (!iff.is_open()) {
    cout << "could not open " << filename << endl;
    return false;
  }

  for (uint i=0;i<4; ++i)
    for (uint j=0;j<4; ++j)
      iff >> M(i,j);

  return true;
}




/**
*
* @param cloud
* @param color
* @param color_height
* @return
*
* @todo nice blue colors
* @todo make compatible to the other colorize cloud so that both functions can be called after
*/
//Cloud colorizeCloud(const Cloud& cloud, float min_height, const cv::Mat& color, float color_height){
// // ROS_INFO("colorize: min %f %f %f", z_min, z_max, color_height);
//
//
// Cloud result;
// cv::Mat img( cloud.height,cloud.width, CV_8UC3);
// img.setTo(0);
// cv::Vec3b col;
// col.val[1] = col.val[2] = 255;
//
// // not a point (but used to keep the cloud organized
// pcl_Point nap; nap.x = nap.y = nap.z =  std::numeric_limits<float>::quiet_NaN();
//
//
// // TODO: use color_height after thresholding as Hue or Saturation Channel
//
//
// for (uint x=0; x<cloud.width; ++x)
//  for (uint y=0; y<cloud.height; ++y){
////   pcl_Point p = cloud.at(x,y);
////   float z = p.z;
////   if (z != z || z < z_min || z > z_max) continue;
//
//   z = color.at<double>(y,x);
//
//   if (z<min_height)
//    continue;
//
//
//   col.val[0] = int(z/color_height*180)%180;
//
//   //ROS_INFO("H-value: %i", col.val[0]);
//
//   img.at<cv::Vec3b>(y,x) = col;
//  }
//
//
// cv::cvtColor(img, img, CV_HSV2BGR);
//
// // cv::namedWindow("bar");
// // cv::imshow("bar", img);
// // cv::waitKey(10);
//
// for (uint y=0; y<cloud.height ; ++y)
//  for (uint x=0; x<cloud.width ; ++x)
//   {
//   pcl_Point p = cloud.at(x,y);
//   z = color.at<double>(y,x);
//
//   if (z<min_height){
//    result.push_back(nap);
//    continue;
//   }
//
//   cv::Vec3b c = img.at<cv::Vec3b>(y,x);
//
//   p.b = c.val[0];
//   p.g = c.val[1];
//   p.r = c.val[2];
//
//   //  ROS_INFO("col %i %i: %i %i %i",x,y,p.r, p.g, p.b);
//
//   uint32_t rgb = ((uint32_t)c.val[2] << 16 | (uint32_t)c.val[1] << 8 | (uint32_t)c.val[0]);
//   p.rgb = *reinterpret_cast<float*>(&rgb);
//   //
//   //   ROS_INFO("colorizeCloud %i %i: %i %i %i",x,y, p.r, p.g, p.b);
//
//   // result.at(x,y).rgb = *reinterpret_cast<float*>(&rgb);
//
//   result.push_back(p);
//
//   //   p = result.at(x,y);
//   //   int color = *reinterpret_cast<const int*>(&(p.rgb));
//   //   uint8_t r = (0xff0000 & color) >> 16;
//   //   uint8_t g = (0x00ff00 & color) >> 8;
//   //   uint8_t b =  0x0000ff & color;
//   //   ROS_INFO("colorizeCloud2 %i %i: %i %i %i",x,y, r, g, b);
//
//   }
//
// assert(result.size() == cloud.size());
//
// result.width = cloud.width;
// result.height = cloud.height;
//
// // for (uint x=0; x<result.width; ++x)
// //  for (uint y=0; y<result.height; ++y){
// //   pcl_Point p = result.at(x,y);
// //   ROS_INFO("colorizeCloud2 %i %i: %i %i %i",x,y, p.r, p.g, p.b);
// //  }
//
//
// return result;
//}



/**
*
*
* @param color (8UC3)
* @param mesh
* @param mask point is ignored if mask is zero (8UC1)
*/
/// transfer colors to the mesh (replace by nan if mask at this position is 0)
Cloud transferColorToMesh(const cv::Mat& color, Cloud& mesh, const cv::Mat* mask){

  assert(color.cols == int(mesh.width));
  assert(color.rows == int(mesh.height));

  Cloud result;
  pcl_Point nap; nap.x = nap.y = nap.z =  std::numeric_limits<float>::quiet_NaN();


  cv::Vec3b c;
  uint32_t rgb;
  pcl_Point p;

  for (int y=0; y<color.rows; ++y)
    for (int x=0; x<color.cols; ++x)
      {

        if (mask && mask->at<uchar>(y,x) == 0){
          result.push_back(nap);
          continue;
        }

        c = color.at<cv::Vec3b>(y,x);
        p = mesh.at(x,y);

        //   ROS_INFO("Mesh: h(%i,%i) = %f", x,y,p.z);

        p.b = c.val[0];
        p.g = c.val[1];
        p.r = c.val[2];

        rgb = ((uint32_t)c.val[2] << 16 | (uint32_t)c.val[1] << 8 | (uint32_t)c.val[0]);
        p.rgb = *reinterpret_cast<float*>(&rgb);

        result.push_back(p);
      }

  assert(result.size() == mesh.size());

  result.width = mesh.width;
  result.height = mesh.height;

  return result;
}


/**
*
* @param img
* @param alpha_channel
* @param water_depth
* @param min_water_depth
* @param max_water_depth
* @param mask
* @todo implement without loops
*/
/// show watervisualization using alpha_channel
void waterVisualizationAlpha(cv::Mat& img, cv::Mat& alpha_channel, const cv::Mat& water_depth, float min_water_depth, float max_water_depth, const cv::Mat* mask){


  // alpha_channel = cv::Mat(img.rows, img.cols, CV_32FC1);
  // cv::Mat new_color = cv::Mat(img.rows, img.cols, CV_8UC3);


  //  new_color.setTo(0);

  img.setTo(0);

  float alpha;

  cv::Vec3b blue(255,0,0);
  cv::Vec3b old_col;

  for (int x=0; x<water_depth.cols; ++x)
    for (int y=0; y<water_depth.rows; ++y){

      if (mask && mask->at<uchar>(y,x) == 0) continue;

      float h = water_depth.at<double>(y,x);
      if (h != h || h <= min_water_depth) continue;


      if (h >= max_water_depth)
        alpha = 1;
      else{

        alpha = (h-min_water_depth)/(max_water_depth-min_water_depth);
        //    alpha = max(min(double(2*(h-min_water_depth)/(max_water_depth-min_water_depth)),1.0),0.0);
      }
      old_col = img.at<cv::Vec3b>(y,x);


      // interpolate with old image
      img.at<cv::Vec3b>(y,x) = blue*alpha+old_col*(1-alpha);

      // new_color.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0); // blue in BGR
      //   alpha_channel.at<float>(y,x) = alpha;
    }




}


/**
*
* @param img             current colors of cloud (will be modified)
* @param water_depth     water depth at each position (CV_64FC1)
* @param min_water_depth cells where the water is higher than this threshold will be visualized
* @param max_water_depth water depth that corresponds to full color
* @param mask            positions with mask == 0 will be ignored (8UC1)
*/
/// Visualize water depth
void waterVisualization(cv::Mat& img, const cv::Mat& water_depth, float min_water_depth, float max_water_depth, const cv::Mat* mask){


  assert(img.size == water_depth.size);
  assert(max_water_depth >= min_water_depth);

  if (mask) (assert(mask->size == img.size));

  cv::cvtColor(img,img,CV_BGR2HSV);

  cv::Vec3b col;
  col.val[2] = 255;
  col.val[0] = 120; // blue...
  float h;

  for (int x=0; x<water_depth.cols; ++x)
    for (int y=0; y<water_depth.rows; ++y){

      if (mask && mask->at<uchar>(y,x) == 0) continue;

      h = water_depth.at<double>(y,x);
      if (h != h || h <= min_water_depth) continue;

      // change saturation depending on color
      col.val[1] = max(min(int((h-min_water_depth)/(max_water_depth-min_water_depth)*255),255),20);

      img.at<cv::Vec3b>(y,x) = col;
    }

  // cv::imwrite("water_HSV.png", img);
  cv::cvtColor(img,img,CV_HSV2BGR);
  // cv::imwrite("water_BGR.png", img);

}


/**
* @param img     current colors of cloud (will be modified)
* @param height  height map of mesh (CV_32FC1)
* @param z_max   maximal z_value that will be rendered
* @param z_min   minimal z_value that will be rendered
* @param color_height  height (in m) of full color range. Colors will repeat if color_height < (z_max-z_min)
* @mask          all pixels with mask == 0 will be ignored (ignore or set to Null if all pixels should be processed) (8UC1)
*/
/// Update colors for visualization depending on height
void heightVisualization(cv::Mat& img, const cv::Mat height, float z_min, float z_max, float color_height,const cv::Mat* mask){

  assert(img.size == height.size);
  assert(z_max >= z_min); assert(color_height >= 0);

  if (mask) (assert(mask->size == img.size));

  cv::Vec3b col;
  col.val[1] = col.val[2] = 255;

  cv::cvtColor(img,img,CV_BGR2HSV);

  float z;

  for (int x=0; x<height.cols; ++x)
    for (int y=0; y<height.rows; ++y){

      if (mask && mask->at<uchar>(y,x) == 0) continue;

      z = height.at<float>(y,x);
      if (z != z || z < z_min || z > z_max) continue;

      //   col.val[0] = int((z-z_min)/(color_height-z_min)*180)%180;
      col.val[0] =  int((z)/(color_height)*180)%180;

      img.at<cv::Vec3b>(y,x) = col;
    }

  cv::cvtColor(img,img,CV_HSV2BGR);

}


/**
* @param cloud input cloud
* @param z_max points with p.z > z_max are not included in the resulting cloud
* @param z_min points with p.z < z_min are not included in the resulting cloud
* @param color_height Hue-Channel is used to compute color. Every color_height m, the colors are repeated
* @return organized copy of input cloud where the color of each point reflects its z-value. (Ignored points are replaced by NaNs)
*/
Cloud colorizeCloud(const Cloud& cloud, float z_max, float z_min, float color_height, const cv::Mat* water_depth, double max_water_height, float min_water_height){


  // ROS_INFO("colorize: min %f %f %f", z_min, z_max, color_height);


  assert(z_max >= z_min);
  assert(color_height >= 0);

  if (water_depth){
    if (!(water_depth->cols == int(cloud.width) && water_depth->rows == int(cloud.height))){
      ROS_INFO("Water: %i %i, cloud: %i %i", water_depth->cols, water_depth->rows, cloud.width, cloud.height);
    }

  }


  Cloud result;
  cv::Mat img( cloud.height,cloud.width, CV_8UC3);
  img.setTo(0);
  cv::Vec3b col;
  col.val[0] = col.val[1] = col.val[2] = 255;

  // not a point (but used to keep the cloud organized
  pcl_Point nap; nap.x = nap.y = nap.z =  std::numeric_limits<float>::quiet_NaN();

  for (uint x=0; x<cloud.width; ++x)
    for (uint y=0; y<cloud.height; ++y){
      pcl_Point p = cloud.at(x,y);
      float z = p.z;
      if (z != z || z < z_min || z > z_max) continue;



      // If there is water, use this to color the point, otherwise use the height information
      double depth;
      if (water_depth)
        depth = water_depth->at<double>(y,x);

      if (water_depth && depth > min_water_height){
        depth = max(depth, max_water_height);
        col.val[1] = 255;//int(depth/max_water_height*250);
        col.val[0] = 255; // ???
        img.at<cv::Vec3b>(y,x) = col;
      }else{
        col.val[0] = int(z/color_height*180)%180;
        col.val[1] = 255;
        img.at<cv::Vec3b>(y,x) = col;
      }

    }


  cv::cvtColor(img, img, CV_HSV2BGR);

  // cv::namedWindow("bar");
  // cv::imshow("bar", img);
  // cv::waitKey(10);

  for (uint y=0; y<cloud.height ; ++y)
    for (uint x=0; x<cloud.width ; ++x)
      {
        pcl_Point p = cloud.at(x,y);

        bool water_valid = (water_depth && water_depth->at<double>(y,x) > min_water_height);
        bool land_valid = !(p.z != p.z || p.z < z_min || p.z > z_max);

        if (!(water_valid || land_valid)){
          result.push_back(nap);
          continue;
        }

        cv::Vec3b c = img.at<cv::Vec3b>(y,x);

        // todo: Farbe wird zwei Mal gesetzt..

        p.b = c.val[0];
        p.g = c.val[1];
        p.r = c.val[2];

        uint32_t rgb = ((uint32_t)c.val[2] << 16 | (uint32_t)c.val[1] << 8 | (uint32_t)c.val[0]);
        p.rgb = *reinterpret_cast<float*>(&rgb);

        result.push_back(p);

      }

  // ROS_INFO("result: %zu, cloud: %zu", result.size(), cloud.size());
  assert(result.size() == cloud.size());

  result.width = cloud.width;
  result.height = cloud.height;

  // for (uint x=0; x<result.width; ++x)
  //  for (uint y=0; y<result.height; ++y){
  //   pcl_Point p = result.at(x,y);
  //   ROS_INFO("colorizeCloud2 %i %i: %i %i %i",x,y, p.r, p.g, p.b);
  //  }


  return result;
}


/**
* @param cloud input Pointcloud
* @param P Projection Matrix (3x4 in homogenous coordinates)
* @param img output image: every valid point is projected into the image and a small circle is drawn at this position. The color reflects the original z-value
* @param z_max maximal z-value for points that are shown in image
* @param z_min minimal z-value for points that are shown in image
* @param color_height Hue-Channel is used for coloring. Every color_height m, the colors are repeated.
*/
void projectCloudIntoImage(const Cloud& cloud, const cv::Mat& P, cv::Mat& img, float z_max, float z_min, float color_height){

  cv::Mat p(4,1,CV_64FC1);
  cv::Mat px(3,1,CV_64FC1);

  int w = img.cols;
  int h = img.rows;

  img.setTo(0);

  float z;

  for (uint i=0; i<cloud.points.size(); ++i){
    p.at<double>(0) = cloud.points[i].x;
    p.at<double>(1) = cloud.points[i].y;
    p.at<double>(2) = cloud.points[i].z;
    p.at<double>(3) = 1;

    z = cloud.points[i].z;

    if (z != z) continue;

    px = P*p;
    px /= px.at<double>(2);
    int x = px.at<double>(0); int y = px.at<double>(1);
    if (x<0 || x >= w || y < 0 || y >= h)
      continue;

    if (z<z_min || (z_max > 0 && z > z_max)) continue;


    cv::Scalar col(int(z/color_height*180)%180,255,255);

    cv::circle(img, cv::Point(x,y), 2, col,-1);

  }

  cv::cvtColor(img,img,CV_HSV2BGR);


}


/**
*
* @param y_direction direction of y_direction
* @param z_direction direction of z_axis
* @param origin origin of coordinate system
* @param transformation transformation into the system defined by the origin and the y- and z-axis (right handed system)
*/
void computeTransformationFromYZVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_direction,
                                                 const Eigen::Vector3f& origin, Eigen::Affine3f& transformation){

  Eigen::Vector3f x = (y_direction.cross(z_direction)).normalized();
  Eigen::Vector3f y = y_direction.normalized();
  Eigen::Vector3f z = z_direction.normalized();

  Eigen::Affine3f sub = Eigen::Affine3f::Identity();
  sub(0,3) = -origin[0];
  sub(1,3) = -origin[1];
  sub(2,3) = -origin[2];


  transformation = Eigen::Affine3f::Identity();
  transformation(0,0)=x[0]; transformation(0,1)=x[1]; transformation(0,2)=x[2]; // x^t
  transformation(1,0)=y[0]; transformation(1,1)=y[1]; transformation(1,2)=y[2]; // y^t
  transformation(2,0)=z[0]; transformation(2,1)=z[1]; transformation(2,2)=z[2]; // z^t

  transformation = transformation*sub;
}

/**
*
* @param depth 32FC1 image
* @param fx  focal length in x
* @param fy  focal length in y
* @param cx  center offset
* @param cy  center offset
* @return corresponding pointcloud
*
* @todo add version with color
*/
/// Creation of the pointcloud from the depth image and camera intrinsics
Cloud createCloud(const cv::Mat& depth, float fx, float fy, float cx, float cy){


  Cloud cloud;
  cloud.points.resize(depth.cols*depth.rows);


  int pos = 0;
  for (int y = 0; y< depth.rows; ++y)
    for (int x = 0; x< depth.cols; ++x){

      pcl_Point p;
      p.z = depth.at<float>(y,x);
      p.x = (x - cx) * p.z / fx;
      p.y = (y - cy) * p.z / fy;

      cloud.points[pos++] = p;
    }

  cloud.width = depth.cols;
  cloud.height = depth.rows;

  return cloud;
}

