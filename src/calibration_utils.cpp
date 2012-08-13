/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "rgbd_utils/calibration_utils.h"


#include "fstream"

using namespace std;


void add(pcl_Point& a,const pcl_Point& b){
 a.x+=b.x; a.y+=b.y; a.z += b.z;
}

void div(pcl_Point& a, float d){
 a.x /= d; a.y /= d; a.z /= d;
}


float norm(const pcl_Point& p){
 return sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
}


pcl_Point setLength(const pcl_Point& p, float s){
 float l = norm(p);
 pcl_Point out;

 out.x = p.x/l*s;
 out.y = p.y/l*s;
 out.z = p.z/l*s;

 if (abs(norm(out)-s)> 0.01){
  ROS_INFO("old; %f %f %f", p.x,p.y,p.z);
  ROS_INFO("new; %f %f %f", out.x,out.y,out.z);
  ROS_INFO("dist: %f", abs(norm(out)-s));

 }

 return out;
}



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

//   ROS_INFO("%i %i", x,y);


   for (uint i=0; i<clouds.size(); ++i){
//    cout << "i" << i << endl;
     pcl_Point p = clouds[i].at(x,y);
//     cout << "b" << endl;
     if (p.x != p.x) continue; // NaN value
     add(mean, p);
     cnt++;
   }


   if (cnt == 0){
//    cout << "nan" << endl;
    out.at(x,y) = clouds[0](x,y); // copy a nan-point at this position
   }else{

    div(mean,cnt);
//    cout << "mean n=" << cnt << endl;
    out.at(x,y) = mean;
//    cout << "out" << cnt << endl;
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


float dist(pcl_Point A, pcl_Point B){
 return sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)+pow(A.z-B.z,2));
}


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

void applyMaskOnCloud(const cv::Mat& mask, const pcl::PointCloud<pcl::Normal>& in, pcl::PointCloud<pcl::Normal>& out){
 assert(mask.cols == int(in.width));
 out.clear();

 // int nan_cnt = 0;

 for (int x=0; x<mask.cols; ++x)
  for (int y=0; y<mask.rows; ++y){
   if (mask.at<uchar>(y,x) > 0){
    pcl::Normal p = in.at(x,y);

    //    if (p.x == p.x)
    out.push_back(p);
    //    else nan_cnt++;
   }
  }


}



void applyMaskOnCloud(const cv::Mat& mask, const Cloud& in, Cloud& out){

 assert(mask.cols == int(in.width));
 out.clear();

 // int nan_cnt = 0;

 for (int x=0; x<mask.cols; ++x)
  for (int y=0; y<mask.rows; ++y){
   if (mask.at<uchar>(y,x) > 0){
    pcl_Point p = in.at(x,y);
    if (p.x == p.x) out.push_back(p);
    //    else nan_cnt++;
   }
  }

 // ROS_INFO("start: %i, nan: %i, out: %i", in.size(), nan_cnt, out.size());

}



// compute trafo, so that fixed = trafo*moved (mean distance between corresponding points is minimized)
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




 cout << "M " << endl << M << endl;


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



void scalePixels(const vector<cv::Point2f>& pxs, cv::Mat& T, vector<cv::Point2f>& transformed){

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

cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P){
 cv::Point3f p3; p3.x = p.x(); p3.y = p.y(); p3.z = p.z();
 return applyPerspectiveTrafo(p3,P);
}


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

cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P){
 cv::Point2f px;
 applyPerspectiveTrafo(p,P,px);
 return px;
}

cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P){
 cv::Point3f p3; p3.x = p.x; p3.y = p.y; p3.z = p.z;
 return applyPerspectiveTrafo(p3,P);
}

void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_){

 cv::Mat pc (3,1,CV_64FC1);
 pc.at<double>(0) = p.x;
 pc.at<double>(1) = p.y;
 pc.at<double>(2) = 1;

 pc = H*pc;

 double z = pc.at<double>(2);

 p_.x = pc.at<double>(0)/z;
 p_.y = pc.at<double>(1)/z;
}

void printTrafo(const Eigen::Affine3f& M){
 for (uint i=0;i<4; ++i){
  for (uint j=0;j<4; ++j)
   cout << M(i,j) << " ";
  cout << endl;
 }
}

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


void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img){

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

  if (! z == z) continue;

  px = P*p;
  px /= px.at<double>(2);
  int x = px.at<double>(0); int y = px.at<double>(1);
  if (x<0 || x >= w || y < 0 || y >= h)
   continue;

  //HACK: rather change whole system
  z = -z;

  if (z<0.03) continue;


  float z_max = 1;

  cv::Scalar col(z/z_max*180,255,255);

  cv::circle(img, cv::Point(x,y), 2, col,-1);

 }

 cv::cvtColor(img,img,CV_HSV2BGR);


}


void computeTransformationFromYZVectorsAndOrigin(const Eigen::Vector3f& y_direction, const Eigen::Vector3f& z_axis,
  const Eigen::Vector3f& origin, Eigen::Affine3f& transformation){

 Eigen::Vector3f x = (y_direction.cross(z_axis)).normalized();
 Eigen::Vector3f y = y_direction.normalized();
 Eigen::Vector3f z = z_axis.normalized();

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