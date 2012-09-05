/*
 * surface_modeler.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: lengelhan
 */

#include "rgbd_utils/surface_modeler.h"


using namespace std;


void Surface_Modeler::getForeground(const Cloud& cloud, float min_prop, cv::Mat& fg_points, cv::Mat* fg_cells, Cloud* fg_cloud){


 assert(model_computed);
 ROS_INFO("0");
 if (fg_points.cols == int(cloud.width) && fg_points.rows == int(cloud.height))
  fg_points.setTo(0);
 else
  fg_points = cv::Mat(cloud.height, cloud.width, CV_8UC1,0);
 ROS_INFO("1");

 if (fg_cells){
  if (fg_cells->cols == cell_cnt_x && fg_cells->rows == cell_cnt_y)
   fg_cells->setTo(0);
  else
   *fg_cells = cv::Mat(cell_cnt_y, cell_cnt_x, CV_8UC1,0);
 }

 ROS_INFO("A");

 if (fg_cloud) fg_cloud->clear();

 for (uint x=0; x<cloud.width; ++x)
  for (uint y=0; y<cloud.height; ++y){
   pcl_Point p = cloud.at(x,y);

   cv::Point pos = grid_pos(p);
   if (pos.x < 0) continue;

   ROS_INFO("pos %i %i", pos.x, pos.y);

   float var = variance.at<float>(pos.y, pos.x);

   if (var <= 0) continue; // no training data in this cell

   float mu = mean.at<float>(pos.y, pos.x);

   ROS_INFO("mu: %f, var: %f, z: %f", mu, var, p.z);


   double pre = 1/(var*sqrt(2*M_PI));
   double prob = pre*exp(-0.5*pow((p.z-mu)/var,2));

   ROS_INFO("mu: %f, var: %f, z: %f, p: %f", mu, var, p.z, prob);

   if (prob < min_prop)
    continue;

   fg_points.at<uchar>(y,x) = 255;

   // Todo: count hits of each cell
   if (fg_cells)
    fg_cells->at<uchar>(pos.y, pos.x) = 255;

   if (fg_cloud)
    fg_cloud->push_back(p);

  }



}






int Surface_Modeler::addTrainingFrame(const Cloud& cloud){

// cv::Mat frame = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1,0);

 int step = 10;

 for (uint i=0; i<cloud.size(); i+=step){
  pcl_Point p = cloud[i];
  cv::Point pos = grid_pos(p);

  if (pos.x < 0) continue;

  dists[pos.y][pos.x].push_back(p.z);
 }

 training_data_cnt++;

 return training_data_cnt;
}


bool Surface_Modeler::computeModel(){

// ROS_INFO("Computing foreground model");


 if (training_data_cnt < 1){
  ROS_WARN("Surface_Modeler: No training data given!");
  return false;
 }

 variance.setTo(0);
 mean.setTo(0);


 uint hit_cnt = 0;
 int mean_meas_cnt = 0;

 for (int x=0; x<cell_cnt_x; ++x)
  for (int y=0; y<cell_cnt_y; ++y){
   vector<float>* ds = &dists[y][x];

   float mu = 0;
   float sigma = 0;

   uint meas_cnt = ds->size();

   //   ROS_INFO("%i %i: %i measurements", x, y, meas_cnt);

   if (meas_cnt == 0){
    mean.at<float>(y,x) = 0;
    variance.at<float>(y,x) = -1;
    continue;
   }

   hit_cnt++;
   mean_meas_cnt += meas_cnt;

   for (uint i=0; i<ds->size(); ++i){ mu += ds->at(i)/meas_cnt; }
   for (uint i=0; i<ds->size(); ++i){ sigma += pow(ds->at(i)-mu,2)/meas_cnt; }

   //   ROS_INFO("%f, %f", mu, sigma);

   mean.at<float>(y,x) = mu;
   variance.at<float>(y,x) = sqrt(sigma);

  }

// double mean = mean_meas_cnt / hit_cnt;
//
//// ROS_INFO("Mean measurement count: %f", mean);
//
// double mn, mx;
//
// cv::minMaxLoc(mean, &mn, &mx);
//
// // ROS_INFO("min: %f, max: %f", mn, mx);
//
// cv::Mat foo;
// foo = (mean-mn)/(mx-mn);
//
// cv::minMaxLoc(foo, &mn, &mx);
//
// // ROS_INFO("min: %f, max: %f", mn, mx);
//
// cv::imwrite("mean.jpg", foo*250);
//
//// ROS_INFO("%i of %i cells with measurements", hit_cnt, cell_cnt_x*cell_cnt_y);

 model_computed = true;

 return true;
}





cv::Point Surface_Modeler::grid_pos(const pcl_Point& p){

 cv::Point pos;
 pos.x = pos.y = -1;

 if (p.x != p.x){ // || p.x < x_min_ || p.x > x_max_ || p.y < y_min_ || p.y > y_max_){
  return pos;
 }

 pos.x = floor((p.x - x_min_)/cell_size_);
 pos.y = floor((p.y - y_min_)/cell_size_);

 if (!(pos.x >= 0 && pos.y >=0) || !(pos.x < cell_cnt_x && pos.y < cell_cnt_y)){
  pos.x = pos.y = -1; return pos;
 }

 return pos;
}

void Surface_Modeler::reset(){
 training_data_cnt = 0;
 variance.setTo(0);
 mean.setTo(0);

 for (int x=0; x<cell_cnt_x; ++x)
  for (int y=0; y<cell_cnt_y; ++y)
   dists[y][x].clear();

 model_computed = false;

}


void Surface_Modeler::init(float cell_size, const Cloud& cloud){

 // pcl::getMinMax3d();

 x_min_ = y_min_ = 1e6;
 x_max_ = y_max_ = -1e6;


 for (uint i=0; i<cloud.size(); ++i){
  pcl_Point p = cloud[i];
  if(p.x != p.x) continue;

  x_min_ = min(x_min_, p.x);
  y_min_ = min(y_min_, p.y);

  x_max_ = max(x_max_, p.x);
  y_max_ = max(y_max_, p.y);
 }

 // add small border:
 x_min_ -= 0.05;
 y_min_ -= 0.05;

 x_max_ += 0.05;
 y_max_ += 0.05;

 init(cell_size, x_min_, x_max_,y_min_, y_max_);
}


Cloud Surface_Modeler::getModel(){

 Cloud result;


 double mean_var = 0;
 int cnt = 0;

 double max_var = -1;

 // ROS_INFO("size: %i %i", result.width, result.height);
 for (int y = 0; y<cell_cnt_y; ++y)
 for (int x = 0; x<cell_cnt_x; ++x)
  {
   pcl_Point p;

   float m = mean.at<float>(y,x);
   float v = variance.at<float>(y,x);


   uint8_t r = 0, g = 255, b = 0;
   //
   //   cout << "M " << m << endl;
   //
   //   if (m > 0.1)
   //    r = 255;
   //   if (m > 0.2)
   //       g = 255;
   //   if (m > 0.3)
   //          b = 255;



//   if (x%2 == 0){
//    r = 255; g = 0;
//    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//    p.rgb = *reinterpret_cast<float*>(&rgb);
//   }else{
//    r = 0; g = 255;
//    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//    p.rgb = *reinterpret_cast<float*>(&rgb);
//
//   }
//
//
//   if (x==0 && y == 0){
//    r = 255; g = 255;
//    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//    p.rgb = *reinterpret_cast<float*>(&rgb);
//   }
//
//   if (x==1 && y == 0){
//     r = 255; g = 255; b = 255;
//     uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//     p.rgb = *reinterpret_cast<float*>(&rgb);
//    }
//
//   if (x==2 && y == 0){
//     b = 255;
//     uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//     p.rgb = *reinterpret_cast<float*>(&rgb);
//    }





   p.x = x_min_+(x+0.5)*cell_size_;
   p.y = y_min_+(y+0.5)*cell_size_;


//   ROS_INFO("create model: x: %i y: %i  %f %f", x,y,p.x,p.y);


   if (v < 0){
    g = 0; r = 0; b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    p.rgb = *reinterpret_cast<float*>(&rgb);
    p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
    result.push_back(p);
    continue;
   }else
    p.z = m;


   result.push_back(p);
   //result.at(x,y) = p;

   mean_var += v;
   cnt++;
   max_var = max(max_var,v*1.0);

   //   result.push_back(p);
   //
   //   g = 0; r = 255;
   //   rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
   //   p.rgb = *reinterpret_cast<float*>(&rgb);
   //
   //
   //   p.z += v;
   //   result.push_back(p);
   //
   //   p.z -= 2*v;
   //
   //   result.push_back(p);




  }


 mean_var /= cnt;

// ROS_INFO("mean variance: %f, max: %f", mean_var, max_var);


 result.width = cell_cnt_x;
 result.height = cell_cnt_y;
 return result;

}


void publishGrid(ros::Publisher& pub, std::string frame, const Cloud& cloud){


 // pcl::

}


void Surface_Modeler::init(float cell_size, float x_min, float x_max, float y_min, float y_max){
 x_min_ = x_min; x_max_ = x_max; y_min_ = y_min; y_max_ = y_max;
 cell_size_ = cell_size;
 cell_cnt_x = ceil((x_max-x_min)/cell_size);
 cell_cnt_y = ceil((y_max-y_min)/cell_size);

 ROS_INFO("Creating grid with %i x %i cells", cell_cnt_x, cell_cnt_y);

 variance = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1);
 mean = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1);

 vector<float> v;
 v.reserve(1000);
 vector<vector<float> > vectors;

 for (int x=0; x<cell_cnt_x; ++x)
  vectors.push_back(v);

 for (int y=0; y<cell_cnt_y; ++y){
  dists.push_back(vectors);
 }

 training_data_cnt = 0;
 model_computed = false;

}
