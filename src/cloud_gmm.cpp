/*
 * cloud_gmm.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: lengelhan
 */



#include "rgbd_utils/cloud_gmm.h"

using namespace std;


bool RunningGaussian::updateIfWithinNSigma(double value, float N){
  if (var >=0 && !isWithinNSigma(value,N))
    return false;

  update(value);

  return true;
}

bool RunningGaussian::isWithinNSigma(double value, float N){
  if (var < 0)
    return false;

  return abs(value-mean) <= N*sigma();
}

/// compute mean and variance for entries [n1...n2]
void meanVar(const vector<double>& meas, int n1, int n2, double& mean, double& var){

  assert(n1>=0 && n2>=n1 && n2 < int(meas.size()));

  int N = n2-n1;

  if (N == 0){
    mean = meas[n1];
    var = -1;
    return;
  }

  mean = var = 0;

  // compute mean
  for (int i=n1; i<=n2; ++i)
    mean += (meas[i]/(N+1));

  // compute variance:
  for (int i=n1; i<=n2; ++i){
    var += (meas[i]-mean)*(meas[i]-mean)/(N+1);
  }

}


/**
  * Test of constant time implementation by comparing it with O(n) computation
  */
void RunningGaussian::run_test(){


  std::vector<double> meas;
  int N = 300;

  for (int i=0; i<N; ++i){
    // meas.push_back(i+10);
    meas.push_back(rand()%100);
  }

  vector<double> means;
  vector<double> vars;

  double m,v;
  for (int i=0; i<N; ++i){
    int n1 = max(0,i-max_queue_length+1);
    int n2 = i;
    meanVar(meas,n1,n2,m,v);
    // ROS_INFO("n1 = %i, n2 = %i, x=%.1f, m=%.1f, var=%.2f",n1,n2,meas[i],m,v);

    means.push_back(m);
    vars.push_back(v);
  }


  reset();

  for (int i=0; i<N; ++i){
    update(meas[i]);

    double m_exp = means[i];
    double v_exp = vars[i];

    //ROS_INFO("i: %.1f exp: %.1f %.2f  got: %.1f %.2f",meas[i],m_exp,v_exp,mean,var);
    assert(abs(m_exp-mean)<1e-4 && abs(v_exp-var)<1e-4);
  }

  ROS_INFO("RunningGaussian::run_test SUCCESS!");

}


void RunningGaussian::update(double new_x){

  int N = x.size();
  assert(N <= max_queue_length);

  if (N == 0){
    x.push(new_x);
    mean = new_x;
    var = -1;
    sq = new_x*new_x;
    return;
  }

  initialized = true;

  // if number of measurements is reached, delete first measurement
  if (N == max_queue_length){
    double rem_val = x.front();
    x.pop();
    mean = (mean*N-rem_val)/(N-1);
    var = -1; // will be set in some lines...
    sq -= rem_val*rem_val;
    N -= 1;
  }


  // (N < max_queue_length)
  sq += new_x*new_x;
  mean = (mean*N+new_x)/(N+1);
  var = sq/(N+1) - mean*mean;
  x.push(new_x);

}



void PixelEnvironmentModel::init(int width, int height, int queue_length){

  gaussians.clear();

  vector<RunningGaussian> line;
  line.reserve(width);

  width_ = width; height_ = height;

  RunningGaussian gaussian(queue_length);

  for (int i=0; i<width; ++i) line.push_back(gaussian);
  for (int i=0; i<height; ++i)  gaussians.push_back(line);

  // access to last element
  assert(!gaussians[height-1][width-1].initialized);
}


/// reset of every gaussian (use init(..) to change size of model)
void PixelEnvironmentModel::resetModel(){
  for (int i=0; i<height_; ++i)
    for (int j=0; j<width_; ++j)
      gaussians[i][j].reset();
}



void PixelEnvironmentModel::update(int x,int y,float value){
  assert( x>=0 && x<width_ && y>=0 && y<height_);
  gaussians[y][x].update(value);
}


/**
* update each gaussian with the norm of the corresponding point
* points with frame_mask > 0 are not updated. This can be used to only update gaussians with
* background pixels. (use getForeground_* to compute such a mask)
*/
void PixelEnvironmentModel::update(const Cloud& cloud, cv::Mat* frame_mask){
  assert(int(cloud.width) == width_ && int(cloud.height) == height_);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;
      if (frame_mask && frame_mask->at<uchar>(y,x) > 0) continue;
      double length = norm(cloud.at(x,y));
      if (length > 0) // check for nans
        gaussians[y][x].update(length);
    }
}


void PixelEnvironmentModel::getInitialized(cv::Mat& inited){
  if (inited.rows != height_ || inited.cols != width_ || inited.type() != CV_8UC1){
    inited = cv::Mat(height_,width_,CV_8UC1);
  }

  inited.setTo(0);
  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      if (gaussians[y][x].initialized)
        inited.at<uchar>(y,x) = 255;
    }
}


void PixelEnvironmentModel::getMeans(cv::Mat& means){
  if (means.rows != height_ || means.cols != width_ || means.type() != CV_32FC1){
    means = cv::Mat(height_,width_,CV_32FC1);
  }
  means.setTo(0);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      means.at<float>(y,x) = gaussians[y][x].mean;
    }

}

void PixelEnvironmentModel::getSigmas(cv::Mat& vars, bool normalize){
  if (vars.rows != height_ || vars.cols != width_ || vars.type() != CV_32FC1){
    ROS_INFO("new sigma imags");
    vars = cv::Mat(height_,width_,CV_32FC1);
  }

  vars.setTo(0);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;

      if (gaussians[y][x].initialized)
        vars.at<float>(y,x) = gaussians[y][x].sigma();
    }


  if (normalize){
    double min_val, max_val;
    cv::minMaxLoc(vars,&min_val,&max_val, NULL,NULL, mask_);

    ROS_INFO("normalizing: %f %f", min_val, max_val);
    vars = (vars-min_val)/(max_val-min_val);

  }


}


void PixelEnvironmentModel::getDistance(const Cloud& current, cv::Mat& dists){
  if (dists.rows != height_ || dists.cols != width_ || dists.type() != CV_32FC1){
    dists = cv::Mat(height_,width_,CV_32FC1);
  }

  assert(1==0);

  dists.setTo(-100);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      // ROS_INFO("x,y: %i %i",x,y);
      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;

      if (!gaussians[y][x].initialized) continue;

      float d = norm(current.at(x,y));

      if (d < 0) continue; // nan point

      dists.at<float>(y,x) = (d-gaussians[y][x].mean);
    }


}


void PixelEnvironmentModel::getForeground_dist(const Cloud& cloud, float max_dist, cv::Mat& foreground){
  if (foreground.rows != height_ || foreground.cols != width_ || foreground.type() != CV_8UC1){
    foreground = cv::Mat(height_,width_,CV_8UC1);
  }

  foreground.setTo(0);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){
      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;

      float mean = gaussians[y][x].mean;
      bool inited = gaussians[y][x].initialized;
      // bool stable =gaussians[y][x].isStableMeasurement();
      float current = norm(cloud.at(x,y));

      if (current < 0) continue; // nan point

      // foreground if there was no measurement so far or distance to mean value is larger than max_dist
      if (!inited || current + max_dist < mean){
        foreground.at<uchar>(y,x) = 255;
      }

    }


  cv::medianBlur(foreground,foreground,3);

  //    cv::erode(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);
  //    cv::dilate(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);

}




bool RunningGaussian::isStableMeasurement(){
  if (!initialized) return false;
  double sig = sigma();
  return sig < 0.0075*mean*mean;
}




/// point is foreground if it is not within N sigma
void PixelEnvironmentModel::getForeground_prob(const Cloud& cloud, float N, cv::Mat& foreground){
  if (foreground.rows != height_ || foreground.cols != width_ || foreground.type() != CV_8UC1){
    foreground = cv::Mat(height_,width_,CV_8UC1);
  }
  foreground.setTo(0);

  for (int y=0; y<height_; ++y)
    for (int x=0; x<width_; ++x){

      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;

      bool inited = gaussians[y][x].initialized;
      float current = norm(cloud.at(x,y));
      if (current < 0) continue; // nan point

      if (!inited || (current < gaussians[y][x].mean && !gaussians[y][x].isWithinNSigma(current,N))){
        foreground.at<uchar>(y,x) = 255;
      }
    }

  cv::medianBlur(foreground,foreground,3);

//  cv::erode(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);
//  cv::dilate(foreground,foreground,cv::Mat(),cv::Point(-1,-1),2);

}






void PixelEnvironmentModel::setMask(const cv::Mat& mask){
  assert(mask.cols == width_ && mask.rows == height_ && mask.type() == CV_8UC1);
  mask.copyTo(mask_);
  mask_set = true;
}


void PixelEnvironmentModel::getCloudRepresentation(const Cloud& model, Cloud& result, float N){

  result.clear();

  uint8_t r = 255, g = 0, b = 0;
  uint32_t rgb_red = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  r = 0; g = 255;
  uint32_t rgb_green = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);


  pcl_Point nap; nap.x = std::numeric_limits<float>::quiet_NaN();

  for (int y = 0; y < height_; ++y)
    for (int x = 0; x < width_; ++x){

      if ((mask_set && mask_.at<uchar>(y,x) == 0) || !gaussians[y][x].initialized){
        if (N<0)
          result.push_back(nap);
        continue;
      }

      float mean_dist = gaussians[y][x].mean;

      // add point with same color but with norm = mean
      pcl_Point p = model.at(x,y);
      if (p.x != p.x) {
        if (N<0) // keep cloud organized if no sigma-points are included
          result.push_back(nap);
        continue;
      }
      pcl_Point mean = setLength(p,mean_dist);
      mean.rgb = p.rgb;

      result.push_back(mean);

      if (N > 0 && gaussians[y][x].initialized){
        float sigma = gaussians[y][x].sigma();

        pcl_Point near = setLength(p,mean_dist-N*sigma);
        near.rgb = *reinterpret_cast<float*>(&rgb_green);
        result.push_back(near);

        pcl_Point far = setLength(p,mean_dist+N*sigma);
        far.rgb = *reinterpret_cast<float*>(&rgb_red);
        result.push_back(far);
      }
    }


  if (N<0){
    assert(int(result.size()) == width_*height_);
    result.width = width_;
    result.height = height_;
  }


}



void PixelEnvironmentModel::getStableMeasurements(cv::Mat& stable){
  if (stable.rows != height_ || stable.cols != width_ || stable.type() != CV_8UC1){
    stable = cv::Mat(height_,width_,CV_8UC1);
  }
  stable.setTo(0);

  for (int x = 0; x < width_; ++x)
    for (int y = 0; y < height_; ++y){

      if (mask_set && mask_.at<uchar>(y,x) == 0) continue;

      if (gaussians[y][x].isStableMeasurement()){
        stable.at<uchar>(y,x) = 255;
      }
    }
}
