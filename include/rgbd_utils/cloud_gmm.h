/*
 * cloud_gmm.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: lengelhan
 */

#ifndef CLOUD_GMM_CPP_
#define CLOUD_GMM_CPP_


#include "rgbd_utils/calibration_utils.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <queue>
#include <limits>

class RunningGaussian {

  double sq; /// internal helper variable
  std::queue<double> x; /// queue containing the last n measurements
  int max_queue_length; /// maximal history length

public:

  void setHistoryLength(int length){
    max_queue_length = length;
    reset();
  }

  bool initialized;
  double mean; /// mean of last max_queue_length measurements
  double var;  /// variance of last max_queue_length measurements
  double sigma(){return sqrt(var);}

  void run_test();


  void reset(){
    var = -1;
    sq = 0;
    mean = -1;
    while(!x.empty())x.pop();
    initialized = false;
  }

  RunningGaussian(int queue_length = 10){
    max_queue_length = queue_length;
    reset();
  }


  /// true if measured variance fits to distance
  /// www.ros.org/wiki/openni_kinect/kinect_accuracy
  bool isStableMeasurement();

  /**
    * @param new_x new measurement
    */
  void update(double new_x);


  /// updated gaussian if value is withing N*sigma to mean (then returns true)
  bool updateIfWithinNSigma(double value, float N);


  /// true if distance to mean not larger than N*sigma
  bool isWithinNSigma(double value, float N);




};


/// @TODO better name!
class PixelEnvironmentModel {

  std::vector<std::vector<RunningGaussian> > gaussians;
  int width_,height_;

  cv::Mat mask_;
  bool mask_set;

public:

  void init(int width, int height, int queue_length = 10);

  void resetModel();


  /// update the gaussian at (x,y) with a new value
  void update(int x,int y,float value);

  void update(const Cloud& cloud,cv::Mat* frame_mask = NULL);

  void getForeground_dist(const Cloud& cloud, float max_dist, cv::Mat& foreground);
  void getForeground_prob(const Cloud& cloud, float N, cv::Mat& foreground);


  void getDistance(const Cloud& current, cv::Mat& dists);

  void getSigmas(cv::Mat& vars, bool normalize);
  void getMeans(cv::Mat& mean);

  /// positions with mask == 0 are ignored (mask has to be CV_8UC1 and correcly sized
  void setMask(const cv::Mat& mask);

  void getCloudRepresentation(const Cloud& model, Cloud& result, float N = 1);

  void getStableMeasurements(cv::Mat& stable);
  void getInitialized(cv::Mat& inited);

  Cloud getModel(float min_x, float min_y, float cell_width, float cell_height);


  void setHistoryLength(int length){
    for (int y=0; y<height_; ++y)
      for (int x=0; x<width_; ++x){
        gaussians[y][x].setHistoryLength(length);
      }

  }

};



#endif
