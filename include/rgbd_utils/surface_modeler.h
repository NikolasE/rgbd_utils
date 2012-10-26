/*
 * surface_modeler.h
 *
 *  Created on: Aug 27, 2012
 *      Author: lengelhan
 */

#ifndef SURFACE_MODELER_H_
#define SURFACE_MODELER_H_

#include "rgbd_utils/calibration_utils.h"
#include "rgbd_utils/meshing.h"


#include <opencv/cv.h>
#include <opencv/highgui.h>



class Surface_Modeler {

 float x_min_,x_max_, y_min_, y_max_;
 float cell_size_;
 int cell_cnt_x,cell_cnt_y;

 float z_min, z_max;


 std::vector<std::vector<std::vector<float> > > dists;

 cv::Mat mean;
 cv::Mat variance;

 uint training_data_cnt;
 bool model_computed;

 bool first_frame;

public:


 /// returns minimal height of all cells
 float getMinHeight(){return z_min;}

 /// returns maximal height off all cells
 float getMaxHeight(){return z_max;}

 /// get minimal x-value of grid
 float min_x(){return x_min_;}
 /// get minimal y-value of grid
 float min_y(){return y_min_;}
 /// get size of grid in x-direction
 float getWidth(){return x_max_-x_min_;}
 /// get size of grid in y-direction
 float getHeight(){return y_max_-y_min_;}


 bool modelComputed(){return model_computed;}

 cv::Point grid_pos(float x, float y);

 cv::Point grid_pos(const pcl_Point& p);


 /** Updatefactor used in updateHeight @see updateHeight*/
 float weight;

 bool updateHeight(const Cloud& cloud,  const float min_diff_m = -1);


 /**
  * @return Number of training frames added so far
  */
 uint getTrainingCnt(){return training_data_cnt;}

 /**
  * Initialization, sets weight to 0.1
  *
  * @return
  * @see weight
  */
 Surface_Modeler(){
  model_computed = false;
  training_data_cnt = 0;
  weight = 0.1;
 }

 bool computeModel();


 void init(float cell_size, const Cloud& cloud);

 void init(float cell_size, float x_min, float x_max, float y_min, float y_max);

 int addTrainingFrame(const Cloud& cloud);

 void reset();

 cv::Mat getFGMask(const Cloud& cloud, float max_dist);

 void getForeground(const Cloud& cloud, float min_prop, cv::Mat& fg_points, cv::Mat* fg_cells = NULL, Cloud* fg_cloud = NULL);

 Cloud getModel();

 float getCellSize(){return cell_size_;}

 /**
  * @return current image with current height estimation for each cell in m as CV_32FC1 image
  */
 cv::Mat getHeightImage(){return mean;};

};


#endif /* SURFACE_MODELER_H_ */
