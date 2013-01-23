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

#include <limits>
#include <opencv/cv.h>
#include <opencv/highgui.h>


#ifdef WITH_LIBGEOMETRY
#include "MeshObject.h" // libgeometry
#endif



typedef std::pair<int,cv::Point2f> uv_neighbour;

//struct UV_Neighbour_cmp {
// bool operator()(const uv_neighbour& a, const uv_neighbour& b) const {return (a.first < b.first);}
//};

struct UV_Patch {

  /// id of center point
  int center_id;

  // set of neighbouring points
  // <id,(u,v)>
  mutable std::vector<uv_neighbour> neighbours;

  bool operator<(const UV_Patch& other){return center_id < other.center_id;}

};

class Surface_Modeler {



  float x_min_,x_max_, y_min_, y_max_;
  float cell_size_;
  int cell_cnt_x,cell_cnt_y;

  double z_min, z_max;


  Cloud model_3d;
  bool model_3d_valid;

  bool model_computed;

  bool first_frame;

  cv::Mat height_sum,hits;// helper for frame update



#ifdef WITH_LIBGEOMETRY
  void copyToMesh(rms::VFTriangleMesh& mesh);

  bool exp_map_initialized;
  // storage for exp-Maps
  std::map<int,UV_Patch> expMaps;
#endif


  int gridPos2expMapid(cv::Point grid_pos){
    return (grid_pos.y*cell_cnt_x+grid_pos.x);
  }

  cv::Point expMapId2gridPos(int id){
    cv::Point pos;
    pos.y = id/cell_cnt_x;
    pos.x = id%cell_cnt_x;
    return pos;
  }



//  void updateHeight_gaussian(const Cloud& cloud, float sigma_factor = -1);
//  bool updateHeight_iterpolation(const Cloud& cloud, float max_dist = -1);


public:

  bool is_initialized;


cv::Mat mean;


#ifdef WITH_LIBGEOMETRY

  bool isExpMapInitialized(){return exp_map_initialized;}
  // wrapper for expMap-Generator
  MeshObject expMapMeshObject;

  bool initExpMapGenerator();
  void writeExpMapCSV();
  UV_Patch getPatchAround(cv::Point grid_pos, float dist = 0.05);
  void expMapTestRun();

  UV_Patch getPatchAround(int x, int y, float dist = 0.05){return getPatchAround(cv::Point(x,y),dist);}

  void visualizePatch_col(const UV_Patch& patch, cv::Mat& img);
  void visualizePatch(const UV_Patch& patch, cv::Mat& mask, cv::Mat& uv_map);
#endif



  bool saveAsObj(std::string filename, bool ignore_triangles_without_normals = false);
  bool saveAsObj_2(std::string filename);


  /// returns minimal height of all cells
  float getMinHeight(){return z_min;}

  /// returns maximal height off all cells
  float getMaxHeight(){return z_max;}


  /// size of one cell
  float cell_size(){return cell_size_;}

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

  bool updateHeight(const Cloud& cloud,  float min_diff_m = -1);


  void reset();


  /**
  * Initialization, sets weight to 0.1
  *
  * @return
  * @see weight
  */
  Surface_Modeler(){
    model_computed = false;
    weight = 0.1;
    model_3d_valid = false;
#ifdef WITH_LIBGEOMETRY
    exp_map_initialized = false;
#endif
    is_initialized = false;
    //  areaMask_computed = false;
  }

  bool computeModel();

  void init(float cell_size, const Cloud& cloud);

  void init(float cell_size, float x_min, float x_max, float y_min, float y_max);


  cv::Mat getFGMask(const Cloud& cloud, float max_dist, cv::Mat* areaMask = NULL);

  // void getForeground(const Cloud& cloud, float min_prop, cv::Mat& fg_points, cv::Mat* fg_cells = NULL, Cloud* fg_cloud = NULL);

  Cloud & getModel();

  float getCellSize(){return cell_size_;}

  /**
  * @return current image with current height estimation for each cell in m as CV_32FC1 image
  */
  cv::Mat getHeightImage(){return mean;}

};


#endif /* SURFACE_MODELER_H_ */
