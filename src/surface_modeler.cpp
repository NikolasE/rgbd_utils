/*
 * surface_modeler.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: lengelhan
 */

#include "rgbd_utils/surface_modeler.h"


using namespace std;

/// lock all cells on which the marker points in the cloud are projected into
void Elevation_map::lockCells(const cv::Mat & mask, const Cloud& current){

  if (locked.cols != mean.cols || locked.rows !=mean.rows || locked.type() == CV_8UC1){
    locked = cv::Mat(mean.rows,mean.cols, CV_8UC1);
  }

  locked.setTo(0);

  assert(mask.cols == int(current.width) && mask.rows == int(current.height));

  for (int x=0; x<mask.cols; ++x)
    for (int y=0; y<mask.rows; ++y){
      if (mask.at<uchar>(y,x) == 0) continue;
      cv::Point pos = grid_pos(current.at(x,y));
      if (pos.x < 0) continue;

      locked.at<uchar>(pos.y,pos.x) = 255;
    }

  locking_active = true;

  //  cv::namedWindow("blocked");
  //  cv::imshow("blocked",locked);

}


cv::Mat Elevation_map::getFGMask(const Cloud& cloud, float max_dist, cv::Mat* areaMask){

  cv::Mat res(cloud.height, cloud.width, CV_8UC1);
  res.setTo(0);

  // if (areaMask){
  //  areaMask = new cv::Mat(cloud.height, cloud.width, CV_8UC1);
  //  areaMask->setTo(0);
  //  ROS_INFO("Area: %i %i", areaMask->cols, areaMask->rows);
  // }

  int step = 1;

  for (uint x=0; x<cloud.width; x += step)
    for (uint y=0; y<cloud.height; y += step){
      pcl_Point p = cloud.at(x,y);
      if (p.x!=p.x) continue;

      cv::Point pos = grid_pos(p);

      if (pos.x < 0) continue;

      if (areaMask){
        areaMask->at<uchar>(y,x) = 255;
      }

      float h = mean.at<float>(pos.y,pos.x);

      if (p.z > h+max_dist){
        res.at<uchar>(y,x) = 255;
      }

    }


  if (step>1){
    cv::dilate(res, res, cv::Mat(), cv::Point(-1,-1),floor(step/2));

    if (areaMask)
      cv::dilate(*areaMask, *areaMask, cv::Mat(), cv::Point(-1,-1),floor(step/2));
  }
  return res;

}


bool Elevation_map::saveAsObj_2(std::string filename){
  Cloud model = getModel();
  std::ofstream off(filename.c_str());

  if (!off.is_open())
    return false;

  float size_factor = 1;

  int pos_in_file[model.size()];
  int valid_pnt_cnt = 0;
  for (uint i=0; i<model.size(); ++i){
    pcl_Point p = model[i];
    off << "v " << size_factor*p.x << " " << size_factor*p.y << " " << size_factor*p.z << "\r\n";
    pos_in_file[i] = valid_pnt_cnt++;
  }

  int w = model.width;
  for (uint x=0; x<model.width-1; ++x){
    for (uint y=0; y<model.height-1; ++y){
      int a_idx = x+y*w;
      int b_idx = (x+1)+y*w;
      int c_idx = x+(y+1)*w;
      int d_idx = (x+1)+(y+1)*w;

      int a = pos_in_file[a_idx]+1; // retrieve position in .obj-file
      int b = pos_in_file[b_idx]+1;
      int c = pos_in_file[c_idx]+1;
      int d = pos_in_file[d_idx]+1;

      off << "f " << a << " " << b << " " << c << "\r\n";
      off << "f " << b << " " << d << " " << c << "\r\n";
    }

  }
  
  return true;
}


#ifdef WITH_LIBGEOMETRY

/**
 *
 * @param mesh
 */
/// transfers the information (vertices, normals, triangles) to the trianglemesh
void Surface_Modeler::copyToMesh(rms::VFTriangleMesh& mesh){

  Cloud model = getModel();
  Cloud_n with_normals;
  computeNormals(model, with_normals);

  Wml::Vector3f e_z = Wml::Vector3f::UNIT_Z;

  for (uint i=0; i<model.size(); ++i){
    pcl_Point p = model[i];
    pcl_Normal n = with_normals[i];
    mesh.AppendVertex(Wml::Vector3f(p.x,p.y,p.z));

    // hack! compute normal for every point!
    if (n.normal_x == n.normal_x)
      mesh.SetNormal(i,Wml::Vector3f(n.normal_x, n.normal_y,n.normal_z));
    else
      mesh.SetNormal(i,e_z);
  }

  int w = model.width;

  // add triangles:
  for (uint x=0; x<model.width-1; ++x)
    for (uint y=0; y<model.height-1; ++y){
      // left upper triangle
      int a = x+y*w;
      int b = (x+1)+y*w;
      int c = x+(y+1)*w;
      int d = (x+1)+(y+1)*w;

      mesh.AppendTriangle(a,b,c);
      mesh.AppendTriangle(b,d,c);
    }

  ROS_INFO("Mesh has %i vertices and %i triangles", mesh.GetVertexCount(), mesh.GetTriangleCount());

}

#endif


/**
*
* @param filename
* @return
*/
/// Save model in .obj-Format with normals
bool Elevation_map::saveAsObj(std::string filename, bool ignore_triangles_without_normals){

  Cloud model = getModel();

  Cloud_n with_normals;
  //
  computeNormals(model, with_normals);
  //
  ROS_INFO("Model has %zu points, computed normals: %zu", model.size(), with_normals.size());


  std::ofstream off(filename.c_str());

  if (!off.is_open())
    return false;


  int normal_pos_in_file[model.size()];

  int valid_pnt_cnt = 0;

  // bool has_normal[model.size()];



  bool write_normals = true;


  // write points
  for (uint i=0; i<model.size(); ++i){
    pcl_Point p = model[i];
    pcl_Normal n = with_normals[i];

    off << "v " << p.x << " " << p.y << " " << p.z << "\r\n";

    // evil hack! compute normals also for border of
    if (n.normal_x != n.normal_x){
      n.normal_x = n.normal_y = 0; n.normal_z = 1;
    }

    //if (write_normals)
    off << "vn " << n.normal_x << "  " << n.normal_y << "  " << n.normal_z << "\r\n";
    normal_pos_in_file[i] = valid_pnt_cnt++;

    //    if (n.normal_x == n.normal_x){
    //      //if (write_normals)
    //      off << "vn " << n.normal_x << "  " << n.normal_y << "  " << n.normal_z << "\r\n";
    //      normal_pos_in_file[i] = valid_pnt_cnt++;
    //    }else{
    //      normal_pos_in_file[i] = -1;
    //    }

  }


  // std::set<int> invalid_normals;
  //
  // // write normals
  // for (uint i=0; i<model.size(); ++i){
  //
  //
  //  if (n.normal_x != n.normal_x)
  //   invalid_normals.insert(i+1); // .obj uses 1-based indices
  //
  //  // invalid normal is also included into the file to keep the numbering consistent
  // }


  // model.at(u,v);  v * this->width + u)
  // ROS_INFO("modelsize: %i %i", model.width, model.height);
  int w = model.width;

  // ROS_INFO("Writing faces");
  // write faces


  /**
 * TODO:
 *
 * invalid normales are in most cases the outer points that don't have enough neighbours.
 *
 */


  // bool only_faces = false;


  for (uint x=0; x<model.width-1; ++x){
    for (uint y=0; y<model.height-1; ++y){
      // left upper triangle
      int a_idx = x+y*w;
      int b_idx = (x+1)+y*w;
      int c_idx = x+(y+1)*w;
      int d_idx = (x+1)+(y+1)*w;


      //   if (only_faces){
      //    off << "f " << a << " " << b << " " << c << "\r\n";
      //    off << "f " << b << " " << d << " " << c << "\r\n";
      //
      //    continue;
      //   }


      //   bool a_normal_valid = (invalid_normals.find(a) == invalid_normals.end());
      //   bool b_normal_valid = (invalid_normals.find(b) == invalid_normals.end());
      //   bool c_normal_valid = (invalid_normals.find(c) == invalid_normals.end());
      //   bool d_normal_valid = (invalid_normals.find(d) == invalid_normals.end());

      int a = normal_pos_in_file[a_idx]; // compute position in .obj-file
      int b = normal_pos_in_file[b_idx];
      int c = normal_pos_in_file[c_idx];
      int d = normal_pos_in_file[d_idx];

      bool a_normal_valid = a >= 0; // check if point was added to the file (aka normal was valid)
      bool b_normal_valid = b >= 0;
      bool c_normal_valid = c >= 0;
      bool d_normal_valid = d >= 0;

      a+=1; b+=1; c+=1; d+=1; // oppa matlab style
      a_idx+=1;b_idx+=1;c_idx+=1;d_idx+=1;

      if (write_normals && a_normal_valid && b_normal_valid && c_normal_valid){
        // first index for point, second index for normal
        off << "f " << a_idx << "//" << a << "  " << b_idx << "//" << b << "  " << c_idx << "//" << c << "\r\n";
      }else{
        //if (!ignore_triangles_without_normals)
        off << "f " << a_idx << " " << b_idx << " " << c_idx << "\r\n";
      }

      // right lower triangle
      if (write_normals && b_normal_valid && c_normal_valid && d_normal_valid)
        off << "f " << b_idx << "//" << b << "  " << d_idx << "//" << d << "  " << c_idx << "//" << c << "\r\n";
      else{
        //if (!ignore_triangles_without_normals)
        off << "f " << b_idx << " " << d_idx << " " << c_idx << "\r\n";
      }
    }


  }





  off.close();

  ROS_INFO("Closing file");

  return true;
}




void Elevation_map::reset(){
  mean.setTo(0);
  model_3d_valid = false;
}



/**
*
* @param cloud new measurement. For each bin, the average height (h_new) of the new points falling into this bin is calculated. If the
* old height was h_old, it is updated to h = (1-weight)h_old+weight*h_new.
*
* @todo (speedup) combine with getFGMask
*
*/
bool Elevation_map::updateHeight(const Cloud& cloud, float min_diff_m){

  bool min_diff_active = min_diff_m > 0;

  update_count++;


  hits.setTo(0);
  height_sum.setTo(0);

  int step = 2; // increase size of gaussian blur accordingly


  //  cv::Mat updated(480,640,CV_8UC1);
  //  updated.setTo(0);


  if (locking_active){
    ROS_INFO("locked: %i %i, hits: %i %i", locked.rows,locked.cols, hits.rows, hits.cols);
    assert(locked.cols == hits.cols);
    assert(locked.rows == hits.rows);
    assert(locked.type() == CV_8UC1);
  }
  //  if (locking_active){
  //    ROS_INFO("Locked: %i %i, cells: %i %i", locked.cols, locked.rows, cell_cnt_x, cell_cnt_y);
  //  }

  for (uint i=0; i<cloud.size(); i += step){
    pcl_Point p = cloud[i];
    cv::Point pos = grid_pos(p);

    if (pos.x < 0) continue;
    assert(0 <= pos.y && 0 <= pos.x && pos.y < cell_cnt_y && pos.x < cell_cnt_x);


    if (locking_active){
//      if (!(pos.y < locked.rows && pos.x < locked.cols)){
//        ROS_INFO("pos: %i %i, size: %i %i", pos.x,pos.y,locked.cols, locked.rows);
//      }

      if (locked.at<uchar>(pos.y,pos.x) > 0){
        continue;
      }
    }

    height_sum.at<float>(pos.y,pos.x) += p.z;
    hits.at<float>(pos.y,pos.x)++;
    //    updated.at<uchar>(pos.y,pos.x) = 255;
  }


  //  cv::namedWindow("updated");
  //  cv::imshow("updated",updated);

  int dyn_pixel_cnt = 0;

  for (int x=0; x<cell_cnt_x; ++x)
    for(int y=0; y<cell_cnt_y; ++y){
      float hit_cnt = hits.at<float>(y,x);


      if (hit_cnt > 0){
        float height = height_sum.at<float>(y,x)/hit_cnt;

        float old = mean.at<float>(y,x);

        if (first_frame || (old != old)){

          z_min = min(height*1.0, z_min);
          z_max = max(height*1.0, z_max);

          mean.at<float>(y,x) = height;
        }
        else{

          // ignore too large updates (they correspond to the hand moving over the surface)
          if (!min_diff_active || abs(height-old) < min_diff_m){

            float h_new = (1-weight)*old+weight*height;

            z_min = min(h_new*1.0, z_min);
            z_max = max(h_new*1.0, z_max);

            mean.at<float>(y,x) = h_new;
          }else{
            dyn_pixel_cnt++;
          }

        }
      }
    }


  // ROS_INFO("Found %i dynamic pixels", dyn_pixel_cnt);

  first_frame = false;
  model_computed = true; // new model was computed
  model_3d_valid = false;// therefore, the old 3d-model is not longer valid and will be recreated on demand

  //small blur to smooth heightfield
  int size = step; if (size%2 ==0) size++;
  if (locking_active){

    // blur image but reset values corresponding to detected objects
    cv::Mat mean_unblured;

    mean.copyTo(mean_unblured);
    cv::imwrite("data/unblured.png",mean_unblured);
    cv::GaussianBlur(mean, mean, cv::Size(size,size),0,0);
    mean_unblured.copyTo(mean,locked);
    cv::imwrite("data/after_locked.png",mean);

  }else{
    cv::GaussianBlur(mean, mean, cv::Size(size,size),0,0);
  }


  // due to measurement errors, some pixels have a high error. If a hand is visible in the image,
  // more than 500 Pixels are counted.
  return dyn_pixel_cnt > 10;
}




/**
* @param y
* @param x 3D  measurement point, intrepreted as (x,y,0)
* @return corresponding bin coordinates (negative if point is not within grid)
*/
cv::Point Elevation_map::grid_pos(float x, float y){
  pcl_Point p;
  p.x = x; p.y = y; p.z = 0;
  return grid_pos(p);
}

/**
*
* @param p measurement point
* @return corresponding bin coordinates (negative if point is not within grid)
*/
cv::Point Elevation_map::grid_pos(const pcl_Point& p){

  cv::Point pos;
  pos.x = pos.y = -1;

  if (p.x != p.x){ // || p.x < x_min_ || p.x > x_max_ || p.y < y_min_ || p.y > y_max_){
    return pos;
  }

  pos.x = round((p.x - x_min_)/cell_size_);
  pos.y = round((p.y - y_min_)/cell_size_);

  if (!(pos.x >= 0 && pos.y >=0) || !(pos.x < cell_cnt_x && pos.y < cell_cnt_y)){
    pos.x = pos.y = -1;
  }

  return pos;
}




/**
* Initialization of grid from first measurement.
*
* @param cell_size length of grid cell in m
* @param cloud first cloud. size of grid is set so that all points have a minimum distance of 5cm to the border of the grid.
*/
void Elevation_map::init(float cell_size, const Cloud& cloud){

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



void Elevation_map::getModel(Cloud& model){
  model.clear();
  pcl_Point p;
  model.resize(cell_cnt_y*cell_cnt_x);

  int pos = 0;
  for (int y = 0; y < cell_cnt_y; ++y){
    p.y = y_min_+(y+0.5)*cell_size_;
    for (int x = 0; x < cell_cnt_x; ++x)
      {
        p.x = x_min_+(x+0.5)*cell_size_;
        p.z = mean.at<float>(y,x);
        //        model.push_back(p);
        model[pos++] = p;
      }
  }

  model.width = cell_cnt_x;
  model.height = cell_cnt_y;

  if (int(model.size()) != cell_cnt_x*cell_cnt_y){
    ROS_INFO("size: %zu, %i % i", model.size(),cell_cnt_x,cell_cnt_y);
    // assert(int(model.size()) == cell_cnt_x*cell_cnt_y);
  }
}


/**
* Each cell is represented as a point in the middle of the cell and the z-value of the mean of the added training points.
*  The model has to be computed befor this function is called.
*
* @return Model as organized cloud
* @see updateHeight
*/
Cloud  Elevation_map::getModel(){

  if (!model_computed){
    model_3d.clear();
    return model_3d;
  }

  if (model_3d_valid)
    return model_3d;


  model_3d.clear();
  /// @todo use old storage
  //model_3d.reserve(cell_cnt_y*cell_cnt_x);
  model_3d.clear();

  pcl_Point p;

  assert(model_3d.size() == 0);

  for (int y = 0; y < cell_cnt_y; ++y){
    p.y = y_min_+(y+0.5)*cell_size_;

    // assert(model_3d.size() == y*cell_cnt_x);
    for (int x = 0; x < cell_cnt_x; ++x)
      {
        // ROS_INFO("y: %i x: %i, size: %zu",y,x,model_3d.size());
        p.x = x_min_+(x+0.5)*cell_size_; 
        p.z = mean.at<float>(y,x);
        model_3d.push_back(p);
      }
  }

  model_3d_valid = true;

  //ROS_INFO("Size: %zu, cnt: %i %i", model_3d.size(),cell_cnt_x, cell_cnt_y);

  model_3d.width = cell_cnt_x;
  model_3d.height = cell_cnt_y;

  // assert(model_3d.size() == cell_cnt_x*cell_cnt_y);
  return model_3d;

}



/**
* Initialization of the grid
*
* @param cell_size length of cell in m
* @param x_min smallest x-value that is within the grid
* @param x_max largest x-value that is within the grid
* @param y_min smallest y-value that is within the grid
* @param y_max largest y-value that is within the grid
*/
void Elevation_map::init(float cell_size, float x_min, float x_max, float y_min, float y_max){

  x_min_ = x_min; x_max_ = x_max; y_min_ = y_min; y_max_ = y_max;
  cell_size_ = cell_size;
  cell_cnt_x = ceil((x_max-x_min)/cell_size);
  cell_cnt_y = ceil((y_max-y_min)/cell_size);

  ROS_INFO("Creating grid with %i x %i cells", cell_cnt_x, cell_cnt_y);

  mean = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1);
  mean.setTo(0);

  height_sum  = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1);
  hits  = cv::Mat(cell_cnt_y, cell_cnt_x, CV_32FC1);

  model_computed = false;
  first_frame = true;
  is_initialized = true;
}
