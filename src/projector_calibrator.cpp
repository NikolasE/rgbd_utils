/*
 * projector_calibrator.cpp
 *
 *      Author: engelhan
 */


using namespace std;

#include "rgbd_utils/projector_calibrator.h"


/**
 * @brief Projector_Calibrator::eval_projection_matrix_Checkerboard
 * @param corners  Positions of detected checkerboard corners
 * @param ss       Information
 * @return Mean distance between projected pixel positions and true positions
 *
 * Evaluation function:checkerboard corners are detected in the RGB-Image, projected into the projector and compared to their correct position.
 *
 */
float Projector_Calibrator::eval_projection_matrix_Checkerboard(Cloud& corners, std::stringstream& ss){

  corners.clear();

  if (input_image.rows == 0){  ss << "can't find corners on empty image!"; return -1; }

  ROS_INFO("Looking for checkerboard with %i %i corners img: %i %i", checkboard_size.width, checkboard_size.height, input_image.cols, input_image.rows);


  cv::Mat *search_image = &input_image;
  cv::Mat masked_image;

  if (checkerboard_search_mask.cols == input_image.cols){
    ROS_INFO("searching on masked image");
    masked_image = cv::Mat(input_image.size(), input_image.type());
    masked_image.setTo(125);

    input_image.copyTo(masked_image,checkerboard_search_mask);
    search_image = &masked_image;
    cv::imwrite("data/masked_image.png", masked_image);
  }else{
    ROS_INFO("No mask defined");
  }



  cv::imwrite("data/search_image.png", *search_image);

  if (!cv::findChessboardCorners(*search_image, checkboard_size, detected_corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
    ss << "Could not find a checkerboard!";
    return -1;
  }

  cv::cvtColor(input_image, gray, CV_BGR2GRAY);
  cv::cornerSubPix(gray, detected_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 2000, 0.0001));


  //  cv::Mat distortion_image(projector_image.size(),CV_8UC3);
  //  distortion_image.setTo(255);


  float error_cv_with = 0;
  float error_cv_without = 0;
  float error_dlt_norm = 0;

  for (uint i=0; i< detected_corners.size(); ++i){
    pcl_Point mean = getInterpolatedPoint(detected_corners[i]);

    cv::Point2f px_cv_with, px_cv_without, px_dlt_with;
    cal_cv_with_dist.projectPoint(mean, px_cv_with);
    cal_cv_no_dist.projectPoint(mean, px_cv_without);
    cal_dlt_with_norm.projectPoint(mean, px_dlt_with);


    error_cv_with += dist(px_cv_with,current_projector_corners[i]);
    error_cv_without += dist(px_cv_without,current_projector_corners[i]);
    error_dlt_norm += dist(px_dlt_with,current_projector_corners[i]);

    corners.push_back(mean);

    //    cv::Point2f pr;// = applyPerspectiveTrafo(mean, proj_Matrix);
    //    cv::Point2f px_true = current_projector_corners[i];

    // ROS_INFO("point %f %f %f projected: %f %f, corrected %f %f",mean.x,mean.y,mean.z,pr.x,pr.y,corrected.x,corrected.y);

    int r = 20;
    drawMarker(projector_image,px_cv_with,CV_RGB(0,255,0),r);
    drawMarker(projector_image,px_cv_without,CV_RGB(0,0,255),r);
    drawMarker(projector_image,px_dlt_with,CV_RGB(255,0,0),r);



    //    // ROS_INFO("Mean: %f %f %f px: %f %f", mean.x,mean.y,mean.z, pr.x,pr.y);

    //    error +=

    //
    //    //    cv::line(projector_image, cv::Point(pr.x,pr.y-r),cv::Point(pr.x,pr.y+r),CV_RGB(0,0,255),3);
    //    //    cv::line(projector_image, cv::Point(pr.x-r,pr.y),cv::Point(pr.x+r,pr.y),CV_RGB(0,0,255),3);

    //    //    cv::circle(projector_image, pr, r, CV_RGB(255,0,0), 4);
    //        cv::circle(projector_image, px_cv_with, 3, CV_RGB(255,0,0), -1);
    //cv::circle(projector_image, px_cv_with, 3, CV_RGB(255,0,0), -1);
    //cv::circle(projector_image, px_cv_with, 3, CV_RGB(255,0,0), -1);
    //    cv::circle(projector_image, corrected, 3, CV_RGB(0,255,0), -1);


    //    cv::line(distortion_image,px_true,pr,CV_RGB(0,0,0),1);
    //    //    cv::line(distortion_image,measured,px,CV_RGB(0,0,0),1);


    //    //    cv::circle(distortion_image,measured,2,CV_RGB(255,0,0),-1);
    //    cv::circle(distortion_image,px_true,2,CV_RGB(0,255,0),-1);
    //    cv::circle(distortion_image,pr,2,CV_RGB(255,0,0),-1);

  }


  //cv::imwrite("board_evaluation.png", distortion_image);

  error_cv_with /= current_projector_corners.size();
  error_cv_without /= current_projector_corners.size();
  error_dlt_norm /= current_projector_corners.size();

  ROS_INFO("errors: CV_with: %.3f, CV_without: %.3f, DLT: %.3f",error_cv_with,error_cv_without,error_dlt_norm);

  //  ROS_INFO("Mean error: %.2f", error);
  return -1;
}


/**
 * @brief Projector_Calibrator::eval_projection_matrix_disc
 * @param mean_c    3d Position corresponding to the center of the disc
 * @param ss        Additional Debug Infos
 * @param area_mask (optional) mask, areas with mask == 0 are ignored
 * @return   true iff disc was found
 *
 * Evaluation procedure: The center of the disc is found in the input_image. (Image is thresholded, and center of
 * largest contour is used as center. Adjust threshold so that there is only one large contour). The corresponding
 * 3D-Pose in the Cloud is extracted (using some local interpolation) and the corresponding pixel in the projector
 * is set to green.
 *
 * @see input_image, eval_brightness_threshold
 *
 */
bool Projector_Calibrator::eval_projection_matrix_disc(Cloud& mean_c, std::stringstream& ss, const cv::Mat* area_mask){


  mean_c.clear();

  cv::Mat bw;

  cv::cvtColor(input_image, bw, CV_BGR2GRAY);

  cv::Mat cpy2;
  bw.copyTo(cpy2);


  if (area_mask && area_mask->cols == input_image.cols)
    {
      cv::Mat foo;
      bw.copyTo(foo, *area_mask);
      bw = foo;
    }

  cv::Mat thres;

  cv::threshold(bw, thres, eval_brightness_threshold, 255,  CV_THRESH_BINARY);

  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  cv::Mat cpy;
  thres.copyTo(cpy);

  cv::findContours(cpy, contours, hierarchy, CV_RETR_TREE,  CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::namedWindow("foreground");
  cv::imshow("foreground", thres);
  cv::waitKey(10);

  if (contours.size() == 0)
    {
      ss << "Found no contours, adapt threshold or adjust mask";
      return false;
    }


  int cnt_large_contours = 0;

  float min_area = 400;

  // ROS_INFO("Found %zu contours",contours.size());

  int largest_contour = -1;
  float largest_area = -1;
  for (uint i=0; i<contours.size(); ++i){
    float area = cv::contourArea(contours[i]);
    if (area < min_area)
      continue;
    else{
      cnt_large_contours++;
      if (area > largest_area){
        largest_area = area;
        largest_contour = i;
      }
    }
  }


  if (cnt_large_contours != 1){
    ss << "Found " << cnt_large_contours << " large contours, adapt threshold or adjust mask";
    return false;
  }


  cv::drawContours(thres, contours, largest_contour, cv::Scalar::all(125), 2, 8, hierarchy, 0, cv::Point());

  input_image.copyTo(cpy);

  // compute center of contour
  cv::Moments mom = cv::moments(contours[largest_contour]);
  cv::Point2f mom_center = cv::Point2f(mom.m10 / mom.m00, mom.m01 / mom.m00);

  cv::circle(cpy, mom_center, 10, CV_RGB(0,255,0), 2);

  cv::line(cpy, cv::Point(mom_center.x,0),cv::Point(mom_center.x,cpy.cols),CV_RGB(255,0,0),1);
  cv::line(cpy, cv::Point(0,mom_center.y),cv::Point(cpy.rows, mom_center.y),CV_RGB(255,0,0),1);


  pcl_Point mean = getInterpolatedPoint(mom_center);

  projector_image.setTo(0);

  cv::Point2f px;
  if (cal_cv_with_dist.projectPoint(mean, px)){
    cv::circle(projector_image, px, 2, CV_RGB(0,255,0), -1);
  }

  if (cal_cv_no_dist.projectPoint(mean, px)){
    cv::circle(projector_image, px, 2, CV_RGB(0,0,255), -1);
  }


  if (cal_dlt_with_norm.projectPoint(mean, px)){
    cv::circle(projector_image, px, 2, CV_RGB(255,0,0), -1);
  }



  ss << "Marker Position: " << mean.x << " " << mean.y << " " << mean.z;


  //  cv::Point2f px;// = applyPerspectiveTrafo(mean, proj_Matrix);
  //  cv::circle(projector_image, px, 2, CV_RGB(0,255,0), -1);


  mean_c.push_back(mean);

  cv::namedWindow("Marker Evaluation");
  cv::imshow("Marker Evaluation", cpy);
  cv::waitKey(10);


  return true;

}



/**
 * @brief Storage of found 2d/3d-Point pairs to calib/obs.txt
 * @return true if file could be written
 *
 * File Format:
 *
 * N number of images from which the points have been extracted
 * n_1  # number of points in i.th image
 *...
 * n_N  # number of points in i.th image
 *
 * 3d-coordinates of all pairs  (sum n_i lines)
 * 2d-coordinates of all pairs  (sum n_i lines)
 */
bool Projector_Calibrator::saveObservations(){

  ofstream off("calib/obs.txt");

  off << number_of_features_in_images.size() << endl;
  for (uint i=0; i<number_of_features_in_images.size(); ++i)
    off << number_of_features_in_images[i] << endl;

  for (uint i=0; i<observations_3d.size(); ++i){
    pcl_Point p = observations_3d[i];
    off << p.x << " " << p.y << " " << p.z << endl;
  }

  for (uint i=0; i<corners_2d.size(); ++i){
    off << corners_2d[i].x << " " << corners_2d[i].y << endl;
  }

  return true;

}

/**
 * @brief Loading of all stored 2d-3d-pairs
 * @return true if file could be read from calib/obs.txt
 *
 */
bool Projector_Calibrator::loadObservations(){

  number_of_features_in_images.clear();
  observations_3d.clear();
  corners_2d.clear();

  ifstream iff("calib/obs.txt");
  int img_cnt; iff >> img_cnt;

  int total = 0;
  int cnt;
  for (int i=0; i<img_cnt; ++i){
    iff >> cnt;
    number_of_features_in_images.push_back(cnt);
    total += cnt;
  }

  ROS_INFO("Reading %i observations for %i image(s)",total,img_cnt);


  pcl_Point p;
  for (int i=0; i<total; ++i){
    iff >> p.x >> p.y >> p.z;
    observations_3d.push_back(p);
  }

  cv::Point2f px;
  for (int i=0; i<total; ++i){
    iff >> px.x >> px.y;
    corners_2d.push_back(px);
  }

  ROS_INFO("Loaded %i pairs in %i images", total, img_cnt);


  return img_cnt>0;
}


Projector_Calibrator::Projector_Calibrator(){
  kinect_orientation_valid = false;
  kinect_trafo_valid = false;
  depth_cam_model_set = false;

  // checkboard_size = cv::Size(10,6);
  // proj_size = cv::Size(1024,768);

  // char foo[100];
  // std::string cwd = getcwd(foo,100);
  //
  // cout << "cwd " << cwd << endl;

  ros::param::param<bool>("projector_calibration/load_everything_on_startup", load_everything_on_startup, true);

  // reading the number of corners from file
  // int check_width, check_height;
  ros::param::param<int>("projector_calibration/checkerboard_internal_corners_x", max_checkerboard_width, 8);
  ros::param::param<int>("projector_calibration/checkerboard_internal_corners_y", max_checkerboard_height, 6 );
  checkboard_size = cv::Size(max_checkerboard_width, max_checkerboard_height);

  // reading the projector's size from file
  int proj_width, proj_height;
  ros::param::param<int>("projector_calibration/projector_px_width", proj_width, 1400); // 1024
  ros::param::param<int>("projector_calibration/projector_px_height", proj_height, 1050); // 768
  proj_size = cv::Size(proj_width, proj_height);

  double marker_size;
  ros::param::param<double>("projector_calibration/printed_marker_corners_dist_mm", marker_size, 25);
  printed_marker_size_mm = marker_size;

  setKinectOrientation(0);


  projector_image = cv::Mat(proj_size, CV_8UC3);

  hom_cv_filename = "homography_opencv";
  hom_svd_filename = "homography_svd.yml";
  proj_matrix_filename = "projection_matrix";
  kinect_trafo_filename = "kinect_trafo";

  test_img = cv::imread("/work/home/engelhan/ros/Touchscreen/imgs/Testbild.png");

  if (!test_img.data){
    ROS_ERROR("Could not open test image at /work/home/engelhan/ros/Touchscreen/imgs/Testbild.png!");
  }
}


/**
 * @brief Projector_Calibrator::initFromFile
 * @param msg  Debug Information
 *
 *  Initialization of the Calibrator from file (data/ -folder)
 *
 *  - Transformation (Kinect-World)
 *  - Projection matrix
 *  - Homography
 *
 * @see kinect_trafo_filename,proj_matrix_filename,hom_cv_filename
 *
 */
void Projector_Calibrator::initFromFile(std::stringstream& msg){



  mask = cv::imread("data/kinect_mask.png",0);
  if (mask.data){
    ROS_INFO("Found mask (%i %i)", mask.cols, mask.rows);
  }else {
    ROS_INFO("Could not find mask at data/kinect_mask.png");
  }

  // Check for Kinect trafo:
  char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
  kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);

  if (kinect_trafo_valid)
    msg << "Loaded transformation from kinect to world frame";
  else
    msg << "Could not load transformation from kinect to world frame from " <<  fn;


  cal_dlt_no_norm.loadFromFile("cal_dlt_no_norm");
  cal_dlt_with_norm.loadFromFile("cal_dlt_with_norm");
  cal_cv_with_dist.loadFromFile("cal_cv_with_dist");
  cal_cv_no_dist.loadFromFile("cal_cv_no_dist");

  //  msg << endl;

  //  // load Matrices
  //  if (loadMat("data/", proj_matrix_filename, proj_Matrix))
  //    msg << "Projection matrix was loaded" << endl;

  //  if (loadMat("data/", hom_cv_filename, hom_CV))
  //    msg << "Homography was loaded" << endl;



  //  ROS_INFO("Loading OpenCV-Calibration");
  //  loadMat("data/", "proj_matrix_opencv", proj_Matrix);
  //  loadMat("data/","camera_matrix_cv",camera_matrix_CV);
  //  loadMat("data/","proj_trans",rvec_CV);
  //  loadMat("data/","proj_rot",tvec_CV);
  //  loadMat("data/","distortion",distCoeffs_CV);


  // loadMat("Homography (SVD)", hom_svd_filename, hom_SVD);
}


/**
 * @brief Kinect-World-Transformation is loaded from data/ $kinect_trafo_filename
 * @param msg  Debug-Info
 * @return true if file was found
 *
 * @see kinect_trafo_filename
 */
bool Projector_Calibrator::loadKinectTrafo(std::stringstream& msg){


  char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
  kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);

  if (kinect_trafo_valid){
    msg << "Loaded transformation from kinect to world frame";
    publishWorldFrame("/openni_rgb_camera_frame","/fixed_frame");

    publishProjectorFrame("/fixed_frame","/projector");

  }else
    msg << "Could not load transformation from kinect to world frame from " <<  fn;

  return kinect_trafo_valid;
}


/**
 * @brief Creation of Checkerboard on the given image
 * @param img  Image on which the Checkerboard will be drawn
 * @param l1  outer corner of board
 * @param l2  other outer corner of board
 * @param size  Number of corner in both directions
 * @param corners_2d  (output) Position of corners (refined by cv::cornerSubPix)
 *
 * Checkerboard is drawn on the image and the created corners refined by cv::cornerSubPix an returned.
 *
 */
void Projector_Calibrator::drawCheckerboard(cv::Mat& img,cv::Point l1, const cv::Point l2, const cv::Size size, vector<cv::Point2f>& corners_2d){

  corners_2d.clear();
  detected_corners.clear();
  // ROS_INFO("clearing detected corders");

  float min_x = min(l1.x,l2.x);
  float min_y = min(l1.y,l2.y);

  float max_x = max(l1.x,l2.x);
  float max_y = max(l1.y,l2.y);


  // draw white border with this size
  // "Note: the function requires some white space (like a square-thick border,
  // the wider the better) around the board to make the detection more robust in various environment"
  float border = 40;

  img.setTo(255);


  // cv::rectangle(img, cv::Point(min_x-20,min_y-20), cv::Point(max_x+20,max_y+20), cv::Scalar::all(255),-1);


  if (min_x > border && min_y > border && max_x < img.cols - border && max_y < img.rows-border)
    border = 0;


  float width = ((max_x-min_x)-2*border)/(size.width+1);
  float height = ((max_y-min_y)-2*border)/(size.height+1);

  float minx = border+min_x;
  float miny = border+min_y;


  // ROS_INFO("GRID: W: %f, H: %f", width, height);

  // start with black square
  for (int j = 0; j<=size.height; j++)
    for (int i = (j%2); i<size.width+1; i+=2){

      cv::Point2f lu = cv::Point2f(minx+i*width,miny+j*height);
      cv::Point2f rl = cv::Point2f(minx+(i+1)*width,miny+(j+1)*height);
      cv::rectangle(img, lu, rl ,cv::Scalar::all(0), -1);

      cv::Point2f ru = cv::Point2f(rl.x,lu.y);

      if (j==0) continue;
      if (i>0){
        corners_2d.push_back(cv::Point2f(lu.x, lu.y));
        //        cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
      }
      if (i<size.width){
        corners_2d.push_back(cv::Point2f(ru.x, ru.y));
        //        cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
      }
    }

  assert(int(corners_2d.size()) == size.width*size.height);

  // improve position of corners: (improvement of reprojection error of about 0.2 px)
  cv::Mat gray;
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  cv::cornerSubPix(gray, corners_2d, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,2000, 0.0001));

  //  cv::circle(img,corners_2d.at(0),10,CV_RGB(255,0,0),-1);
  //  cv::circle(img,corners_2d.at(1),10,CV_RGB(0,255,0),-1);
}




// set wall_region to same ratio as image
bool Projector_Calibrator::setupImageProjection(float width_m, float off_x_m, float off_y_m, const cv::Size& img_size){
  return setupImageProjection(width_m, width_m/img_size.width*img_size.height, off_x_m, off_y_m, img_size);
}




bool Projector_Calibrator::setupImageProjection(const cv_RectF& wall_area, const cv::Size& img_size){

  if (wall_area.width == 0){
    ROS_ERROR("setupImageProjection: wall area has width of 0!");
    return false;
  }
  return setupImageProjection(wall_area.width, wall_area.height, wall_area.x, wall_area.y, img_size);
}

bool Projector_Calibrator::setupImageProjection(float width_m, float height_m, float off_x_m, float off_y_m, const cv::Size& img_size){


  //  if(!projMatorHomSet()){
  //    ROS_WARN("setupImageProjection: Neither Projection Matrix nor Homography computed!");
  //    return false;
  //  }

  //  int width_px  = img_size.width;
  //  int height_px = img_size.height;

  //  float height_m_2 = width_m/width_px*height_px;

  //  if (fabs(height_m_2-height_m) > 0.1){
  //    ROS_WARN("setupImageProjection: Image and wall-section have different ratios!");
  //  }

  //  if (homOpenCVSet() || homSVDSet()){

  //    // Compute from Homography:
  //    cv::Mat px_to_world(cv::Size(3,3), CV_64FC1);
  //    px_to_world.setTo(0);

  //    // ROS_INFO("Computing warp from Homography!");

  //    px_to_world.at<double>(2,2) = 1;
  //    px_to_world.at<double>(0,0) = width_m/width_px;
  //    px_to_world.at<double>(0,2) = off_x_m;
  //    px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
  //    px_to_world.at<double>(1,2) = off_y_m;

  //    if (homOpenCVSet())
  //      warp_matrix = hom_CV*px_to_world;
  //    else
  //      warp_matrix = hom_SVD*px_to_world;

  //    warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

  //    // cout << "Warp_matrix" << endl << warp_matrix << endl;

  //    return true;

  //  }

  //  cv::Mat px_to_world(cv::Size(3,4), CV_64FC1);
  //  px_to_world.setTo(0);

  //  //  ROS_INFO("Computing warp from Projection Matrix!");

  //  px_to_world.at<double>(3,2) = 1;
  //  px_to_world.at<double>(0,0) = width_m/width_px;
  //  px_to_world.at<double>(0,2) = off_x_m;
  //  px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
  //  px_to_world.at<double>(1,2) = off_y_m;

  //  warp_matrix = proj_Matrix*px_to_world;
  //  warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

  //  //  cout << "Warp_matrix: " << endl << warp_matrix << endl;

  return true;



}

void Projector_Calibrator::showUnWarpedImage(const cv::Mat& img){

  if (!warpMatrixSet()){
    ROS_INFO("showUnWarpedImage: call setupImageProjection first.."); return;
  }

  // clear projector image
  projector_image.setTo(0);

  cv::Size size(projector_image.cols, projector_image.rows);

  cv::warpPerspective(img, projector_image, warp_matrix, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT);


  int l = unused_pixels_rows;
  cv::rectangle(projector_image, cv::Point(1,l),cv::Point(projector_image.cols-2,projector_image.rows-l), CV_RGB(255,255,0), 1);


  // IplImage img_ipl = projector_image;
  // cvShowImage("fullscreen_ipl", &img_ipl);

}


bool Projector_Calibrator::computeHomography_OPENCV(float& mean_error){
#define DO_SCALING
  assert(observations_3d.size() == corners_2d.size());
  ROS_WARN("COMPUTING HOMOGRAPHY WITH openCV");

  if (number_of_features_in_images.size() > 1){
    ROS_INFO("More than one image in list, but computing Homography only from first image!");
  }

  if (number_of_features_in_images.size() == 0){
    ROS_INFO("No observations to compute homography from!");
    return false;
  }


  uint N = number_of_features_in_images[0];

  // count number of 3d points which are more than 2cm away from the z=0-plane
  float z_max = 0.03; int cnt = 0;
  for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
  if (cnt>N*0.1) {  ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


  // copy the 3d observations (x,y,~0) into 2d (x,y) (z==0 by construction of coordinate system)
  vector<cv::Point2f> src; src.reserve(N);
  for (uint i=0; i<N; ++i){src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));}

  // 2d = H*3d H*(x,y,1)


  vector<cv::Point2f> first_proj_corners;
  first_proj_corners.insert(first_proj_corners.begin(),corners_2d.begin(), corners_2d.begin()+N);

#ifdef DO_SCALING
  cv::Mat T,U;
  vector<cv::Point2f> src_trafoed, d2_trafoed;
  scalePixels(src,  T, src_trafoed);
  scalePixels(first_proj_corners,  U, d2_trafoed);
  hom_CV = cv::findHomography(src_trafoed,d2_trafoed);
  hom_CV = U.inv()*hom_CV*T;
#else
  hom_CV = cv::findHomography(src,first_proj_corners);
#endif


  // cout << "Homography with OpenCV: " << endl << hom_CV << endl;

  // compute error:
  mean_error = 0;

  cv::Point2f px;
  float err_x = 0;
  float err_y = 0;

  float e_x, e_y;

  for (uint i=0; i<N; ++i){
    applyHomography(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y),hom_CV, px);

    e_x = abs((px.x-current_projector_corners.at(i).x));
    e_y = abs((px.y-current_projector_corners.at(i).y));

    err_x += e_x/N; err_y += e_y/N ;
    mean_error += sqrt(e_x*e_x+e_y*e_y)/N;

    //  ROS_INFO("Proj: %f %f, goal: %f %f (Error: %f)", px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
  }


  ROS_INFO("mean error: %f (x: %f, y: %f)", mean_error, err_x, err_y);

  saveMat("data/", hom_cv_filename, hom_CV);

  return true;
}


bool Projector_Calibrator::computeHomography_SVD(){
#define SCALE_SVD

  ROS_WARN("COMPUTING HOMOGRAPHY WITH SVD");

  assert(observations_3d.size() == corners_2d.size());

  if (number_of_features_in_images.size() == 0){
    ROS_INFO("No image with observations!");
    return false;
  }


  if (number_of_features_in_images.size() > 1){
    ROS_INFO("More than one image in list, but computing Homography only from first image!");
  }


  // uint N = current_projector_corners.size();
  // if(observations_3d.size() < N){
  //  ROS_ERROR("computeHomography_SVD: less 3d-points than 2d-points, aborting");
  //  return false;
  // }

  int N = number_of_features_in_images[0];


  // count number of 3d points which are more than 2cm away from the z=0-plane
  float z_max = 0.03; int cnt = 0;
  for (int i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
  if (cnt>N*0.1) { ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


  vector<cv::Point2f> first_proj_corners;
  first_proj_corners.insert(first_proj_corners.begin(),corners_2d.begin(), corners_2d.begin()+N);

#ifdef SCALE_SVD
  cv::Mat T;
  vector<cv::Point2f> d2_trafoed;
  scalePixels(first_proj_corners,  T, d2_trafoed);
#else
  vector<cv::Point2f> d2_trafoed = first_proj_corners;
#endif



  // Pointcloud to 2d-points (z ==0, by construction of the coordinate system)
  vector<cv::Point2f> src, src_trafoed;
  for (int i=0; i<N; ++i) src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));

#ifdef SCALE_SVD
  cv::Mat U;
  scalePixels(src, U,src_trafoed);
#else
  src_trafoed = src;
#endif


  cv::Mat A = cv::Mat(2*N,9,CV_64FC1);
  A.setTo(0);

  // p_ cross H*p = 0
  for (int i=0; i<N; ++i){
    cv::Point2f P = src_trafoed.at(i);
    cv::Point2f p = d2_trafoed.at(i);

    // ROS_INFO("P: %f %f,  p: %f %f", P.x, P.y, p.x, p.y);

    float f[9] = {0,0,0,-P.x,-P.y,-1,p.y*P.x,p.y*P.y,p.y};
    for (uint j=0; j<9; ++j) A.at<double>(2*i,j) = f[j];

    float g[9] = {P.x,P.y,1,0,0,0,-p.x*P.x,-p.x*P.y,-p.x};
    for (uint j=0; j<9; ++j) A.at<double>(2*i+1,j) = g[j];
  }
  // now solve A*h == 0
  // Solution is the singular vector with smallest singular value

  cv::Mat h = cv::Mat(9,1,CV_64FC1);
  cv::SVD::solveZ(A,h);


  // h only fixed up to scale -> set h(3,3) = 1;
  h /= h.at<double>(8);

  // cout << "h: " << h << endl;

  hom_SVD = cv::Mat(3,3,CV_64FC1);

  for (uint i=0; i<3; ++i){
    for (uint j=0; j<3; ++j)
      hom_SVD.at<double>(i,j) =  h.at<double>(3*i+j);
  }

  // cout << "Tinv " << T.inv() << endl;
  // cout << "U " << U << endl;

  // undo scaling
#ifdef SCALE_SVD
  hom_SVD = T.inv()*hom_SVD*U;
#endif

  // 2d = H*3d H*(x,y,1)


  //  cout << "Homography with SVD: " << endl << hom_SVD << endl;


  // compute error:
  float error = 0;
  float err_x = 0;
  float err_y = 0;

  float a,b;




  cv::Point2f px;
  for (int i=0; i<N; ++i){
    applyHomography(src.at(i), hom_SVD, px);

    a = abs(px.x-current_projector_corners.at(i).x);
    b = abs(px.y-current_projector_corners.at(i).y);

    err_x += a/N; err_y += b/N;

    error += sqrt(a*a+b*b)/N;

    //ROS_INFO("src: %f %f, Proj: %f %f, goal: %f %f (Error: %f)", src.at(i).x, src.at(i).y, px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
  }

  ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);

  saveMat("data/", hom_svd_filename, hom_SVD);

  return true;
}



cv::Point2f   undistort(float x, float y, float fx, float fy,float cx, float cy, cv::Mat dist_coeffs){



  assert(dist_coeffs.cols == 4 || dist_coeffs.cols == 5);
  assert(dist_coeffs.type() == CV_64FC1);

  // cout << "coeffs" << endl << dist_coeffs <<endl;

  float k1 = dist_coeffs.at<double>(0);
  float k2 = dist_coeffs.at<double>(1);
  float p1 = dist_coeffs.at<double>(2);
  float p2 = dist_coeffs.at<double>(3);
  float k3 = (dist_coeffs.cols == 5)?dist_coeffs.at<double>(4):0;


  //  float r2 = x*x+y*y;

  float xc = x-cx;
  float yc = y-cy;


  float r2 = xc*xc+yc*yc;

  ROS_INFO("undistort %f %f  %f:",x,y,r2);

  float x_ = xc*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p1*xc*yc+p2*(r2*+2*xc);
  float y_ = yc*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p2*xc*yc+p1*(r2*+2*yc);


  ROS_INFO("x_,y_: %f %f",x_,y_);

  cv::Point2f res;
  res.x = fx*x_+cx;
  res.x = fy*y_+cy;

  return res;


}





/**
 * @brief Compute Projection Matrix using OpenCV-functions
 * @param mean_error (output) mean reprojection error
 * @param with_distortion  flag if distortion parameters are allowed (true) or if they should be fixed to zero (false)
 * @return true if projection matrix could be computed
 *
 * Computation of Projection matrix based on cv::calibrateCamera.
 *
 * corners_2d contains the exact corners on the projector image (drawCheckerboard)
 * observations_3d contains the corresponding measured 3D-Points
 *
 * @see observations_3d, number_of_features_in_images, corners_2d, drawCheckerboard
 *
 */
bool Projector_Calibrator::computeProjectionMatrix_OPENCV(float& mean_error, bool with_distortion){


  ROS_WARN("COMPUTING Projection Matrix with openCV");
  assert(observations_3d.size() == corners_2d.size());

  if (observations_3d.size() == 0){
    ROS_WARN("Can't compute projection matrix without observations");
    return false;
  }

  // compute range in z-direction
  float z_min = 1e5; float z_max = -1e5;
  for (uint i=0; i<observations_3d.size(); ++i){
    pcl_Point p = observations_3d[i]; if (p.x!=p.x) continue;
    z_min = min(z_min,p.z); z_max = max(z_max,p.z);
  }

  if (abs(z_max-z_min) < 0.1){
    ROS_WARN("computeProjectionMatrix_OPENCV: Observations have to little variation in z! (%f to %f)", z_min, z_max);
    // return false;
  }

  vector<vector<cv::Point3f> > world_points;
  vector<vector<cv::Point2f> > pixels;
  vector<cv::Mat> rvecs, tvecs;

  vector<cv::Point3f> pt_3;

  cv::Point3f d3;
  //  All observations are measured in the same coordinate system so that all points create
  //  a single large calibration object. All pairs are therefore concatenated into a single observation.

  for (uint i=0; i<observations_3d.size(); ++i){
    pcl_Point p = observations_3d[i];
    d3.x = p.x;   d3.y = p.y;  d3.z = p.z;
    pt_3.push_back(d3);
  }

  world_points.push_back(pt_3);
  pixels.push_back(corners_2d);



  //  calibration.camera_matrix = cv::Mat(3,3,CV_32FC1);
  //  calibration.proj_trafo = cv::Mat(3,4,CV_64FC1);
  //  calibration.distCoeffs = cv::Mat(1,5,CV_32FC1);
  //  calibration.distCoeffs.setTo(0);


  Calibration cal;

  cal.camera_matrix.setTo(0);
  cal.camera_matrix.at<float>(0,2) = proj_size.width/2;
  cal.camera_matrix.at<float>(1,2) = proj_size.height/2;
  cal.camera_matrix.at<float>(0,0) = 3000;
  cal.camera_matrix.at<float>(1,1) = 3000;
  cal.camera_matrix.at<float>(2,2) = 1;

  cout << "init intrinicsss" << cal.camera_matrix << endl;

  int flag;

  if (with_distortion)
    flag = CV_CALIB_USE_INTRINSIC_GUESS; //  | CV_CALIB_FIX_ASPECT_RATIO;
  else
    flag = CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3;


  ROS_INFO("proj: %i %i",proj_size.width,proj_size.height);
  cout << "init intrinic" << cal.camera_matrix << endl;

  cv::calibrateCamera(world_points, pixels, proj_size, cal.camera_matrix, cal.distCoeffs, rvecs, tvecs, flag);

  cout << "camera: " << cal.camera_matrix << endl;
  cout << "distortion: " << cal.distCoeffs << endl;

  // build projection matrix:
  cout << "position: " << tvecs[0] << endl;

  assert(cal.camera_matrix.type() == CV_64FC1);

  cv::Rodrigues(rvecs[0], cal.rotMatrix);

  cv::hconcat(cal.rotMatrix, tvecs[0], cal.proj_trafo);

  cal.proj_matrix =  cal.camera_matrix*cal.proj_trafo;
  cal.proj_matrix /= cal.proj_matrix.at<double>(2,3);

  //    cout << "P: " << endl << proj_matric_cv << endl;

  int N = observations_3d.size();

  vector<cv::Point2f> reprojected;

  cal.rvec = rvecs[0];
  cal.tvec = tvecs[0];

  cv::projectPoints(pt_3,cal.rvec,cal.tvec,cal.camera_matrix,cal.distCoeffs,reprojected);


  mean_error = 0;
  float total_x = 0;
  float total_y = 0;


  cv::Mat distortion_image(projector_image.size(),CV_8UC3);
  distortion_image.setTo(255);


  assert(cal.camera_matrix.type() == CV_64FC1);

  cout << cal.camera_matrix <<endl;

  //  float f_x = camera_matrix_CV.at<double>(0,0);
  //  float f_y = camera_matrix_CV.at<double>(1,1);
  //  float c_x = camera_matrix_CV.at<double>(0,2);
  //  float c_y = camera_matrix_CV.at<double>(1,2);

  //  ROS_INFO("cam: f,x: %f %f %f %f",f_x,f_y,c_x,c_y);


  //  cv::Mat ts = cv::Mat::zeros(0,3,CV_32FC1);
  //  cv::Mat rs = cv::Mat::zeros(0,3,CV_32FC1);
  //  cv::Mat f = cv::Mat::ones(0,3,CV_32FC1);

  //  vector<cv::Point2f> redistorted;
  //  cv::projectPoints(reprojected,rs,ts,f,distCoeffs_CV,redistorted);




  for (int i=0; i<N; ++i){
    cv::Point2f projected = reprojected[i];
    cv::Point2f measured = corners_2d.at(i);





    cv::Point2f  px = applyPerspectiveTrafo(observations_3d.points.at(i),cal.proj_matrix);

    cv::Point2f undis = undistort(px.x,px.y,cal.f_x(),cal.f_y(),cal.c_x(),cal.c_y(),cal.distCoeffs);





    ROS_INFO("Opencv: %f %f, self: %f %f",projected.x,projected.y,undis.x,undis.y);

    // ROS_INFO("proj: %.1f %.1f (selbst: %.1f %.1f) meas: %.1f %.1f", projected.x,projected.y, px.x,px.y,measured.x,measured.y);


    if (with_distortion){
      cv::line(distortion_image,measured,projected,CV_RGB(0,0,0),1);
      cv::line(distortion_image,measured,px,CV_RGB(0,0,0),1);


      cv::circle(distortion_image,measured,2,CV_RGB(255,0,0),-1);
      cv::circle(distortion_image,projected,2,CV_RGB(0,255,0),-1);
      cv::circle(distortion_image,px,2,CV_RGB(0,0,255),-1);
    }

    total_x += abs(projected.x-measured.x)/N;
    total_y += abs(projected.y-measured.y)/N;

    mean_error += sqrt(pow(projected.x-measured.x,2)+pow(projected.y-measured.y,2))/N;
  }


  if (with_distortion){
    //cv::resize(distortion_image,distortion_image,cv::Size(),0.5,0.5);
    cv::imwrite("projection_error.png",distortion_image);
  }

  //    cout << "sandbox position " << tvecs[0] << endl;

  //  cv::Mat cam,trans, rot;
  //  cv::decomposeProjectionMatrix(proj_matrix_cv, cam,rot, trans);
  //  trans /= trans.at<double>(3);

  //  cout << "OPENCV" << endl;
  //  cout << "cam " << endl << camera_matrix_CV << endl << cam << endl;
  //  cout << "rot " << endl << rotMatrix_CV << endl << rot << endl;
  //  cout << "trans " << endl << tvecs[0] << endl << trans << endl;


  // invert transformation so that the pose of the projector is given
  //  rotMatrix_CV = rotMatrix_CV.inv();
  //  projector_position_CV = -rotMatrix_CV.inv()*tvecs[0];

  //    cout << "ProjectorPosition: " << projector_position_CV << endl;
  //    cout << "ProjectorRotation: " << rotMatrix_CV << endl;

  ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", mean_error, total_x, total_y);

  if (with_distortion)
    cal_cv_with_dist = cal;
  else
    cal_cv_no_dist = cal;


  return true;
}



/**
 * @brief Compute Projection Matrix using the DLT-Algorithm
 * @param mean_error (output) mean reprojection error
 * @param do_scaling  flag if normalization should be used
 * @return true if projection matrix could be computed
 *
 * Computation of Projection matrix based the DLT-Algorithm. (minimizing algebraic error)
 *
 * corners_2d contains the exact corners on the projector image (drawCheckerboard)
 * observations_3d contains the corresponding measured 3D-Points
 *
 * @see observations_3d, number_of_features_in_images, corners_2d, drawCheckerboard
 *
 */
bool Projector_Calibrator::computeProjectionMatrix(float& mean_error, bool do_scaling){

  ROS_INFO("COMPUTING Projection Matrix");
  assert(observations_3d.size() == corners_2d.size());


  if (number_of_features_in_images.size() == 0){
    ROS_WARN("Can't compute projection matrix without observations");
    return false;
  }

  // get range in z:
  float z_min = 1e5; float z_max = -1e5;
  for (uint i=0; i<observations_3d.size(); ++i){
    pcl_Point p = observations_3d[i]; if (p.x!=p.x) continue;
    z_min = min(z_min,p.z); z_max = max(z_max,p.z);
  }

  if (abs(z_max-z_min) < 0.1){
    ROS_WARN("computeProjectionMatrix: Observations have to little variation in z! (%f to %f)", z_min, z_max);
    return false;
  }


  uint N = observations_3d.size();
  ROS_INFO("Computing Projection matrix from %zu images and %u pairs", number_of_features_in_images.size(),N);


  Cloud trafoed_corners;
  vector<cv::Point2f> trafoed_px;
  cv::Mat U,T;

  if (do_scaling){
    scaleCloud(observations_3d, U, trafoed_corners);
    scalePixels(corners_2d, T, trafoed_px);
  }
  else{
    U = cv::Mat::eye(4,4,CV_64FC1); // 3d in homog coords
    T = cv::Mat::eye(3,3,CV_64FC1); // 2d in homog coords
    trafoed_corners = observations_3d;
    trafoed_px = corners_2d;
  }

  cv::Mat A = cv::Mat(2*N,12,CV_64FC1);
  A.setTo(0);

  //ROS_ERROR("Projection:");

  // p_ cross H*p = 0
  for (uint i=0; i<N; ++i){
    pcl_Point   P = trafoed_corners.at(i);
    cv::Point2f p = trafoed_px.at(i);

    //  ROS_INFO("from %f %f %f to %f %f", P.x,P.y,P.z,p.x,p.y);

    float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,-1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
    for (uint j=0; j<12; ++j) A.at<double>(2*i,j) = f[j];

    float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
    for (uint j=0; j<12; ++j) A.at<double>(2*i+1,j) = g[j];
  }

  // now solve A*h == 0
  // Solution is the singular vector with smallest singular value

  cv::Mat h = cv::Mat(12,1,CV_64FC1);
  cv::SVD::solveZ(A,h);



  Calibration &cal = do_scaling?cal_dlt_with_norm:cal_dlt_no_norm;
  cal.proj_matrix = cv::Mat::zeros(3,4,CV_64FC1);

  for (uint i=0; i<3; ++i){
    for (uint j=0; j<4; ++j)
      cal.proj_matrix.at<double>(i,j) =  h.at<double>(4*i+j);
  }

  // cout << "Tinv " << T.inv() << endl;
  // cout << "U " << U << endl;


  cal.proj_matrix = T.inv()*cal.proj_matrix*U;  // undo scaling
  cal.proj_matrix /= cal.proj_matrix.at<double>(2,3); // defined up to scale

  mean_error = 0;
  double total_x = 0;
  double total_y = 0;

  cv::Point2f px;

  Normal_dist<float> eval_x, eval_y;

  for (uint i=0; i<N; ++i){

    pcl_Point   p = observations_3d.points.at(i);
    cv::Point2f p_ = corners_2d.at(i);

    applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),cal.proj_matrix,px);

    // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

    eval_x.add(px.x-p_.x);
    eval_y.add(px.y-p_.y);

    total_x += abs(px.x-p_.x)/N;
    total_y += abs(px.y-p_.y)/N;
    mean_error += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/N;

  }

  eval_x.evaluate();
  eval_y.evaluate();

  if (!eval_x.isProbablyZeroCentered() || abs(eval_x.mean) > 10){
    ROS_WARN("Projection matrix: check values of x-direction!");
    ROS_WARN("Err in x: mu = %f, var: %f", eval_x.mean, eval_x.variance);
  }


  if (!eval_y.isProbablyZeroCentered()|| abs(eval_y.mean) > 10){
    ROS_WARN("Projection matrix: check values of y-direction!");
    ROS_WARN("Err in y: mu = %f, var: %f", eval_y.mean, eval_y.variance);
  }

  ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", mean_error, total_x, total_y);
  // saveMat("Projection Matrix", proj_matrix_filename, proj_Matrix);
  // saveMat("data/", proj_matrix_filename, proj_Matrix);




  cv::decomposeProjectionMatrix(cal.proj_matrix, cal.camera_matrix,cal.rotMatrix, cal.projector_position);

  cal.projector_position /= cal.projector_position.at<double>(3);
  cal.projector_position = cal.projector_position.rowRange(0,3);
  cal.camera_matrix /= cal.camera_matrix.at<double>(2,2);

  // compute camera pose in sandbox frame
  cal.print();




  /*
 cv::Mat P_(proj_Matrix.colRange(0,3));
 P_ = proj_Matrix.colRange(cv::Range(0,3));
// proj_Matrix.copyTo(P_);

 cout << "sub matrix: " << P_ << endl;

 cv::Mat F_(proj_Matrix.colRange(3,4));
 F_ = proj_Matrix.colRange(cv::Range(3,4));
 cout << "F_ matrix: " << F_ << endl;

cv::Mat new_pose = P_.inv()*F_;

cout << "new_pose " << new_pose << endl;
  */



  // do some testing:
  // cv::Point3f out;
  // project3D(cv::Point2f(0,0), proj_Matrix,1, out);
  // project3D(cv::Point2f(100,100), proj_Matrix,1, out);
  // project3D(cv::Point2f(400,123), proj_Matrix,1, out);




  return true;

}


/**
 * @brief Chessboard is detected on the input_image using cv::findChessboardCorners and cv::cornerSubPix
 * @return true iff detection was successfull
 *
 * The detected points are stored in detected_corners
 *
 * @see detected_corners, input_image, detected_corners
 */
bool Projector_Calibrator::findCheckerboardCorners(){
  // #define SHOW_DETECTIONS
  detected_corners.clear();

  if (input_image.rows == 0){  ROS_WARN("can't find corners on empty image!"); return false;  }

  ROS_INFO("Looking for checkerboard with %i %i corners img: %i %i", checkboard_size.width, checkboard_size.height, input_image.cols, input_image.rows);


  cv::Mat *search_image = &input_image;
  cv::Mat masked_image;


  if (checkerboard_search_mask.cols == input_image.cols){
    // ROS_INFO("searching on masked image");
    masked_image = cv::Mat(input_image.size(), input_image.type());
    masked_image.setTo(0);

    input_image.copyTo(masked_image,checkerboard_search_mask);
    search_image = &masked_image;
    cv::imwrite("data/masked_search_image.png", masked_image);
  }else{
    // ROS_INFO("No mask defined");
  }



  if (!cv::findChessboardCorners(*search_image, checkboard_size,detected_corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
    ROS_WARN("Could not find a checkerboard!");
    return false;
  }

  cv::cvtColor(input_image, gray, CV_BGR2GRAY);
  cv::cornerSubPix(gray, detected_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 2000, 0.0001));

  createMaskFromDetections();


#ifdef SHOW_DETECTIONS
  cv::Mat cpy = input_image.clone();
  cv::drawChessboardCorners(cpy, checkboard_size, detected_corners, true);
  cv::namedWindow("search",1); cv::imshow("search", cpy);
  cv::waitKey(-1);
#endif



  return true;
}


/**
 * @brief Write  hom_CV to data/$hom_cv_filename
 * @param msg
 * @return true if file was written
 */
bool Projector_Calibrator::saveHomographyCV(std::stringstream& msg){
  if (hom_CV.cols == 0){
    msg << "Homography is not yet computed!";
    return false;
  }

  if (saveMat("data/", hom_cv_filename, hom_CV)){
    msg << "Homography was written to " << hom_cv_filename;
    return true;
  }else{
    msg << "Problems writing Homography to " << hom_cv_filename;
    return false;
  }

}

/**
 * @brief Write  proj_Matrix to data/$proj_matrix_filename
 * @param msg
 * @return true if file was written
 */
bool Projector_Calibrator::saveCalibrations(std::stringstream& msg){


  cal_dlt_no_norm.writeToFile("cal_dlt_no_norm");
  cal_dlt_with_norm.writeToFile("cal_dlt_with_norm");
  cal_cv_with_dist.writeToFile("cal_cv_with_dist");
  cal_cv_no_dist.writeToFile("cal_cv_no_dist");


  return true;

  //  if (proj_Matrix.cols == 0){
  //    msg << "Projection matrix is not yet computed!";
  //    return false;
  //  }


  //  if (saveMat("data/", proj_matrix_filename, proj_Matrix)){
  //    msg << "Projection Matrix was written to " << proj_matrix_filename;
  //    return true;
  //  }else{
  //    msg << "Problems writing Projection Matrix to " << proj_matrix_filename;
  //    return false;
  //  }


  //  ROS_INFO("Storing OpenCV calibration");





  //  saveMat("data/", "proj_matrix_opencv", proj_Matrix);
  //  saveMat("data/","camera_matrix_cv",camera_matrix_CV);
  //  saveMat("data/","proj_trans",rvec_CV);
  //  saveMat("data/","proj_rot",tvec_CV);
  //  saveMat("data/","distortion",distCoeffs_CV);



}


pcl_Point Projector_Calibrator::getInterpolatedPoint(cv::Point2f pos){
  // todo: also interpolate point at border of image!
  //ROS_INFO("POS: %f %f size: %i %i", pos.x, pos.y, input_cloud.height, input_cloud.width);
  assert(pos.x > 0 && pos.y > 0 && pos.x < input_cloud.width-1 && pos.y < input_cloud.height-1);
  assert(depth_cam_model_set);


  pcl_Point mean;

  // create depth image:
  cv::Mat depth(3,3,CV_32FC1);

  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy){
      float d = norm(input_cloud.at(pos.x+dx,pos.y+dy));
      depth.at<float>(dx+1,dy+1) = d;
    }

  cv::Mat patch;
  cv::Point2f pos_2(pos.x-floor(pos.x)+1,pos.y-floor(pos.y)+1);
  cv::getRectSubPix(depth, cv::Size(1,1), pos_2, patch);
  float new_depth = depth.at<float>(0,0);


  cv::Point3d ray = depth_cam_model.projectPixelTo3dRay(pos);
  mean.x = ray.x; mean.y = ray.y; mean.z = ray.z;
  mean = setLength(mean,new_depth);

  Cloud c; c.push_back(mean);
  pcl::getTransformedPointCloud(c,kinect_trafo,c);
  return c[0];
}




/**
 * @brief Extraction of the corresponding 3d-points to the detected chessboard corners
 * @return true if points have been extracted
 * @param interpolation_size size of interpolatio area of depth value
 * @param inter_type interpolation type (simple: mean point of surronding pixels, INTER_BI: bilinear interpolation)
 * for each detected point in (detected_corners), the corresponding point in cloud_moved is extracted and stored in
 *
 */
bool Projector_Calibrator::storeCurrentObservationPairs(INTERPOLATION_TYPE inter_type, int interpolation_size ){


  assert(inter_type != INTER_SIMPLE || interpolation_size > 0);


  // for each 3d observation, there is a projected pixel (in current image)
  assert(current_projector_corners.size() == detected_corners.size());
  assert(current_projector_corners.size() > 0);

  if (!kinect_trafo_valid){
    ROS_WARN("can't store Observations in global Frame since it is not defined!");
    return false;
  }

  if (detected_corners.size() == 0){
    ROS_WARN("Can't add Observations since Corners havn't been detected");
    return false;
  }

  if (input_cloud.size() == 0){
    ROS_WARN("storeCurrent3DObservations: input_cloud.size() == 0");
    return false;
  }


  int l = interpolation_size;

  Cloud c_3d; pcl_Point p;
  for (uint i=0; i<detected_corners.size(); ++i){

    p = input_cloud.at(detected_corners[i].x, detected_corners[i].y);

    if (inter_type == INTER_NON){
      if (!(p.x == p.x)){
        ROS_WARN("storeCurrent3DObservations: Found Corner without depth!");
        return false;
      }
      c_3d.points.push_back(p);
      continue;
    }


    cv::Point2f px = detected_corners[i];

    if (inter_type == INTER_SIMPLE){
      pcl_Point mean(0,0,0);

      int valid = 0;
      for (int x = px.x-l;x <= px.x+l ; ++x )
        for (int y = px.y-l;y <= px.y+l ; ++y )
          {
            if (x < 0 || (x > int(input_cloud.width)) || y < 0 || (y > int(input_cloud.height)) ) continue;

            pcl_Point foo = input_cloud.at(x, y);
            if (foo.x != foo.x)  continue;

            valid++;
            add(mean, foo);
          }

      if (valid < 3){
        ROS_WARN("no depth value on corner %i",i);
        return false;
      }

      div(mean, valid);
      c_3d.points.push_back(mean);
    }


    if (inter_type == INTER_BI){
      pcl_Point mean_bi = getInterpolatedPoint(px);
      //    ROS_INFO("direct: %f %f %f, inter1: %f %f %f, inter_bi: %f %f %f", p.x,p.y,p.z,mean.x,mean.y,mean.z,mean_bi.x,mean_bi.y,mean_bi.z);
      c_3d.points.push_back(mean_bi);
    }

  }

  assert(corners_2d.size() == observations_3d.size());

  // getInterpolatedPoint returns point already in world-frame
  if (inter_type != INTER_BI){
    pcl::getTransformedPointCloud(c_3d,kinect_trafo,c_3d);
  }

  // add new observations and pixel positions
  corners_2d.insert(corners_2d.end(), current_projector_corners.begin(), current_projector_corners.end());
  observations_3d.insert(observations_3d.end(), c_3d.begin(),c_3d.end());

  // observations_3d.points.insert(observations_3d.points.end(),c_3d.points.begin(), c_3d.points.end());
  ROS_INFO("Added %zu points, now %zu 3D-Observations",c_3d.size(), observations_3d.size());

  number_of_features_in_images.push_back(current_projector_corners.size());

  ROS_INFO("Image cnt: %zu", number_of_features_in_images.size());

  return true;
}

/**
 * @brief Clears all stored information on 2d/3d-pairs
 */
void Projector_Calibrator::restartCalibration(){
  number_of_features_in_images.clear();
  observations_3d.clear();
  corners_2d.clear();
}

/**
 * @brief Remove pairs from last image
 * @return true if there was an image to remove
 */
bool Projector_Calibrator::removeLastObservations(){


  if (number_of_features_in_images.size() == 0)
    return false;


  uint obs_in_last = *number_of_features_in_images.rbegin();

  ROS_INFO("removing %i pairs, total # of pairs: %zu", obs_in_last,observations_3d.size());

  //uint old_size = observations_3d.size();

  // end() points 1 after the last, s.t. obs.end()-1 is last valid iterator

  observations_3d.erase(observations_3d.end()-obs_in_last-1,observations_3d.end()-1);
  corners_2d.erase(corners_2d.end()-obs_in_last-1,corners_2d.end()-1);

  // ROS_INFO("erased, now: 2d: %zu 3d: %zu", corners_2d.size(), observations_3d.size());

  // remove last entry in list
  number_of_features_in_images.erase(number_of_features_in_images.begin()+number_of_features_in_images.size()-1);

  // assert(observations_3d.size() + obs_in_last == old_size);
  assert(observations_3d.size() == corners_2d.size());

  if (observations_3d.size() == 0)
    ROS_WARN("removed last image");

  return true;

}


bool z_comp(pcl_Point A, pcl_Point B){return A.z > B.z;}



/**
 * @brief Set projector image by projecting every point of the world-Cloud into the projector.
 * @return Colored cloud that can be shown in RVIZ
 *
 */
Cloud Projector_Calibrator::visualizePointCloud(){

  Cloud coled;

  projector_image.setTo(0);


  if (!cloud_moved.size()>0){
    ROS_INFO("No cloud!"); return coled;
  }

  // min dist from wall
  float max_dist = 0.25; // needed to scale color

  // sort cloud in z-direction

  // std::vector<pcl_Point> sorted; sorted.reserve(cloud_moved.size());
  // for (uint i=0; i<cloud_moved.size(); ++i) {
  //  pcl_Point p = cloud_moved[i];
  //  if (p.x!=p.x) continue;
  //  sorted.push_back(p);
  // }

  // std::sort(sorted.begin(), sorted.end(), z_comp);


  for (uint x=0; x<cloud_moved.width; ++x)
    for (uint y=0; y<cloud_moved.height; ++y){
      if (mask.at<uchar>(y,x) == 0) continue;

      pcl_Point p = cloud_moved.at(x,y);


      float z = -p.z;// z points into wall

      // cout << z << endl;


      // dont't show groundplane
      if (z<0 || z>max_dist)
        continue;

      cv::Point2f px;
      applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),cal_cv_no_dist.proj_matrix,px);

      int x = px.x; int y = px.y;


      if (x<0 || y<0 || x>=projector_image.cols || y >= projector_image.rows) continue;

      //  cv::Vec3b col = projector_image.at<cv::Vec3b>(y,x);
      //
      //  if (col.val[0]>0) continue;

      //  p.b = p.r = p.g = 0;


      //  if (z<min_dist)
      //   p.r = 255;
      //  else
      //   if (z<0.5)
      //    p.g = 255;
      //   else
      //    p.b = 255;

      // px.y -= 5;
      //  cv::circle(projector_image, px, 10, CV_RGB(p.r,p.g,p.b),-1);


      //  projector_image.at<cv::Vec3b>(px.y,px.x) = cv::Vec3b((int((z*3/max_dist)*180))%180,255,255);

      cv::circle(projector_image, px, 3, cv::Scalar((int(((z)/max_dist*1.5)*180))%180,255,255) ,-1);

      //   cv::circle(projector_image, px, 3, CV_RGB(255,0,0),-1);


      //coled.push_back(p);


    }

  cv::cvtColor(projector_image,projector_image, CV_HSV2BGR);


  // IplImage proj_ipl = projector_image;
  // cvShowImage("fullscreen_ipl", &proj_ipl);

  return coled;

}



/**
 * @brief Computation of transformation between Kinect and Sandbox
 * @param msg  Debug INFO
 * @return true if trafo was computed
 *
 * Calibration Procedure:
 *
 * - create flat surface and project a pattern on it (e.g. sandsurface or just a cardboard)
 * - call this function
 * - a plane is fitted to all points within the area of the pattern (defining z-plane)
 * - center point of pattern is used as origin of new world-system
 * - rightmost corner of patten (with same height as center point) is used to calculate y-direction
 * - x is computed to create right-handed system
 *
 * @see kinect_trafo
 */
bool  Projector_Calibrator::computeKinectTransformation(std::stringstream& msg){

  if (!kinect_orientation_valid){
    ROS_INFO("Can't compute KinectTrafo without Kinect's orientation angle");
    return false;
  }

  if (detected_corners.size() == 0){
    msg << "can't compute Kinect trafo without observed corners";
    // ROS_WARN("can't compute Kinect trafo without observed corners");
    return false;
  }


  Cloud filtered;
  applyMaskOnInputCloud(filtered);

  Eigen::Vector4f plane_model;
  fitPlaneToCloud(filtered, plane_model);

  int w = checkboard_size.width;
  int h = checkboard_size.height;

  ROS_INFO("Checkerboard size: w,h: %i %i",w,h);


  int m = (h/2*w)+(w-1)/2; // center of pattern
  int right = (h/2*w)+(w-1); // rightmost corner with same height as center point

  pcl_Point p  = input_cloud.at(detected_corners[m].x, detected_corners[m].y);
  pcl_Point p2 = input_cloud.at(detected_corners[right].x, detected_corners[right].y);
  //  pcl_Point p2 = input_cloud.at(detected_corners[m].x+sin(-kinect_tilt_angle_deg/180*M_PI)*100, detected_corners[m].y-cos(-kinect_tilt_angle_deg/180*M_PI)*100);

  kinect_frame_points.clear();
  kinect_frame_points.push_back(p);
  kinect_frame_points.push_back(p2);

  if ( p2.x != p2.x){
    msg << "NAN in pointcloud, no calculation of new world-frame" << endl;
    ROS_WARN("NAN in pointcloud, no calculation of new world-frame");
    return false;
  }

  Eigen::Vector3f pl_center = Eigen::Vector3f(p.x,p.y,p.z);
  Eigen::Vector3f pl_right = Eigen::Vector3f(p2.x-p.x,p2.y-p.y,p2.z-p.z);


  float plane_direction = plane_model.head<3>()[2]>0?1:-1;   // z-axis upwards

  pcl::getTransformationFromTwoUnitVectorsAndOrigin(pl_right,-plane_direction*plane_model.head<3>(), pl_center,kinect_trafo);

  pcl::getTransformedPointCloud(input_cloud,kinect_trafo,cloud_moved);
  kinect_trafo_valid = true;

  return true;
}



bool Projector_Calibrator::publishProjectorFrame(const std::string& kinect_frame, const std::string& projector_frame){

  if (cal_cv_no_dist.proj_trafo.cols != 4){
    // ROS_INFO("publishProjectorFrame trafo size: %i %i",proj_trafo_CV.rows,proj_trafo_CV.cols);
    return false;
  }

  Eigen::Affine3f eigen;
  mat2Eigen(cal_cv_no_dist.proj_trafo,eigen);

  Eigen::Affine3f inv = pcl::getInverse(eigen*kinect_trafo);
  sendTrafo(kinect_frame, projector_frame,inv);

  return true;

}



/**
 * @brief Sending Kinect-World-Trafo as tf-Frame
 * @param kinect_frame  name of Kinect-Frame
 * @param world_frame   name of new Frame
 * @return false if transformation was not yet computed
 */
bool Projector_Calibrator::publishWorldFrame(const std::string& kinect_frame, const std::string& world_frame){
  if (!kinect_trafo_valid){
    ROS_WARN("Can't publish kinect trafo since it is not yet set!");
    return false;
  }

  Eigen::Affine3f inv = pcl::getInverse(kinect_trafo);
  sendTrafo(kinect_frame,world_frame, inv);

  return true;
}



/**
 * @brief Computation of image region in which the just detected pattern is visible
 * @param pts (output)  corner points of image region with pattern
 * @see checkboard_size, detected_corners
 */
void Projector_Calibrator::getCheckerboardArea(vector<cv::Point2i>& pts){

  pts.clear();
  if (detected_corners.size() == 0){
    ROS_WARN("getCheckerboardArea: no corners!"); return;
  }

  // find corners of checkerboard (OpenCV only returns inner corners..)

  int w = checkboard_size.width;
  int h = checkboard_size.height;


  cv::Point2i p = detected_corners[0]; // first corner in top row
  cv::Point2i q = detected_corners[w+1]; // second corner in second row
  pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y)); // extrapolation to find left upper corner

  p = detected_corners[w-1];
  q = detected_corners[2*w-2];
  pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));


  p = detected_corners[w*h-1];
  q = detected_corners[w*h-1-w-1];
  pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));


  p = detected_corners[(h-1)*w];
  q = detected_corners[(h-2)*w+1];
  pts.push_back(cv::Point2i(2*p.x-q.x,2*p.y-q.y));

  assert(pts.size() == 4);
}



bool Projector_Calibrator::getProjectionAreain3D(Cloud& corners){

  vector<cv::Mat> Corners_3d;

  if (!getProjectionAreain3D(Corners_3d)) return false;

  pcl_Point p;

  for (uint i=0;i<Corners_3d.size(); ++i){
    p.x = Corners_3d[i].at<double>(0);
    p.y = Corners_3d[i].at<double>(1);
    p.z = Corners_3d[i].at<double>(2);

    //  ROS_INFO("corner: %f %f %f", p.x,p.y,p.z);
    corners.push_back(p);
  }

  return true;


}


bool Projector_Calibrator::getProjectionAreain3D(vector<cv::Mat>& corners){


  if (!homOpenCVSet()){ return false;}

  cv::Mat hom_inv = hom_CV.inv();

  cv::Mat p(3,1,CV_64FC1);
  cv::Mat p_3d(3,1,CV_64FC1);


  // projector does not show lowest n pixels and top n pixels are under the OS-bar


  int space_vertical = unused_pixels_rows;
  int space_horizontal = 10;


  // for each projector corner, find the corresponding 3d point on the z=0 plane
  p.at<double>(0) = space_horizontal; p.at<double>(1) = space_vertical; p.at<double>(2) = 1;
  p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

  p.at<double>(0) = proj_size.width-space_horizontal; p.at<double>(1) = space_vertical; p.at<double>(2) = 1;
  p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

  p.at<double>(0) = proj_size.width-space_horizontal; p.at<double>(1) = proj_size.height-space_vertical; p.at<double>(2) = 1;
  p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

  p.at<double>(0) = space_horizontal; p.at<double>(1) = proj_size.height-space_vertical; p.at<double>(2) = 1;
  p_3d = hom_inv*p; p_3d/=p_3d.at<double>(2); p_3d.at<double>(2) = 0; corners.push_back(p_3d.clone());

  return true;

}


bool Projector_Calibrator::findOptimalProjectionArea2(cv::Mat::MSize img_px_size, std::stringstream& msg){

  if (!homOpenCVSet()){
    msg << "Homography is not set!"; return false;
  }

  float ratio = img_px_size[1]*1.0/img_px_size[0];


  // get 3d Poses of the projector corners:
  vector<cv::Mat> Corners_3d;
  getProjectionAreain3D(Corners_3d);


  double min_x = 1e10;
  double min_y = 1e10;

  assert(Corners_3d.size() == 4);

  for (uint i=0; i<Corners_3d.size(); ++i){
    min_x = min(min_x, Corners_3d[i].at<double>(0));
    min_y = min(min_y, Corners_3d[i].at<double>(1));
  }

  vector<cv::Point2i> px_coordinates;
  int max_x, max_y;
  max_x = max_y = -1;

  for (uint i=0; i<Corners_3d.size(); ++i){
    px_coordinates.push_back(cv::Point2f((Corners_3d[i].at<double>(0)-min_x)*100,(Corners_3d[i].at<double>(1)-min_y)*100)); // in cm <=> 1px
    max_x = max(max_x, int(px_coordinates[i].x));
    max_y = max(max_y, int(px_coordinates[i].y));
  }

  // ROS_INFO("max: %i %i",max_y,max_x);
  cv::Mat search_img(max_y,max_x,CV_8UC1); search_img.setTo(0);
  cv::fillConvexPoly(search_img,px_coordinates,CV_RGB(255,255,255));


  cv::imwrite("data/search_img.jpg", search_img);

  //#ifdef SHOW_SEARCH_IMAGE
  // cv::namedWindow("search_img",1);
  // cv::imshow("search_img", search_img);
  // cv::waitKey(10);
  //#endif


  // find largest rect in white area:

  // ratio = width/height
  bool finished = false;

  float step = 0.02; // check every X m

  float width, height; int x = 0; int y = 0;
  for (width = max_x; width > 0 && !finished; width -= step){
    height = width/ratio;

    // ROS_INFO("Width: %f, height: %f", width, height);

    // find fitting left upper corner (sliding window)
    for (x = 0; x < max_x-width && !finished; x+= step*100){
      for (y = 0; y < max_y-height; y+=step*100){
        // ROS_INFO("Checking x = %i, y = %i", x, y);

        int x_w = x+width; int y_w = y+height;
        assert(x >= 0 && y >= 0 && x_w < search_img.cols && y< search_img.rows);
        // check if all corners are withing white area:
        if (search_img.at<uchar>(y,x) == 0) continue;
        if (search_img.at<uchar>(y,x_w) == 0) continue;
        if (search_img.at<uchar>(y_w,x_w) == 0) continue;
        if (search_img.at<uchar>(y_w,x) == 0) continue;
        // ROS_INFO("Found fitting pose (w,h: %f %f)", width, height);
        //#ifdef SHOW_SEARCH_IMAGE
        cv::rectangle(search_img, cv::Point(x,y), cv::Point(x_w, y_w), CV_RGB(125,125,125));
        //#endif

        finished = true; // break outer loops
        break;
      } // for y
    } // for x
  } // for width

#ifdef SHOW_SEARCH_IMAGE
  cv::imshow("search_img", search_img);
  cv::waitKey(10);
#endif

  if (!finished)
    return false;


  cv::imwrite("data/search_img_final.jpg", search_img);

  // restore pose in wall_frame
  optimal_projection_area.width = width/100;
  optimal_projection_area.height = height/100;
  optimal_projection_area.x = x/100.0+min_x;
  optimal_projection_area.y = y/100.0+min_y;

  // show area on input image:

  ROS_INFO("Optimal rect: x,y: %f %f, w,h: %f %f", optimal_projection_area.x, optimal_projection_area.y, optimal_projection_area.width, optimal_projection_area.height);

  setupImageProjection(optimal_projection_area, cv::Size(img_px_size[1],img_px_size[0]));

  msg << "found optimal image region";

  return true;

}




bool Projector_Calibrator::findOptimalProjectionArea(float ratio, cv_RectF& rect,std::stringstream& msg){
  // #define SHOW_SEARCH_IMAGE


  if(!kinect_trafo_valid){
    ROS_WARN("findOptimalProjectionArea: no kinect trafo!"); return false;
  }

  if (detected_corners.size() == 0){
    ROS_WARN("findOptimalProjectionArea: no corners!"); return false;
  }

  if (cloud_moved.size() == 0){
    ROS_WARN("findOptimalProjectionArea: no input cloud!"); return false;
  }


  vector<cv::Point2i> c;
  getCheckerboardArea(c);

  // get 3d coordinates of corners in the wall-system
  vector<cv::Point2f> rotated;
  float min_x = 1e10;
  float min_y = 1e10;



  for (uint i=0; i<c.size(); ++i){
    // ROS_INFO("c: %i %i", c[i].x,c[i].y);
    if (!(c[i].x >= 0 && c[i].x < int(cloud_moved.width) && c[i].y>=0 && c[i].y < int(cloud_moved.height))){
      ROS_WARN("Checkerboard to close to image border! [activate #define SHOW_MASK_IMAGE]"); return false;
    }
    pcl_Point p = cloud_moved.at(c[i].x,c[i].y);

    //   pcl_Point q = input_cloud.at(c[i].x,c[i].y);
    //
    //   ROS_INFO("P: %f %f %f", p.x,p.y,p.z);
    //   ROS_INFO("Q: %f %f %f", q.x,q.y,q.z);


    if (!(p.x == p.x)) {ROS_WARN("Found NAN in input cloud, move camera a bit and rerun"); return false; }
    rotated.push_back(cv::Point2f(p.x,p.y));
    min_x = min(min_x, p.x);
    min_y = min(min_y, p.y);
    // ROS_INFO("pre: %f %f", rotated[i].x, rotated[i].y);
  }

  vector<cv::Point2i> pt_i;
  int max_x, max_y;
  max_x = max_y = -1;

  // ROS_INFO("min: %f %f", min_x, min_y);
  for (uint i=0; i<c.size(); ++i){

    rotated[i] = cv::Point2f((rotated[i].x-min_x)*100,(rotated[i].y-min_y)*100); // in cm <=> 1px
    pt_i.push_back(cv::Point2i(rotated[i].x,rotated[i].y));
    max_x = max(max_x, pt_i[i].x);
    max_y = max(max_y, pt_i[i].y);

  }


  cv::Mat search_img(max_y,max_x,CV_8UC1); search_img.setTo(0);
  cv::fillConvexPoly(search_img,pt_i,CV_RGB(255,255,255));


#ifdef SHOW_SEARCH_IMAGE
  cv::namedWindow("search_img",1);
  cv::imshow("search_img", search_img);
  cv::waitKey(10);
#endif


  // find largest rect in white area:

  // ratio = width/height
  bool finished = false;

  float step = 0.02; // check every X m

  float width, height;
  int x = 0; int y = 0;
  for (width = max_x; width > 0 && !finished; width -= step){ // check every 5 cm
    height = width/ratio;

    // ROS_INFO("Width: %f, height: %f", width, height);

    // find fitting left upper corner (sliding window)
    for (x = 0; x < max_x-width && !finished; x+= step*100){
      for (y = 0; y < max_y-height; y+=step*100){
        // ROS_INFO("Checking x = %i, y = %i", x, y);

        int x_w = x+width; int y_w = y+height;
        assert(x >= 0 && y >= 0 && x_w < search_img.cols && y< search_img.rows);
        // check if all corners are withing white area:
        if (search_img.at<uchar>(y,x) == 0) continue;
        if (search_img.at<uchar>(y,x_w) == 0) continue;
        if (search_img.at<uchar>(y_w,x_w) == 0) continue;
        if (search_img.at<uchar>(y_w,x) == 0) continue;
        // ROS_INFO("Found fitting pose (w,h: %f %f)", width, height);
#ifdef SHOW_SEARCH_IMAGE
        cv::rectangle(search_img, cv::Point(x,y), cv::Point(x_w, y_w), CV_RGB(125,125,125));
#endif

        finished = true; // break outer loops
        break;
      } // for y
    } // for x
  } // for width

#ifdef SHOW_SEARCH_IMAGE
  cv::imshow("search_img", search_img);
  cv::waitKey(10);
#endif

  if (!finished) return false;

  // restore pose in wall_frame
  rect.width = width/100;
  rect.height = height/100;

  rect.x = x/100.0+min_x;
  rect.y = y/100.0+min_y;

  // show area on input image:

  //  ROS_INFO("Optimal rect: x,y: %f %f, w,h: %f %f", rect.x, rect.y, rect.width, rect.height);

  return true;

}

void Projector_Calibrator::setInputCloud(const Cloud& cloud){
  // #define COMPUTE_NANS

  if (cloud.size() == 0){
    ROS_WARN("Tried to set new cloud to empty cloud!");
    return;
  }

  timing_start("copy");
  input_cloud = cloud;
  //timing_end("copy");

#ifdef COMPUTE_NANS
  // count invalid points:
  int input_nan = 0;

  for (uint i=0; i<cloud.size(); ++i) {
    pcl_Point p = cloud[i];
    if (!(p.x == p.x)) input_nan++;
  }

  int output_nan = 0;
#endif

  if (kinect_trafo_valid){
    //  ros::Time foo = ros::Time::now();
    //  timing_start("trafo");
    pcl::getTransformedPointCloud(input_cloud,kinect_trafo,cloud_moved);
    //  timing_end("trafo");
    //  ROS_INFO("transforming: %f ms", (ros::Time::now()-foo).toSec()*1000);

#ifdef COMPUTE_NANS
    for (uint i=0; i<cloud_moved.size(); ++i) {
      pcl_Point p = cloud_moved[i];
      if (!(p.x == p.x)) output_nan++;
    }
#endif

  }else{
    cloud_moved.clear();
  }


#ifdef COMPUTE_NANS
  ROS_INFO("NAN: input: %i, output: %i", input_nan, output_nan);
#endif


}



void Projector_Calibrator::setInputImage(cv::Mat& image, const cv::Mat* mask){
  assert(image.type() == CV_8UC3);
  if(mask){
    assert(mask->type() == CV_8UC1);
    mask->copyTo(this->checkerboard_search_mask);
  }
  image.copyTo(input_image);
}


/**
 * @brief Create mask of image that masks the area in which the pattern is visible
 * @see getCheckerboardArea, detected_corners
 */
void Projector_Calibrator::createMaskFromDetections(){
  //#define SHOW_MASK_IMAGE

  if (detected_corners.size() != uint(checkboard_size.width*checkboard_size.height)){
    ROS_INFO("can't create mask if the corners were not detected!"); return; }

  mask = cv::Mat(cv::Size(640,480), CV_8UC1);  mask.setTo(0);

  vector<cv::Point2i> c;
  getCheckerboardArea(c);

  cv::fillConvexPoly(mask,c,CV_RGB(255,255,255));

  // ROS_INFO("Writing kinect_mask to data/kinect_mask.png");
  cv::imwrite("data/kinect_mask.png", mask);


#ifdef SHOW_MASK_IMAGE
  cv::Mat cpy = input_image.clone();

  for (uint i=0; i<c.size(); ++i){
    cv::circle(cpy, c[i] ,10,CV_RGB(255,0,0));
    ROS_INFO("%i %i", c[i].x, c[i].y);
  }

  cv::namedWindow("Mask on Kinect Image");
  cv::imshow("Mask on Kinect Image", cpy);
  cv::waitKey(-1);

#endif
}


/**
 * @brief Fit plane to the given points using pcl::SampleConsensusModelPlane
 * @param cloud  points   points on which the model will be fitted
 * @param model  (output) resulting cloud
 * @return ration of inliers
 */
float Projector_Calibrator::fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model){
  // ROS_INFO("Fitting plane to cloud with %zu points", cloud.size());

  if (cloud.size() < 1000){
    ROS_WARN("fitPlaneToCloud: cloud size very small: %zu", cloud.size());
  }


  // http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
  pcl::SampleConsensusModelPlane<pcl_Point>::Ptr
      model_p (new pcl::SampleConsensusModelPlane<pcl_Point> (cloud.makeShared()));

  pcl::RandomSampleConsensus<pcl_Point> ransac(model_p);
  ransac.setDistanceThreshold(0.005); // max dist of 5mm
  ransac.computeModel();

  Eigen::VectorXf c;
  ransac.getModelCoefficients(c);
  model = c;

  std::vector<int> inliers;
  ransac.getInliers(inliers);
  float inlier_pct = inliers.size()*100.0/cloud.size();

  if (inlier_pct<0.5){ ROS_WARN("Only %.3f %%  inlier in fitPlaneToCloud!", inlier_pct); }
  return inlier_pct;
}





void Projector_Calibrator::applyMaskOnInputCloud(Cloud& out){
  if (!mask_valid()) createMaskFromDetections();
  assert(mask_valid() && int(input_cloud.width) == mask.cols);
  applyMaskOnCloud(mask, input_cloud, out);
}



/**
 * @brief Draw Checkerboard between the given points
 * @param l1  one corner of the pattern area
 * @param l2  other corner of the pattern area
 *
 * @see checkboard_size
 */
void Projector_Calibrator::projectSmallCheckerboard(cv::Point l1, cv::Point l2){
  drawCheckerboard(projector_image, l1,l2,
                   checkboard_size,
                   current_projector_corners);
}



/**
 * @brief Set projector to white or black image and remove detected corners
 * @param white if true, projector image will be white, otherwise black
 */
void Projector_Calibrator::projectUniformBackground(bool white){
  projector_image.setTo(white?255:0);
  current_projector_corners.clear(); // no corners on projector
  detected_corners.clear();           // and no detected corners anymore
}



void Projector_Calibrator::projectFullscreenCheckerboard(){
  drawCheckerboard(projector_image, cv::Point(0,0),
                   cv::Point(projector_image.cols, projector_image.rows),
                   checkboard_size,
                   current_projector_corners);
}


/**
 * @brief save Kinect Trafo to data/ $kinect_trafo_filename
 * @param msg
 * @return true if file was written
 */
bool Projector_Calibrator::saveKinectTrafo(std::stringstream& msg){

  if (!isKinectTrafoSet()){
    msg << "Can't save kinect trafo since it is not computed";
    return false;
  }


  char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
  if (saveAffineTrafo(kinect_trafo,fn))
    msg << "Wrote kinect_trafo to " <<  fn;
  else
    msg << "Problems writing kinect trafo to " <<  fn;

  return true;
}


/**
 * @brief kinect_trafo(2,3) += dz;
 * @param dz
 */
void Projector_Calibrator::translateKinectTrafo(float dz){
  assert(isKinectTrafoSet());
  kinect_trafo(2,3) += dz;
}


/**
 * @brief  apply yaw rotation matrix on kinect trafo
 * @param dyaw incremental yaw
 */
void Projector_Calibrator::rotateKinectTrafo(float dyaw){

  assert(isKinectTrafoSet());

  Eigen::Affine3f t = Eigen::Affine3f::Identity();

  t(0,0) = t(1,1) = cos(dyaw);
  t(0,1) = sin(dyaw);
  t(1,0) = -sin(dyaw);

  kinect_trafo = t*kinect_trafo;
}


/**
 * @brief Creation of RVIZ-Arrow Marker to show the estimated position of the projector using the DLT-Algorithm
 * @param marker
 */
void Projector_Calibrator::createProjektorMarker(visualization_msgs::Marker& marker){


  marker.header.frame_id = "/fixed_frame";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.ns = "ns_projector";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05; // shaft diameter
  marker.scale.y = 0.1;  // head diameter
  marker.scale.z = 0.05; // head length


  marker.color.a = 1;
  marker.color.r = 1;

  geometry_msgs::Point p;


  if (cal_cv_no_dist.projector_position.cols < 4){
    ROS_INFO("can't send marker if cal_cv_no_dist is not defined");
    return;
  }

  p.x = cal_cv_no_dist.projector_position.at<double>(0);
  p.y = cal_cv_no_dist.projector_position.at<double>(1);
  p.z = cal_cv_no_dist.projector_position.at<double>(2);

  marker.points.push_back(p);

  cv::Mat dir(3,1,CV_64FC1);
  dir.setTo(0);
  dir.at<double>(2) = 1;

  cv::Mat head = cal_cv_no_dist.rotMatrix*dir;

  p.x += head.at<double>(0);
  p.y += head.at<double>(1);
  p.z += head.at<double>(2);


  marker.points.push_back(p);

}

/**
 * @brief Creation of RVIZ-Arrow Marker to show the estimated position of the projector using the OpenCV-Approach
 * @param marker
 */
void Projector_Calibrator::createProjektorMarker_CV(visualization_msgs::Marker& marker){

  ROS_INFO("createProjektorMarker_CV");


  marker.header.frame_id = "/fixed_frame";
  marker.header.stamp = ros::Time::now();
  marker.id = 1;
  marker.ns = "ns_projector";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05; // shaft diameter
  marker.scale.y = 0.1;  // head diameter
  marker.scale.z = 0.05; // head length


  marker.color.a = 1;
  marker.color.b = 1;

  geometry_msgs::Point p;


  cv::Mat cam,trans, rot;
  cv::decomposeProjectionMatrix(cal_cv_no_dist.proj_matrix, cam,rot, trans);
  trans /= trans.at<double>(3);
  cout << "opencv rot: " << endl << rot << "trans " << trans << endl;

  p.x = trans.at<double>(0);
  p.y = trans.at<double>(1);
  p.z = trans.at<double>(2);

  //  p.x = projector_position_CV.at<double>(0);
  //  p.y = projector_position_CV.at<double>(1);
  //  p.z = projector_position_CV.at<double>(2);

  marker.points.push_back(p);

  cv::Mat dir(3,1,CV_64FC1);

  dir.setTo(0);
  dir.at<double>(2) = 1;

  cv::Mat head = rot*dir;
  //  cv::Mat head = rotMatrix_CV*dir;

  p.x += head.at<double>(0);
  p.y += head.at<double>(1);
  p.z += head.at<double>(2);


  marker.points.push_back(p);
}



bool Calibration::writeToFile(const std::string& filename)
{

  char fn[100]; sprintf(fn,"data/%s.yml", filename.c_str());
  ROS_INFO("Saving calibration to %s", fn);

  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  if (!fs.isOpened()){
    ROS_WARN("Could not write to %s", fn);
    return false;
  }

  fs << "projector_position" << projector_position;
  fs << "proj_matrix" << proj_matrix;
  fs << "rotMatrix" << rotMatrix;
  fs << "camera_matrix" << camera_matrix;
  fs << "distCoeffs" << distCoeffs;
  fs << "proj_trafo" << proj_trafo;
  fs << "rvec" << rvec;
  fs << "tvec" << tvec;

  return true;
}


bool Calibration::loadFromFile(const std::string& filename)
{

  char fn[100]; sprintf(fn,"data/%s.yml", filename.c_str());
  ROS_INFO("Reading Calibation from %s",fn);

  cv::FileStorage fs(fn, cv::FileStorage::READ);
  if (!fs.isOpened()){
    ROS_WARN("Could not read calibration %s", fn);
    return false;
  }

  fs["proj_matrix"] >> proj_matrix;
  fs["rotMatrix"] >> rotMatrix;
  fs["camera_matrix"] >> camera_matrix;
  fs["distCoeffs"] >> distCoeffs;
  fs["proj_trafo"] >> proj_trafo;
  fs["rvec"] >> rvec;
  fs["tvec"] >> tvec;
  fs["projector_position"] >> projector_position;


  fs.release();
  return true;

}

void Calibration::print(){
  cout << "rotMatrix  " << rotMatrix << endl;
  cout << "trans  " << projector_position << endl;
  cout << "Projection Matrix" << endl << proj_matrix << endl;
  cout << "camera_matrix" << endl << camera_matrix << endl;
  cout << "Proj Position: " << endl << projector_position << endl;
  cout << "Projector Rot: " << endl << rotMatrix << endl;
}


bool Calibration::projectPoints(std::vector<cv::Point3f> d3, std::vector<cv::Point2f>& px){
  px.clear();
  if (proj_matrix.cols != 4 || proj_matrix.rows != 3){
    return false;
  }
  checkCamMatrix();
  if (cv::sum(distCoeffs).val[0] < 1e-3){
    for (uint i=0; i<d3.size(); ++i){
      px.push_back(applyPerspectiveTrafo(d3[i],proj_matrix));
    }
  }else{
    cv::projectPoints(d3,rvec,tvec,camera_matrix,distCoeffs,px);
  }
  return true;
}

bool Calibration::projectPoint(pcl_Point d3, cv::Point2f& px){
  cv::Point3f d;
  d.x =d3.x; d.y =d3.y; d.z =d3.z;
  return projectPoint(d,px);
}


bool Calibration::projectPoint(cv::Point3f d3, cv::Point2f& px){

  if (proj_matrix.cols != 4 || proj_matrix.rows != 3){
    return false;
  }

  checkCamMatrix();

  if (cv::sum(distCoeffs).val[0] < 1e-3){
    px = applyPerspectiveTrafo(d3,proj_matrix);
  }else{
    vector<cv::Point3f> foo;
    foo.push_back(d3);

    vector<cv::Point2f> bar;
    cv::projectPoints(foo,rvec,tvec,camera_matrix,distCoeffs,bar);

    px = bar[0];

  }

  return true;
}
