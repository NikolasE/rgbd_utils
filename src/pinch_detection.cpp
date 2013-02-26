/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "rgbd_utils/pinch_detection.h"

using namespace std;
using namespace cv;



bool Tracked_Object::getAngle_PCA_3D(const Cloud& cloud, float& angle, Eigen::Affine3f* trafo){
  if(mask.size()==0) return false;

  angle = -1e6;

  cv::Mat data(mask.size(),3,CV_32FC1);
  // fill data matrix:
  int pos = 0;
  for (uint i=0; i<mask.size(); ++i){

    pcl_Point p = cloud.at(mask[i].x,mask[i].y);
    if (p.x != p.x)
      continue;
    data.at<float>(pos,0) = p.x;
    data.at<float>(pos,1) = p.y;
    data.at<float>(pos,2) = p.z;

    pos++;
  }


  cv::PCA pca(data.rowRange(0,pos),cv::Mat(),CV_PCA_DATA_AS_ROW);

  cv::Mat mean = pca.mean;
  // ROS_INFO("Mean: %f %f %f", mean.at<float>(0),mean.at<float>(1),mean.at<float>(2));


  if (trafo){

    Eigen::Vector3f mean_e;
    mean_e.x() = mean.at<float>(0);
    mean_e.y() = mean.at<float>(1);
    mean_e.z() = mean.at<float>(2);

    //  cv::Mat ev_x = pca.eigenvectors.row(0);
    cv::Mat ev_y = pca.eigenvectors.row(1);
    cv::Mat ev_z = pca.eigenvectors.row(2);

    Eigen::Vector3f vec_z;
    vec_z.x() = -ev_z.at<float>(0);
    vec_z.y() = -ev_z.at<float>(1);
    vec_z.z() = -ev_z.at<float>(2);
    //    vec_z.x() = 0;
    //    vec_z.y() = 0;
    //    vec_z.z() = 1;

    Eigen::Vector3f vec_y;
    //    vec_y.x() = 0;
    //    vec_y.y() = 1;
    //    vec_y.z() = 0;
    vec_y.x() = ev_y.at<float>(0);
    vec_y.y() = ev_y.at<float>(1);
    vec_y.z() = ev_y.at<float>(2);


    //    cout << "m " << mean_e << endl;
    //    cout << "y " << vec_y << endl;
    //    cout << "z " << vec_z << endl;

    pcl::getTransformationFromTwoUnitVectorsAndOrigin(vec_y, vec_z,mean_e,*trafo);
    //    printTrafo(*trafo);



    Eigen::Affine3f inv = pcl::getInverse(*trafo);
    sendTrafo("/openni_rgb_optical_frame","/hand_frame",inv);


  }


  //assert(pca.eigenvectors.type() == CV_64SC1);

  //  cv::Mat ev_0 = pca.eigenvectors.row(0);
  //  cv::Mat ev_1 = pca.eigenvectors.row(1);




  return true;
}

bool Tracked_Object::getAngle_PCA_2D(float& angle){
  if(mask.size()==0) return false;

  angle = -1;

  cv::Mat data(mask.size(),2,CV_32FC1);
  // fill data matrix:
  for (uint i=0; i<mask.size(); ++i){
    data.at<float>(i,0) = mask[i].x;
    data.at<float>(i,1) = mask[i].y;
  }


  cv::PCA pca(data,cv::Mat(),CV_PCA_DATA_AS_ROW);

  // cv::Mat mean = pca.mean;
  //  ROS_INFO("Mean: %f %f", mean.at<float>(0),mean.at<float>(1));


  //assert(pca.eigenvectors.type() == CV_64SC1);

  cv::Mat ev_0 = pca.eigenvectors.row(0);
  // cv::Mat ev_1 = pca.eigenvectors.row(1);

  //  ROS_INFO("Ev 0: (val: %f) %f %f", pca.eigenvalues.at<float>(0), ev_0.at<float>(0),ev_0.at<float>(1));
  //  ROS_INFO("Ev 1: (val: %f) %f %f", pca.eigenvalues.at<float>(1), ev_1.at<float>(0),ev_1.at<float>(1));


  cv::RotatedRect rrect = cv::minAreaRect(mask);
  float angle_rect = rrect.angle;
  if (angle_rect < 0) angle_rect+=180;

  angle = atan2(ev_0.at<float>(1),ev_0.at<float>(0))/M_PI*180-90;

  if (angle < -30)
    angle += 180;

  // cout << angle << endl;

  //  ROS_INFO("pca: %f, rect: %f", angle, angle_rect);

  return true;
}

void drawObjectContours(Object_tracker<Playing_Piece,Track<Playing_Piece> >& piece_tracker, cv::Mat& img){
  assert(img.type() == CV_8UC1);
  img.setTo(0);

  std::vector<std::vector<cv::Point> > contours;

  // create vector of observations (needed interface for cv::drawContours)
  for (PieceTrack_it it = piece_tracker.tracks.begin(); it != piece_tracker.tracks.end(); ++it){
    contours.push_back(it->second.last_detection()->contour);
  }

  for (uint i=0; i<contours.size(); ++i)
    cv::drawContours(img,contours,i,CV_RGB(255,255,255),-1);

}

void Detector::setDetectionArea(const cv::Mat& mask){

  detection_area_set = true;

  mask.copyTo(detection_area);

  dummy = cv::Mat(detection_area.rows,detection_area.cols,CV_8UC1);


  // compute border of area (used to detect objects entering the area (aka hands)
  cv::Canny(detection_area, detection_area_edge, 1,10);
  //cv::dilate(detection_area_edge, detection_area_edge, cv::Mat(), cv::Point(-1,-1),1);
}

bool Detector::intersectsDetectionAreaBorder_withHeight(const std::vector<std::vector<cv::Point> > contours, int i, const cv::Mat& dist){
  assert(detection_area_set);

  // cv::Mat foo = cv::Mat::zeros(detection_area_edge.size(), CV_8UC1);

  ensureSizeAndType(hand_test,detection_area_edge.cols,detection_area_edge.rows,CV_8UC1);
  hand_test.setTo(0);

  cv::drawContours(hand_test, contours,i, CV_RGB(255,255,255),2);

  //  double m,x;
  //  cv::minMaxLoc(dist,&m,&x);
  //  ROS_INFO("diff image 2: %f %f", m,x);

  // if the contour is closer than 10cm to the surface, it's definitely no hand (but rather some moved sand)
  ensureSizeAndType(dist_hand,dist);
  dist_hand.setTo(0);

  dist.copyTo(dist_hand, hand_test);
  double min_,max_;
  cv::minMaxLoc(dist_hand, &min_, &max_);

  //  ROS_INFO("Height %i: %f %f",i,min_, max_);

  if (max_ < 0.1)
    return false;

  ensureSizeAndType(intersection,hand_test);
  intersection.setTo(0);
  hand_test.copyTo(intersection,detection_area_edge);

  return cv::countNonZero(intersection)>0;
}

void Detector::newFrame(const cv::Mat& foreground, const cv::Mat& dists, Cloud* cloud){
  if (!detection_area_set){
    ROS_WARN("detector: no detection area!");
    return;
  }

  areas.clear();
  contours.clear();

  hierarchy.clear();
  intersects.clear();

  //  cv::imwrite("data/fg.png", foreground);
  //  cv::imwrite("data/fg_mask.png", detection_area);


  foreground.copyTo(foreground_,detection_area);
  cv::findContours(foreground_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  foreground.copyTo(foreground_,detection_area); // findcontours changes source image

  dists.copyTo(diff);

  cloud_ = cloud;

  handVisible = false;

  //  double m,x;
  //  cv::minMaxLoc(dists,&m,&x);
  //  ROS_INFO("diff: %f %f", m,x);

  //  float area_sum = 0;

  for ( uint i = 0; i< contours.size(); i++ ){
    float area = cv::contourArea(contours[i]);
    areas.push_back(area);


    //    area_sum += area;

    //    ROS_INFO("area: %i %f",i,area);
    if (area <= 50){
      intersects.push_back(false);
      continue;
    }

    bool inter = intersectsDetectionAreaBorder_withHeight(contours,i, dists);

    intersects.push_back(inter);
    handVisible = handVisible || inter;
  }

  //  ROS_INFO("areasun: %f", area_sum);

  assert(areas.size() == contours.size());

}

void Detector::showDetectionAreaEdge(cv::Mat& img){
  cv::Mat red(img.rows, img.cols,CV_8UC3,CV_RGB(255,0,0));
  red.copyTo(img,detection_area_edge);
}

/*
void Detector::getFingerTips_2(cv::Mat* rgb){


  cv::Mat diff = last_static_norm-norm_;

  cv::Mat dist_thres;

  // remove everything below 2cm
  cv::threshold(diff,dist_thres,fingertip_max_dist,-1,CV_THRESH_TOZERO_INV);
  cv::threshold(dist_thres,dist_thres,0,1,CV_THRESH_BINARY);

  cv::namedWindow("2");


  dummy.setTo(0);
  for (uint i=0; i<contours.size(); ++i){
    if (intersects[i])
      cv::drawContours(dummy, contours,i, CV_RGB(255,255,255),-1);
  }

  cv::erode(dummy,dummy,cv::Mat(), cv::Point(-1,-1),1);

  cv::Mat fg = cv::Mat(480,640,CV_8UC1);
  fg.setTo(0);
  dist_thres.copyTo(fg, dummy);

  cv::imshow("2",fg);

  cv::namedWindow("hand");
  cv::imshow("hand",dummy);


  std::vector<std::vector<cv::Point> > contours_finger;
  std::vector<cv::Vec4i> hierarchy_finger;

  cv::Mat cpy;
  fg.convertTo(cpy, CV_8UC1);
  cv::findContours(cpy, contours_finger, hierarchy_finger, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


  int N = int(contours_finger.size());
  for (int i=0; i<N; ++i){


    cv::Moments mom = cv::moments(contours_finger[i]);

    if (rgb){
      cv::Point2f px = cv::Point2f(mom.m10/mom.m00,mom.m01/mom.m00);
      cv::circle(*rgb,px,5,CV_RGB(0,255,0),1);
    }
  }


}
*/

void Detector::getFingerTips(cv::Mat* rgb){

  bool verbose = false;

  if (verbose){
    cv::namedWindow("dist_thres");
    cv::namedWindow("hand_egde");
    cv::namedWindow("and");
  }


  ensureSizeAndType(dist_thres,foreground_.cols,foreground_.rows, CV_32FC1);
  diff.copyTo(dist_thres,foreground_);

  // remove everything above 4cm
  cv::threshold(diff,dist_thres,fingertip_max_dist,-1,CV_THRESH_TOZERO_INV);

  // everything non zeros gets one
  cv::threshold(dist_thres,dist_thres,0,1,CV_THRESH_BINARY);

  cv::medianBlur(dist_thres,dist_thres,3);
  if (verbose) cv::imshow("dist_thres",dist_thres);

  std::vector<std::vector<cv::Point> > contours_finger;
  std::vector<cv::Vec4i> hierarchy_finger;

  for (uint i=0; i<contours.size(); ++i){
    if (!intersects[i] || is_grasp[i])
      continue;

    contours_finger.clear();
    hierarchy_finger.clear();

    ensureSizeAndType(hand_edge,foreground_.cols,foreground_.rows, CV_8UC1);
    hand_edge.setTo(0);
    cv::drawContours(hand_edge, contours,i, CV_RGB(255,255,255),-1);
    //    cv::Canny(hand_edge, hand_edge, 1,10);
    //    cv::dilate(hand_edge,hand_edge,cv::Mat());

    if (verbose) cv::imshow("hand_egde",hand_edge);

    masked_edge.setTo(0);
    dist_thres.copyTo(masked_edge,hand_edge);
    if (verbose){
      cv::imshow("and",masked_edge);
      //cv::waitKey(100);
    }


    masked_edge.convertTo(cpy, CV_8UC1);
    assert(cpy.type() == CV_8UC1);
    assert(cpy.cols > 0);
    cv::findContours(cpy, contours_finger, hierarchy_finger, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int N = int(contours_finger.size());
    if (verbose)  ROS_INFO("Found %i possible fingertip contours", N);

    if (N == 0)
      return;

    int largest = -1;
    float max_area = fingertip_min_size;

    for (int i=0; i<N; ++i){
      float area = cv::contourArea(contours_finger[i]);
      if (area >= max_area){
        max_area = area; largest = i;
      }
    }

    if (largest < 0)
      continue;

    // ROS_INFO("Max size: %f", max_area);

    // new fingertip detected!
    Fingertip tip;

    cv::Moments mom = cv::moments(contours_finger[largest]);

    tip.pixel_position = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);

    dummy.setTo(0);
    cv::drawContours(dummy,contours_finger,largest,CV_RGB(255,255,255),-1);

    for (int x=0; x<dummy.cols; ++x)
      for (int y=0; y< dummy.rows; ++y){
        if (dummy.at<uchar>(y,x)>0){
          tip.mask.push_back(cv::Point(x,y));
        }
      }


    if (rgb){
      cv::circle(*rgb,tip.pixel_position,3,CV_RGB(255,255,0),3);
    }

    if (cloud_){
      tip.position_kinect = getCenter(tip.mask,cloud_);
      if (trafo_set){
        ROS_INFO("computing position in world frame");
        tip.position_world = getTransformedPoint(tip.position_kinect,kinect_trafo_);

        ROS_INFO("Kinect: %f %f %f world: %f %f %f",tip.position_kinect.x,tip.position_kinect.y,tip.position_kinect.z,
                 tip.position_world.x,tip.position_world.y,tip.position_world.z);
      }
    }

    finger_detections.push_back(tip);

  } // loop over all contours

}


void Detector::getObjects(cv::Mat* rgb_result){


  for( int current = 0; current<int(contours.size()); current++){
    float area = areas[current];

    if (hierarchy[current][3] != -1)
      continue;

    if (area < 10)
      continue;

    Playing_Piece piece;
    piece.area = area;
    cv::Moments mom = cv::moments(contours[current]);

    piece.pixel_position = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);


    piece.contour = contours[current];

    // store pixel position corresponding to the object
    dummy.setTo(0);
    cv::drawContours(dummy,contours,current,CV_RGB(255,255,255),-1);
    for (int x=0; x<dummy.cols; ++x)
      for (int y=0; y< dummy.rows; ++y){
        if (dummy.at<uchar>(y,x)>0){
          piece.mask.push_back(cv::Point(x,y));
        }
      }


    if (cloud_){
      piece.position_kinect = getCenter(piece.mask,cloud_);
      if (trafo_set){
        piece.position_world = getTransformedPoint(piece.position_kinect,kinect_trafo_);
      }
    }

    object_detections.push_back(piece);

    if (rgb_result)
      cv::drawContours(*rgb_result,contours,current,CV_RGB(255,0,0),2);

  }
}



void Detector::getGrasps(cv::Mat* rgb_result){


  is_grasp.clear();

  for( int current = 0; current<int(contours.size()); current++) {

    if (!intersects[current]){
      is_grasp.push_back(false);
      continue;
    }


    // look for largest hole in this contour:
    int largest_hole_id = -1;
    float largest_hole_area = small_area_threshold;

    for (uint j=0; j<contours.size(); ++j){
      if (hierarchy[j][3] != current)  continue;

      if (intersects[j]) continue;

      float hole_area = areas[j];

      if (hole_area > largest_hole_area){
        largest_hole_area = hole_area;
        largest_hole_id = j;
      }

    }


    // no hole with sufficient size was found
    if (largest_hole_id < 0) {
      is_grasp.push_back(false);
      continue;
    }

    ROS_INFO("max Grasp area: %f (min %f)",largest_hole_area,small_area_threshold);


    is_grasp.push_back(true);

    Grasp grasp;

    grasp.area = largest_hole_area;
    cv::Moments mom = cv::moments(contours[largest_hole_id]);
    grasp.pixel_position = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);

    // get points on hand around the hole:
    dummy.setTo(0);

    cv::drawContours(dummy,contours,largest_hole_id,CV_RGB(255,255,255),2);

    // cv::dilate(dummy,dummy,cv::Mat(), cv::Point(-1,-1),2);
    cv::Mat edge_image;
    dummy.copyTo(edge_image,foreground_);

    for (int x=0; x<edge_image.cols; ++x)
      for (int y=0; y< edge_image.rows; ++y){
        // ROS_INFO("%i %i",x,y);
        if (edge_image.at<uchar>(y,x)>0){
          if (rgb_result) rgb_result->at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,255);
          grasp.mask.push_back(cv::Point(x,y));
        }
      }

    if (cloud_){
      grasp.position_kinect = getCenter(grasp.mask,cloud_);
      if (trafo_set){
        grasp.position_world = getTransformedPoint(grasp.position_kinect,kinect_trafo_);
      }
    }
    grasp_detections.push_back(grasp);


    if (rgb_result){
      // show hand:
      cv::drawContours(*rgb_result,contours,current,CV_RGB(0,0,255),2);

      // and hole:
      // cv::drawContours(*rgb_result,contours,largest_hole_id,CV_RGB(0,255,0),2);
    }
  }

  assert(is_grasp.size() == intersects.size());
}


void Detector::analyseScene(cv::Mat* rgb_result){

  if (!detection_area_set){
    ROS_WARN("detector: no detection area!");
    return;
  }

  //  ROS_INFO("areas: %zu, contours: %zu", areas.size(), contours.size());
  assert(areas.size() == contours.size());

  finger_detections.clear();
  grasp_detections.clear();
  object_detections.clear();

  if (handVisible){
    getGrasps(rgb_result);
    getFingerTips(rgb_result);
  }else{
    getObjects(rgb_result);
  }

  ROS_INFO("Found %zu objects", object_detections.size());

}




//void findConvexityDefects(const vector<Point>& contour, const vector<int>& hull, vector<CvConvexityDefect>& convexDefects){
//  if(!(hull.size() > 0 && contour.size() > 0))
//    return;

//  CvSeq* contourPoints;
//  CvSeq* defects;
//  CvMemStorage* storage;
//  CvMemStorage* strDefects;
//  CvMemStorage* contourStr;
//  CvConvexityDefect *defectArray = 0;

//  strDefects = cvCreateMemStorage();
//  defects = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvSeq),sizeof(CvPoint), strDefects );

//  //We transform our vector<Point> into a CvSeq* object of CvPoint.
//  contourStr = cvCreateMemStorage();
//  contourPoints = cvCreateSeq(CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), contourStr);
//  for(int i=0; i<(int)contour.size(); i++) {
//    CvPoint cp = {contour[i].x,  contour[i].y};
//    cvSeqPush(contourPoints, &cp);
//  }

//  //Now, we do the same thing with the hull index
//  int count = (int)hull.size();
//  //int hullK[count];
//  int* hullK = (int*)malloc(count*sizeof(int));
//  for(int i=0; i<count; i++){hullK[i] = hull.at(i);}
//  CvMat hullMat = cvMat(1, count, CV_32SC1, hullK);

//  //We calculate convexity defects
//  storage = cvCreateMemStorage(0);
//  defects = cvConvexityDefects(contourPoints, &hullMat, storage);
//  defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*defects->total);
//  cvCvtSeqToArray(defects, defectArray, CV_WHOLE_SEQ);
//  //printf("DefectArray %i %i\n",defectArray->end->x, defectArray->end->y);

//  //We store defects points in the convexDefects parameter.
//  for(int i = 0; i<defects->total; i++){
//    convexDefects.push_back(defectArray[i]);
//  }

//  //We release memory
//  //  cvReleaseMemStorage(contourStr);
//  //  cvReleaseMemStorage(strDefects);
//  //  cvReleaseMemStorage(storage);

//}


//bool goodDefect(const CvConvexityDefect& defect){
//  return dist(*defect.start,*defect.end) > 30;
//}



//void Detector::detectPointingGesture(const cv::Mat& dists, cv::Mat* rgb_result){

//  vector<vector<cv::Point> > hull( contours.size() );
//  vector<vector<int> > hullsI(contours.size());
//  vector<vector<cv::Vec4i> > convdefect(contours.size());

//  vector<CvConvexityDefect> defects;

//  for( uint i = 0; i < contours.sizROS_INFO("XXXXXXXXXXXXXX imageCallback rgb");e(); i++ )
//    {
//      cv::convexHull( cv::Mat(contours[i]), hull[i], false);
//      cv::convexHull( cv::Mat(contours[i]), hullsI[i], false);

//      findConvexityDefects(contours[i],hullsI[i],defects);


//      if (defects.size() < 2) continue;

//      // compute minimal distance between defect points

//      int id1 = -1;
//      int id2 = -1;
//      float min_dist = 20;
//      float dist_;

//      for (uint d=0; d<defects.size()-1; ++d){
//        cv::Point p1 = *defects[d].depth_point;

//        if (!goodDefect(defects[d])) continue;

//        if (detection_area_edge.at<uchar>(p1.y,p1.x) >0)
//          continue;


//        for (uint d2=d+1; d2<defects.size(); ++d2){

//          cv::Point p2 = *defects[d2].depth_point;
//          if (detection_area_edge.at<uchar>(p2.y,p2.x) >0)
//            continue;


//          if (!goodDefect(defects[d2])) continue;


//          dist_ = dist(p1,p2);

//          if (dist_ < min_dist){
//            min_dist = dist_;
//            id1 = d;
//            id2 = d2;
//          }
//        }
//      }

//      if (id1>=0){
//        ROS_INFO("min dist: %f (%i %i)",min_dist,id1,id2);
//        cv::circle(*rgb_result,*defects[id1].depth_point,5,CV_RGB(0,255,255),3);
//        cv::circle(*rgb_result,*defects[id2].depth_point,5,CV_RGB(0,255,255),3);

//        cv::line(*rgb_result,*defects[id1].depth_point,*defects[id2].depth_point,CV_RGB(0,255,255),2);

//        if (id2-id1 < int(contours.size()/2))
//          cv::circle(*rgb_result,mean(*defects[id1].end,*defects[id2].start),5,CV_RGB(0,255,0),2);
//        else
//          cv::circle(*rgb_result,mean(*defects[id1].start,*defects[id2].end),5,CV_RGB(0,255,0),2);


//        //cv::circle(*rgb_result,*defects[id1].end,3,CV_RGB(0,255,0),2);
//        // cv::circle(*rgb_result,*defects[id1].start,3,CV_RGB(0,255,0),2);

//        //cv::circle(*rgb_result,*defects[id2].end,3,CV_RGB(0,0,255),2);
//        // cv::circle(*rgb_result,*defects[id2].start,3,CV_RGB(0,0,255),2);
//      }


//      //      for (uint d=0; d<defects.size(); ++d){
//      //        //        cv::circle(*rgb_result,*defects[d].start,5,CV_RGB(0,0,255),2);
//      //        //        cv::circle(*rgb_result,*defects[d].end,3,CV_RGB(0,255,0),2);
//      //        // cv::circle(*rgb_result,*defects[d].depth_point,5,CV_RGB(255,0,0),1);
//      //      }

//      //      if(hullsI[i].size() > 3 )
//      //        cvConvexityDefects(contours[i],hullsI[i],convdefect[i])
//      //cv::convexityDefects();
//      //            cv::convex



//    }


//  //  return;


//  //  // todo: improve running time :)
//  //  // for each point on the contour, compute distance to closest point in convex hull:

//  //  for (int c=0; c<contours.size(); ++c){ // for each contour

//  //    int N = contours[c].size();

//  //    if (N < 50)
//  //      continue;


//  //    float dist2Contour[N];

//  //    for (int p=0; p<N; ++p){ // for each contour point
//  //      cv::Point c_p = contours[c][p];
//  //      int best_fit = -1;
//  //      float min_dist = 0;

//  //      for (int h=0; h<hull[c].size(); ++h){ // for each point on contour
//  //        cv::Point h_p = hull[c][h];

//  //        float d = sqrt(pow(c_p.x-h_p.x,2)+pow(c_p.y-h_p.y,2));

//  //        if (d<min_dist || best_fit == -1){
//  //          min_dist = d;
//  //          best_fit = h;
//  //        }

//  //      }

//  //      dist2Contour[p] = min_dist;

//  //    }


//  //    // print distances:
//  //    ROS_INFO("distances for contour %i",c);
//  //    for (int i=0; i<N; ++i){
//  //      ROS_INFO("%f",dist2Contour[i]);
//  //    }




//  //  }






//  /// Draw contours + hull results
//  // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
//  if (rgb_result){
//    for( uint i = 0; i< contours.size(); i++ )
//      {
//        // size_t count = contours[i].size();

//        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
//        cv::drawContours( *rgb_result, contours, i, cv::Scalar(255,0,0), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
//        cv::drawContours( *rgb_result, hull, i, cv::Scalar(0,0,255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
//      }
//  }



//}



//float getAreaOfTriangle(const pcl_Point& a, const pcl_Point& b, const pcl_Point& c){

//  if (a.x!=a.x) return 0;
//  if (b.x!=b.x) return 0;
//  if (c.x!=c.x) return 0;

//  pcl_Point u = sub(b,a);
//  pcl_Point v = sub(c,a);


//  // ROS_INFO("A: %f %f %f", a.x,a.y,a.z);
//  // ROS_INFO("B: %f %f %f", b.x,b.y,b.z);
//  // ROS_INFO("C: %f %f %f", c.x,c.y,c.z);


//  // compute cross product:
//  pcl_Point cr;
//  cr.x = u.y*v.z-u.z*v.y;
//  cr.y = u.z*v.x-u.x*v.z;
//  cr.z = u.x*v.y-u.y*v.x;

//  // ROS_INFO("Are/a: %f", norm(cr)/2);

//  return norm(cr)/2;
//}

///**
//*
//* @param object
//* @param cloud
//* @return
//*/
///// get size of object in cm^2
//float getArea(const cv::Mat& object, const Cloud& cloud){

//  assert(object.type() == CV_8UC1);

//  // check every triangle in the image and

//  // todo (?) erode object to get rid of measurements at the edge of objects
//  // errors could otherwise dominate the area of the object

//  float total_area = 0;

//  // code not perfect: if only top right corner is nan, both triangles will have no area
//  // TODO:
//  for (int x=0; x<object.cols-1; ++x)
//    for (int y=0; y<object.rows-1; ++y){

//      // check if the pixels on the corner of the rectangle with (x,y) as top left corner are on the object
//      bool tl = object.at<uchar>(y,x) > 0;
//      bool tr = object.at<uchar>(y,x+1) > 0;
//      bool ll = object.at<uchar>(y+1,x) > 0;
//      bool lr = object.at<uchar>(y+1,x+1) > 0;

//      pcl_Point tl_p = cloud.at(x,y);
//      pcl_Point tr_p = cloud.at(x+1,y);
//      pcl_Point ll_p = cloud.at(x,y+1);
//      pcl_Point lr_p = cloud.at(x+1,y+1);

//      // check upper left triangle:
//      if (tl && tr && ll){
//        total_area += getAreaOfTriangle(tl_p, tr_p, ll_p); // returns 0 if one of the points contains NaN
//      }

//      // check lower left triangle:
//      if (tr && ll && lr){
//        total_area += getAreaOfTriangle(tr_p, ll_p, lr_p); // returns 0 if one of the points contains NaN
//      }

//    }

//  return total_area;

//}

