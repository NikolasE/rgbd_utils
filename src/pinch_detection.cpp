/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "rgbd_utils/pinch_detection.h"

using namespace std;
using namespace cv;





float getAreaOfTriangle(const pcl_Point& a, const pcl_Point& b, const pcl_Point& c){

  if (a.x!=a.x) return 0;
  if (b.x!=b.x) return 0;
  if (c.x!=c.x) return 0;

  pcl_Point u = sub(b,a);
  pcl_Point v = sub(c,a);


  // ROS_INFO("A: %f %f %f", a.x,a.y,a.z);
  // ROS_INFO("B: %f %f %f", b.x,b.y,b.z);
  // ROS_INFO("C: %f %f %f", c.x,c.y,c.z);


  // compute cross product:
  pcl_Point cr;
  cr.x = u.y*v.z-u.z*v.y;
  cr.y = u.z*v.x-u.x*v.z;
  cr.z = u.x*v.y-u.y*v.x;

  // ROS_INFO("Are/a: %f", norm(cr)/2);

  return norm(cr)/2;
}

/**
*
* @param object
* @param cloud
* @return
*/
/// get size of object in cm^2
float getArea(const cv::Mat& object, const Cloud& cloud){

  assert(object.type() == CV_8UC1);

  // check every triangle in the image and

  // todo (?) erode object to get rid of measurements at the edge of objects
  // errors could otherwise dominate the area of the object

  float total_area = 0;

  // code not perfect: if only top right corner is nan, both triangles will have no area
  // TODO:
  for (int x=0; x<object.cols-1; ++x)
    for (int y=0; y<object.rows-1; ++y){

      // check if the pixels on the corner of the rectangle with (x,y) as top left corner are on the object
      bool tl = object.at<uchar>(y,x) > 0;
      bool tr = object.at<uchar>(y,x+1) > 0;
      bool ll = object.at<uchar>(y+1,x) > 0;
      bool lr = object.at<uchar>(y+1,x+1) > 0;

      pcl_Point tl_p = cloud.at(x,y);
      pcl_Point tr_p = cloud.at(x+1,y);
      pcl_Point ll_p = cloud.at(x,y+1);
      pcl_Point lr_p = cloud.at(x+1,y+1);

      // check upper left triangle:
      if (tl && tr && ll){
        total_area += getAreaOfTriangle(tl_p, tr_p, ll_p); // returns 0 if one of the points contains NaN
      }

      // check lower left triangle:
      if (tr && ll && lr){
        total_area += getAreaOfTriangle(tr_p, ll_p, lr_p); // returns 0 if one of the points contains NaN
      }

    }

  return total_area;

}






void Detector::setDetectionArea(const cv::Mat& mask){

  detection_area_set = true;

  mask.copyTo(detection_area);

  dummy = cv::Mat(detection_area.rows,detection_area.cols,CV_8UC1);


  // compute border of area (used to detect objects entering the area (aka hands)
  cv::Canny(detection_area, detection_area_edge, 1,10);
  //cv::dilate(detection_area_edge, detection_area_edge, cv::Mat(), cv::Point(-1,-1),1);
}



bool Detector::intersectsDetectionAreaBorder(const std::vector<std::vector<cv::Point> > contours, int i){
  assert(detection_area_set);

  cv::Mat foo = cv::Mat::zeros(detection_area_edge.size(), CV_8UC1);

  cv::drawContours(foo, contours,i, CV_RGB(255,255,255),2);

  cv::Mat intersection;
  foo.copyTo(intersection,detection_area_edge);

  return cv::countNonZero(intersection)>0;
}



void Detector::newFrame(const cv::Mat& foreground){
  assert(detection_area_set);

  grasp_detections.clear();
  contours.clear();
  hierarchy.clear();
  areas.clear();
  intersects.clear();

  foreground.copyTo(foreground_,detection_area);
  cv::findContours(foreground_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  foreground.copyTo(foreground_,detection_area); // findcontours changes source image


  handVisible = false;
  for( uint i = 0; i< contours.size(); i++ ){
    areas.push_back(cv::contourArea(contours[i]));
    bool inter = intersectsDetectionAreaBorder(contours,i);
    intersects.push_back(inter);
    handVisible = handVisible || inter;
  }

}


void findConvexityDefects(const vector<Point>& contour, const vector<int>& hull, vector<CvConvexityDefect>& convexDefects){
  if(!(hull.size() > 0 && contour.size() > 0))
    return;

  CvSeq* contourPoints;
  CvSeq* defects;
  CvMemStorage* storage;
  CvMemStorage* strDefects;
  CvMemStorage* contourStr;
  CvConvexityDefect *defectArray = 0;

  strDefects = cvCreateMemStorage();
  defects = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvSeq),sizeof(CvPoint), strDefects );

  //We transform our vector<Point> into a CvSeq* object of CvPoint.
  contourStr = cvCreateMemStorage();
  contourPoints = cvCreateSeq(CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), contourStr);
  for(int i=0; i<(int)contour.size(); i++) {
    CvPoint cp = {contour[i].x,  contour[i].y};
    cvSeqPush(contourPoints, &cp);
  }

  //Now, we do the same thing with the hull index
  int count = (int)hull.size();
  //int hullK[count];
  int* hullK = (int*)malloc(count*sizeof(int));
  for(int i=0; i<count; i++){hullK[i] = hull.at(i);}
  CvMat hullMat = cvMat(1, count, CV_32SC1, hullK);

  //We calculate convexity defects
  storage = cvCreateMemStorage(0);
  defects = cvConvexityDefects(contourPoints, &hullMat, storage);
  defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*defects->total);
  cvCvtSeqToArray(defects, defectArray, CV_WHOLE_SEQ);
  //printf("DefectArray %i %i\n",defectArray->end->x, defectArray->end->y);

  //We store defects points in the convexDefects parameter.
  for(int i = 0; i<defects->total; i++){
    convexDefects.push_back(defectArray[i]);
  }

  //We release memory
  //  cvReleaseMemStorage(contourStr);
  //  cvReleaseMemStorage(strDefects);
  //  cvReleaseMemStorage(storage);

}


bool goodDefect(const CvConvexityDefect& defect){
  return dist(*defect.start,*defect.end) > 30;
}



void Detector::detectPointingGesture(const cv::Mat& dists, cv::Mat* rgb_result){

  vector<vector<cv::Point> > hull( contours.size() );
  vector<vector<int> > hullsI(contours.size());
  vector<vector<cv::Vec4i> > convdefect(contours.size());

  vector<CvConvexityDefect> defects;

  for( uint i = 0; i < contours.size(); i++ )
    {
      cv::convexHull( cv::Mat(contours[i]), hull[i], false);
      cv::convexHull( cv::Mat(contours[i]), hullsI[i], false);

      findConvexityDefects(contours[i],hullsI[i],defects);


      if (defects.size() < 2) continue;

      // compute minimal distance between defect points

      int id1 = -1;
      int id2 = -1;
      float min_dist = 20;
      float dist_;

      for (uint d=0; d<defects.size()-1; ++d){
        cv::Point p1 = *defects[d].depth_point;

        if (!goodDefect(defects[d])) continue;

        if (detection_area_edge.at<uchar>(p1.y,p1.x) >0)
          continue;


        for (uint d2=d+1; d2<defects.size(); ++d2){

          cv::Point p2 = *defects[d2].depth_point;
          if (detection_area_edge.at<uchar>(p2.y,p2.x) >0)
            continue;


          if (!goodDefect(defects[d2])) continue;


          dist_ = dist(p1,p2);

          if (dist_ < min_dist){
            min_dist = dist_;
            id1 = d;
            id2 = d2;
          }
        }
      }

      if (id1>=0){
        ROS_INFO("min dist: %f (%i %i)",min_dist,id1,id2);
        cv::circle(*rgb_result,*defects[id1].depth_point,5,CV_RGB(0,255,255),3);
        cv::circle(*rgb_result,*defects[id2].depth_point,5,CV_RGB(0,255,255),3);

        cv::line(*rgb_result,*defects[id1].depth_point,*defects[id2].depth_point,CV_RGB(0,255,255),2);

        if (id2-id1 < int(contours.size()/2))
          cv::circle(*rgb_result,mean(*defects[id1].end,*defects[id2].start),5,CV_RGB(0,255,0),2);
        else
          cv::circle(*rgb_result,mean(*defects[id1].start,*defects[id2].end),5,CV_RGB(0,255,0),2);


        //cv::circle(*rgb_result,*defects[id1].end,3,CV_RGB(0,255,0),2);
        // cv::circle(*rgb_result,*defects[id1].start,3,CV_RGB(0,255,0),2);

        //cv::circle(*rgb_result,*defects[id2].end,3,CV_RGB(0,0,255),2);
        // cv::circle(*rgb_result,*defects[id2].start,3,CV_RGB(0,0,255),2);
      }


      //      for (uint d=0; d<defects.size(); ++d){
      //        //        cv::circle(*rgb_result,*defects[d].start,5,CV_RGB(0,0,255),2);
      //        //        cv::circle(*rgb_result,*defects[d].end,3,CV_RGB(0,255,0),2);
      //        // cv::circle(*rgb_result,*defects[d].depth_point,5,CV_RGB(255,0,0),1);
      //      }

      //      if(hullsI[i].size() > 3 )
      //        cvConvexityDefects(contours[i],hullsI[i],convdefect[i])
      //cv::convexityDefects();
      //            cv::convex



    }


  //  return;


  //  // todo: improve running time :)
  //  // for each point on the contour, compute distance to closest point in convex hull:

  //  for (int c=0; c<contours.size(); ++c){ // for each contour

  //    int N = contours[c].size();

  //    if (N < 50)
  //      continue;


  //    float dist2Contour[N];

  //    for (int p=0; p<N; ++p){ // for each contour point
  //      cv::Point c_p = contours[c][p];
  //      int best_fit = -1;
  //      float min_dist = 0;

  //      for (int h=0; h<hull[c].size(); ++h){ // for each point on contour
  //        cv::Point h_p = hull[c][h];

  //        float d = sqrt(pow(c_p.x-h_p.x,2)+pow(c_p.y-h_p.y,2));

  //        if (d<min_dist || best_fit == -1){
  //          min_dist = d;
  //          best_fit = h;
  //        }

  //      }

  //      dist2Contour[p] = min_dist;

  //    }


  //    // print distances:
  //    ROS_INFO("distances for contour %i",c);
  //    for (int i=0; i<N; ++i){
  //      ROS_INFO("%f",dist2Contour[i]);
  //    }




  //  }






  /// Draw contours + hull results
  // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  if (rgb_result){
    for( uint i = 0; i< contours.size(); i++ )
      {
        // size_t count = contours[i].size();

        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        cv::drawContours( *rgb_result, contours, i, cv::Scalar(255,0,0), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
        cv::drawContours( *rgb_result, hull, i, cv::Scalar(0,0,255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
      }
  }



}


void Detector::showDetectionAreaEdge(cv::Mat& img){
  cv::Mat red(img.rows, img.cols,CV_8UC3,CV_RGB(255,0,0));
  red.copyTo(img,detection_area_edge);
}


void Detector::analyseScene(cv::Mat* rgb_result){

  assert(areas.size() == contours.size());

  grasp_detections.clear();
  object_detections.clear();


  for( int current = 0; current<int(contours.size()); current++)
    {

      float area = areas[current];

      if (!intersects[current] && hierarchy[current][3] == -1){ // top level object and no intersection

        if (area < 10)
          continue;

        Playing_Piece piece;
        piece.area = area;
        cv::Moments mom = cv::moments(contours[current]);
        piece.position = pcl_Point(mom.m10/mom.m00, mom.m01/mom.m00,-1);



        // store pixel position corresponding to the object
        dummy.setTo(0);
        cv::drawContours(dummy,contours,current,CV_RGB(255,255,255),2);
        for (int x=0; x<dummy.cols; ++x)
          for (int y=0; y< dummy.rows; ++y){
            if (dummy.at<uchar>(y,x)>0){
              piece.mask.push_back(cv::Point(x,y));
            }
          }

        object_detections.push_back(piece);

        if (rgb_result)
          cv::drawContours(*rgb_result,contours,current,CV_RGB(255,0,0),2);

        continue;
      }



      if (area < large_area_threshold){
        // ROS_INFO("found small intersecting contour (%.0f / %i)", area,large_area_threshold);
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
      if (largest_hole_id < 0) continue;

      Grasp detection;

      detection.area = largest_hole_area;
      cv::Moments mom = cv::moments(contours[largest_hole_id]);
      detection.position = pcl_Point(mom.m10/mom.m00, mom.m01/mom.m00,-1);


      // get points on hand around the hole:
      dummy.setTo(0);

      cv::drawContours(dummy,contours,largest_hole_id,CV_RGB(255,255,255),2);

      cv::dilate(dummy,dummy,cv::Mat(), cv::Point(-1,-1),3);
      cv::Mat edge_image;
      dummy.copyTo(edge_image,foreground_);

      for (int x=0; x<edge_image.cols; ++x)
        for (int y=0; y< edge_image.rows; ++y){
          // ROS_INFO("%i %i",x,y);
          if (edge_image.at<uchar>(y,x)>0){
            rgb_result->at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,255);
            detection.edge.push_back(cv::Point(x,y));
          }
        }

      grasp_detections.push_back(detection);


      if (rgb_result){
        // show hand:
        cv::drawContours(*rgb_result,contours,current,CV_RGB(0,0,255),2);

        // and hole:
        cv::drawContours(*rgb_result,contours,largest_hole_id,CV_RGB(0,255,0),2);
      }
    }

}


pcl_Point getCenter(const std::vector<cv::Point>& pts, const Cloud& cloud){
  pcl_Point mean(0,0,0);

  int valid_cnt = 0;

  for (uint i=0; i<pts.size(); ++i){
    pcl_Point P = cloud.at(pts[i].x,pts[i].y);

    if (P.x == P.x){
      valid_cnt++;
      add(mean,P);

      if (P.z < 0 || P.z > 10)
        ROS_INFO("pt %i: %f %f %f",i,P.x,P.y,P.z);

    }
  }

  div(mean,valid_cnt);

  return mean;

}




