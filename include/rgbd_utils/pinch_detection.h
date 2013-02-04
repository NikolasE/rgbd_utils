/*
 * pinch_detection.h
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#ifndef PINCH_DETECTION_H_
#define PINCH_DETECTION_H_


#include "rgbd_utils/gaussian_model.h"


#define C_MIN_OBJECT_AREA 100

// Grasp_Confirmed and Grasp_Finished are intermediate states
// and correspond to button down and button up event.
// Track_active is like mouse_move (moving with pressed button)
enum Tracking_State {Track_Initialized  = 0, Track_Confirmed, Track_Active, Track_Finished};

enum Trackable_Type {TT_DEFAULT, TT_Object, TT_GRASP,TT_FINGERTIP};


struct Tracked_Object {
  ros::Time last_seen;

  int id;
  Tracking_State state;
  uint not_seen_cnt;

  Tracked_Object(){
    state = Track_Initialized;
    not_seen_cnt = 0;
  }

  float dist_to(const Tracked_Object& other, Tracking_State* state = NULL){
    assert(false);
    return -1;
  }

  std::vector<cv::Point> mask;
  pcl_Point position_world;
  pcl_Point position_kinect;
  cv::Point2f pixel_position;


  bool getAngle_PCA_2D(float& angle);
  bool getAngle_PCA_3D(const Cloud& cloud,float& angle, Eigen::Affine3f* trafo = NULL);

};


struct Fingertip : public Tracked_Object {
  Fingertip(){}

  float dist_to(const Fingertip& other,Tracking_State* state = NULL){
    return norm(sub(position_world,other.position_world));
  }

};



struct Playing_Piece : public Tracked_Object {

  Playing_Piece(){}

  float dist_to(const Playing_Piece& other,Tracking_State* state = NULL){

    double dist = norm(sub(position_world,other.position_world));

    if (state && *state != Track_Active && dist > 0.05)
      return -1;

    return dist;
  }

  float area;


  std::vector<cv::Point> contour;

  //      // copy image of detection:
  //      cv::Mat foo = cv::Mat::zeros( foreground.size(), CV_8UC3 );
  //      cv::drawContours(foo,contours,largest_hole_id,CV_RGB(255,255,255),-1);
  //      rgb.copyTo(detection.);

};

struct Grasp : public Tracked_Object {

  /// maximal distance (in m) between track and object for update
  static const float max_dist = 1e6;//0.02;


  Grasp(){}

  float dist_to(const Grasp& other,Tracking_State* state = NULL){
    float d = norm(sub(position_world,other.position_world));
    if (d>max_dist) return -1;
    return d;
  }

  float area;


};



template<class Trackable>
struct Track {

  ros::Time last_seen;

  int id;
  Tracking_State state;
  uint not_seen_cnt;

  std::vector<Trackable> detections;
  bool updated_in_current_frame;

  Track(){
    state = Track_Initialized;
    not_seen_cnt = 0;
    updated_in_current_frame = false;
  }

  void appendObservation(const Trackable & other){
    detections.push_back(other);
  }


  Trackable last_detection(){
    // todo: return pointer only (or const reference)
    assert(detections.size()>0);
    return detections[detections.size()-1];
  }

  void visualizeOnImage(cv::Mat& img, cv::Scalar color, Trackable_Type type = TT_DEFAULT){
    uint n = detections.size();

    if (n==0) return;

    float l = 5;

    if (type == TT_FINGERTIP){
      cv::Point p = detections[0].pixel_position;
      cv::rectangle(img,cv::Point(p.x-l,p.y-l),cv::Point(p.x+l,p.y+l),color,2);
      p = detections[n-1].pixel_position;
      cv::rectangle(img,cv::Point(p.x-l,p.y-l),cv::Point(p.x+l,p.y+l),color,2);
    }else{
      cv::circle(img,detections[0].pixel_position,2*l,color,2);
      cv::circle(img,detections[n-1].pixel_position,2*l,color,2);
    }

    for (uint i=0; i<n-1; ++i){
      cv::line(img,detections[i].pixel_position,detections[i+1].pixel_position,color,3);
    }

  }


  void visualize_on_surface(const cv::Mat P, cv::Mat& img,cv::Scalar color){

    uint n = detections.size();

    if (n==0) return;

    float l = 5;

    Trackable obs = detections[detections.size()-1];


    //  if (type == TT_FINGERTIP){
    cv::Point2f px = applyPerspectiveTrafo(obs.position_kinect,P);
    cv::circle(img,px,30,color,-1);
    //  }


    //    cv::rectangle(img,cv::Point(p.x-l,p.y-l),cv::Point(p.x+l,p.y+l),color,2);
    //    p = detections[n-1].pixel_position;
    //    cv::rectangle(img,cv::Point(p.x-l,p.y-l),cv::Point(p.x+l,p.y+l),color,2);
    //  }else{
    //    cv::circle(img,detections[0].pixel_position,2*l,color,2);
    //    cv::circle(img,detections[n-1].pixel_position,2*l,color,2);
    //  }

    //  for (uint i=0; i<n-1; ++i){
    //    cv::line(img,detections[i].pixel_position,detections[i+1].pixel_position,color,3);
    //  }

  }


};









typedef std::map<int,Track<Grasp> >::iterator GraspTrack_it;
typedef std::map<int,Track<Playing_Piece> >::iterator PieceTrack_it;
typedef std::map<int,Track<Fingertip> >::iterator FingerTrack_it;



template<class Trackable_, class Tracker_>
class Object_tracker {

  typedef typename std::map<int,Tracker_ >::iterator track_it;

private:

  // tracks are published after so many observations
  uint detection_hysteresis;

  // track is closed after so many frames without detection
  uint deletion_hysteresis;

  static int next_id;

  void unsetUpdated(){
    for (track_it it = tracks.begin(); it != tracks.end(); ++it)
      it->second.updated_in_current_frame = false;
  }

public:


  std::map<int,Tracker_> tracks;

  /**
 *
 * @param detections  object detections in the current frame
 * @param max_dist    maximal distance (in px) of a matched pair
 */
  /// match tracks with detections by finding iteratively the closest pair
  void update_tracks(const std::vector<Trackable_>& detections, float max_dist = -1){


    //    ROS_INFO("Updating tracks with %zu observations", detections.size());

    // remember which detection was used to update a track
    bool detection_used[detections.size()];
    for (uint i=0; i < detections.size(); ++i){detection_used[i] = false;}

    unsetUpdated(); // set updated-flag to zero (no track was updated yet)

    while (true){

      int best_track_id = -1;      // id of updated track
      int best_detection_pos = -1; // id of detection used to update
      float min_dist = max_dist>0?max_dist:1e6;   // distance of best pair

      for (track_it it = tracks.begin(); it!=tracks.end(); ++it){

        if (it->second.updated_in_current_frame){ continue; }

        for (uint i=0; i < detections.size(); ++i){

          if (detection_used[i]){ continue; }

          // ROS_INFO("track %i has %zu detections",it->first,it->second.detections.size());

          float dist = it->second.last_detection().dist_to(detections[i]);

          // ROS_INFO("comparing track %i with obs %i: %f", it->first, i, dist);

          // if track and observation are too different (size, color,...) dist_to returns -1
          if (dist < 0)
            continue;


          if (dist < min_dist){
            min_dist = dist;
            best_track_id = it->first;
            best_detection_pos = i;
          }

        } // iteration over detections

      } // iteration over tracks


      // check if a pair with distance < max_dist was found
      if (best_detection_pos >= 0){

        // update best pair
        tracks[best_track_id].appendObservation(detections[best_detection_pos]);
        tracks[best_track_id].updated_in_current_frame = true;

        detection_used[best_detection_pos] = true;

      }else{
        // no match was found. Unmatched detections will spawn new tracks
        break;
      }

    } // while true


    // mark tracks that have not been updated
    for (track_it it = tracks.begin(); it!=tracks.end(); ++it){
      if (! it->second.updated_in_current_frame)
        it->second.not_seen_cnt++;
    }

    // update state of tracks
    track_it it = tracks.begin();
    while( it != tracks.end()){

      Tracker_ *g = &it->second;
      //    ROS_INFO("track %i: state %i", g->id,g->state);

      if (g->state == Track_Confirmed){
        //   ROS_INFO("track %i: now active", g->id);
        g->state = Track_Active;
      }

      /// track was seen in several frames and is now an official track
      if (g->detections.size() == detection_hysteresis && g->state == Track_Initialized){
        //   ROS_INFO("New track at %f %f", g->last_detection.x ,g->last_detection.y);
        g->state = Track_Confirmed;
        //   ROS_INFO("track %i: now confirmed", g->id);
      }


      bool track_too_old = g->not_seen_cnt > deletion_hysteresis;

      // remove grasp if it was finished in the last update
      // http://stackoverflow.com/questions/263945/what-happens-if-you-call-erase-on-a-map-element-while-iterating-from-begin-to
      if (g->state == Track_Finished || (track_too_old && g->state == Track_Initialized)){
        tracks.erase(it++);
      }else{

        if (track_too_old)
          g->state = Track_Finished;

        ++it;
      }

    }


    for (uint i=0; i<detections.size(); ++i){
      if (detection_used[i])
        continue;

      Tracker_ new_track;
      new_track.appendObservation(detections[i]);
      new_track.id = next_id++;
      new_track.state = Track_Initialized;

      tracks[new_track.id] = new_track;

    }


    for (track_it it = tracks.begin(); it!=tracks.end(); ++it){
      assert(it->second.detections.size()>0);
    }



  } // update_tracks




  Object_tracker(){
    detection_hysteresis = 10;
    deletion_hysteresis = 10;
  }

};


template<class Trackable_, class Tracker_>
int Object_tracker<Trackable_, Tracker_>::next_id = 0;


void drawObjectContours(Object_tracker<Playing_Piece,Track<Playing_Piece> >& piece_tracker, cv::Mat& img);




class Detector  {


  static const float small_area_threshold = 100;

  // static const float fingertip_min_dist = 0.02;
  static const float fingertip_max_dist = 0.06;
  static const float fingertip_min_size = 30;


  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<float> areas;
  std::vector<bool> intersects;
  std::vector<bool> is_grasp;

  //  std::vector<int> hand_contours;
  cv::Mat detection_area_edge;
  cv::Mat detection_area;


  bool detection_area_set;

  bool intersectsDetectionAreaBorder(const std::vector<std::vector<cv::Point> > contours, int i);

  bool handVisible;

  cv::Mat masked_edge,hand_edge,cpy;

  cv::Mat foreground_; // copy of foreground image
  cv::Mat norm_;
  cv::Mat last_static_norm;
  cv::Mat dummy, dummy_2; // helper image 8UC1
  cv::Mat diff, dist_thres;

  void getFingerTips(cv::Mat* rgb = NULL);
  void getFingerTips_2(cv::Mat* rgb = NULL);
  void getObjects(cv::Mat* rgb_result);
  void getGrasps(cv::Mat* rgb_result);

  Cloud* cloud_;
  Eigen::Affine3f kinect_trafo_;
  bool trafo_set;


public:

  void setTransformation(Eigen::Affine3f kinect_trafo){
    kinect_trafo_ = kinect_trafo;
    trafo_set = true;
  }


  void showDetectionAreaEdge(cv::Mat& img);

  std::vector<Grasp> grasp_detections;
  std::vector<Playing_Piece> object_detections;
  std::vector<Fingertip> finger_detections;

  Detector(){
    detection_area_set = false;
    handVisible = false;
    trafo_set = false;
  }



  void newFrame(const cv::Mat& foreground, const cv::Mat& norm, Cloud* cloud = NULL);

  void analyseScene(cv::Mat* rgb_result = NULL);
  //  void detectPointingGesture(const cv::Mat& dists, cv::Mat* rgb_result = NULL);

  void setDetectionArea(const cv::Mat& mask);

  bool handVisibleInLastFrame(){return handVisible;}
  bool detectionAreaSet(){return detection_area_set;}

};



#endif /* PINCH_DETECTION_H_ */
