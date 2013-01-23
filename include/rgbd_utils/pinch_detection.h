/*
 * pinch_detection.h
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#ifndef PINCH_DETECTION_H_
#define PINCH_DETECTION_H_


#include "rgbd_utils/cloud_gmm.h"


#define C_MIN_HAND_AREA 1000
#define C_MIN_OBJECT_AREA 100

// Grasp_Confirmed and Grasp_Finished are intermediate states
enum Tracking_State {Track_Initialized  = 0, Track_Confirmed, Track_Active, Track_Finished};

//struct Contour_info {
//  float area;
//  bool border_crossing;
//  cv::Point2f center;
//};


struct Tracked_Object {
  ros::Time last_seen;

  int id;
  Tracking_State state;
  uint not_seen_cnt;

  Tracked_Object(){
    state = Track_Initialized;
    not_seen_cnt = 0;
  }

  float dist_to(const Tracked_Object& other){
    assert(false);
    return -1;
  }


};



struct Playing_Piece : public Tracked_Object {

  Playing_Piece(){}

  float dist_to(const Playing_Piece& other){
    // TODO: return -1 of area of volume is too different
    return norm(sub(position,other.position));
  }

  float area;

  std::vector<cv::Point> mask;

  pcl_Point position;

  //      // copy image of detection:
  //      cv::Mat foo = cv::Mat::zeros( foreground.size(), CV_8UC3 );
  //      cv::drawContours(foo,contours,largest_hole_id,CV_RGB(255,255,255),-1);
  //      rgb.copyTo(detection.);


};

struct Grasp : public Tracked_Object {

  /// maximal distance (in m) between track and object for update
  static const float max_dist = 0.02;


  Grasp(){}

  float dist_to(const Grasp& other){
    float d = norm(sub(position,other.position));
    if (d>0.02) return -1;
    return d;
  }

  float area;
  std::vector<cv::Point> edge;
  pcl_Point position;


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
    assert(detections.size()>0);
    return detections[detections.size()-1];
  }



};




typedef std::map<int,Grasp>::iterator Grasp_it;
typedef std::map<int,Playing_Piece>::iterator Piece_it;

typedef std::map<int,Tracked_Object>::iterator Track_it;







template<class Trackable_, class Tracker_>
class Object_tracker {

  typedef typename std::map<int,Tracker_ >::iterator track_it;

private:
  float max_dist_;

  // tracks are published after so many observations and closed after so many frames without detection
  uint detection_hysteresis;

  static int next_id;

  /*
    T last_element(const std::vector<T>& objects){
        assert(objects.size()>0);
        return objects[objects.size()-1];
    }
    */

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
    if (max_dist <= 0)  max_dist = max_dist_;

    // remember which detection was used to update a track
    bool detection_used[detections.size()];
    for (uint i=0; i < detections.size(); ++i){detection_used[i] = false;}

    unsetUpdated(); // set updated-flag to zero (no track was updated yet)

    // ROS_INFO("matching %zu detections with %zu tracks", detections.size(), tracks.size());

    while (true){

      int best_track_id = -1;      // id of updated track
      int best_detection_pos = -1; // id of detection used to update
      float min_dist = max_dist;   // distance of best pair

      for (track_it it = tracks.begin(); it!=tracks.end(); ++it){

        if (it->second.updated_in_current_frame){ continue; }

        for (uint i=0; i < detections.size(); ++i){

          // ROS_INFO("i: %i",i);
          // ROS_INFO("clouds: %zu %zu",detections[i].corner_points_local.size(),detections[i].corner_points_kinect_frame.size() );

          // assert(detections[i].corner_points_local.size() == 8);
          // assert(detections[i].corner_points_kinect_frame.size() == 8);

          if (detection_used[i]){ continue; }

          float dist = it->second.last_detection().dist_to(detections[i]);

          // ROS_INFO("comparing track %i with obs %i: %f", it->first, i, dist);

          // detections[i] is an observation and has only one last_detection
          // cv::Point2f detection = detections[i].last_detection;

          // ROS_INFO("detection %i vs grasp %i: %f px", i,it->first,dist);

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

        // track_updated[best_track_id] = true;
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


      bool track_too_old = g->not_seen_cnt > detection_hysteresis;

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

      Tracker_ new_track; // (detections[i].last_detection);
      new_track.appendObservation(detections[i]);
      new_track.id = next_id++;
      new_track.state = Track_Initialized;

      tracks[new_track.id] = new_track;

      // assert(detections[i].corner_points_local.size() >= 8);
      // assert(detections[i].corner_points_kinect_frame.size() >= 8);

      // ROS_INFO("clouds: %zu %zu",detections[i].corner_points_local.size(),detections[i].corner_points_kinect_frame.size() );
      // ROS_INFO("creating new track with id %i for obs %i", new_track.id,i);
    }

  } // update_tracks




  Object_tracker(){
    max_dist_ = 30;
    detection_hysteresis = 5;
  }

};

//template <class T>
//        int Object_tracker<T>::next_id = 0;

template<class Trackable_, class Tracker_>
int Object_tracker<Trackable_, Tracker_>::next_id = 0;


// typedef Object_tracker<Grasp> Grasp_Tracker;
// typedef Object_tracker<Playing_Piece> Piece_Tracker;




pcl_Point getCenter(const std::vector<cv::Point>& pts, const Cloud& cloud);




class Detector  {


  static const float small_area_threshold = 100;
  static const float large_area_threshold = 1500;


  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<float> areas;
  std::vector<bool> intersects;

  cv::Mat detection_area_edge;
  cv::Mat detection_area;


  bool detection_area_set;

  bool intersectsDetectionAreaBorder(const std::vector<std::vector<cv::Point> > contours, int i);

  bool handVisible;

  cv::Mat foreground_; // copy of foreground image
  cv::Mat dummy; // helper image 8UC1

public:

  void showDetectionAreaEdge(cv::Mat& img);

  std::vector<Grasp> grasp_detections;
  std::vector<Playing_Piece> object_detections;

  Detector(){
    detection_area_set = false;
    handVisible = false;
  }


  void newFrame(const cv::Mat& foreground);

  void analyseScene(cv::Mat* rgb_result = NULL);
  void detectPointingGesture(const cv::Mat& dists, cv::Mat* rgb_result = NULL);

  void setDetectionArea(const cv::Mat& mask);

  bool handVisibleInLastFrame(){return handVisible;}
  bool detectionAreaSet(){return detection_area_set;}

};



#endif /* PINCH_DETECTION_H_ */
