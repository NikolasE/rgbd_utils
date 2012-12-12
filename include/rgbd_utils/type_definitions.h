/*
 * cloud_processing.h
 *
 *  Created on: Feb 28, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CLOUD_PROCESSING_H_
#define CLOUD_PROCESSING_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.h>


typedef pcl::PointXYZRGB pcl_Point;
typedef pcl::PointCloud<pcl_Point> Cloud;

typedef pcl::Normal pcl_Normal;
typedef pcl::PointCloud<pcl_Normal> Cloud_n;


#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

//static timeval _RMSTUNE_starts[16];
//static timeval _RMSTUNE_ends[16];
//static long _RMSTUNE_accums[16];

typedef std::pair<timeval,timeval> Timer_;

static std::map<std::string, Timer_> clocks;


static void timing_start(const std::string& name ){
  timeval time;
  gettimeofday(&time,NULL);
  clocks[name] = std::make_pair<timeval,timeval>(time,time);
}



static long msBetween(const std::string& name){
  if (clocks.find(name) == clocks.end()){
    std::cerr << "no started timer with name " << name << std::endl;
    return -1;
  }
  Timer_ timer = clocks[name];

  long seconds  = timer.second.tv_sec  - timer.first.tv_sec;
  long useconds = timer.second.tv_usec - timer.first.tv_usec;
  return  ((seconds) * 1000 + useconds/1000.0) + 0.5;

}


static void timing_print(const std::string& name){
 if (clocks.find(name) == clocks.end()){
   std::cerr << "no started timer with name " << name << std::endl;
   return;
 }

 std::cout << "Timer " << name << " : " << msBetween(name) << " ms" << std::endl;

}





static bool timing_end(const std::string& name, bool quiet = false){

  timeval time;
  gettimeofday(&time,NULL);

  if (clocks.find(name) == clocks.end()){
    std::cerr << "no started timer with name " << name << std::endl;
    return false;
  }

  Timer_ timer = clocks[name];
  timer.second = time;
  clocks[name] = timer;

  if (!quiet)
   timing_print(name);


  return true;
}












#endif /* CLOUD_PROCESSING_H_ */
