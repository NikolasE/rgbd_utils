/*
 * exp_map.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: engelhan
 */


#include "rgbd_utils/surface_modeler.h"

using namespace std;


#ifdef WITH_LIBGEOMETRY

void Surface_Modeler::writeExpMapCSV(){
  // std::string filename("expmap1.txt");
  // expMapMeshObject.ExportCSV_Edges(filename.c_str());

  rms::ExpMapGenerator  *gen = expMapMeshObject.GetExpMapGen();
  rms::VFTriangleMesh &mesh = expMapMeshObject.GetMesh();

  // was machen diese vier Zeilen?
  Wml::Vector3f vVertex, vNormal;
  mesh.GetVertex(1, vVertex, &vNormal);
  rms::Frame3f vInitFrame(vVertex, vNormal);
  gen->SetSurfaceDistances( vInitFrame, 0.0f, 1 );

  std::vector<unsigned int> vIdx;
  std::vector<float> vU, vV;
  rms::VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
  //unsigned int nExpmaps = mesh.GetVertexCount();
  //float fAvgNbrs = 0;

  int foo = 0;
  while ( curv != endv ) {
    _RMSTUNE_start(2);
    rms::IMesh::VertexID vID = *curv;  curv++;
    mesh.GetVertex(vID, vVertex, &vNormal);
    rms::Frame3f vSeedFrame(vVertex, vNormal);


    //vVertex.X() = 0;


    gen->SetSurfaceDistances( vVertex, 0.0f, 0.05, &vSeedFrame);


    //       switch ( eMode ) {
    //         case MaxK:
    //           expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, nMaxK);
    //           break;
    //         case MaxGeoDist:
    //           expmapgen.SetSurfaceDistances( vVertex, 0.0f, fMaxGeoDist, &vSeedFrame);
    //           break;
    //         case Hybrid:
    //           expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, fMaxGeoDist, nMaxK);
    //           break;
    //       }


    vIdx.resize(0); vU.resize(0); vV.resize(0);
    gen->GetVertexUVs(vIdx, vU, vV);
    _RMSTUNE_end(2);
    _RMSTUNE_accum(2);
    size_t nCount = vIdx.size();

    ROS_INFO("B cnt: %i, nCount=%zu", foo++, nCount);

    UV_Patch patch;
    patch.center_id = vID; // again zero-based! (TODO: VERIFY)

    //patch.neighbours.resize(nCount);
    for ( unsigned int k = 0; k < nCount; ++k ) {
      //        output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << " " << sqrt(vU[k]*vU[k]+vV[k]*vV[k]) << std::endl;
      //        output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;
      // cout << (vID+1) << " " << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;

      // patch.neighbours[k] = std::make_pair(vIdx[k],cv::Point2f(vU[k],vV[k]));

    }

    //expMaps[patch.center_id] = patch;

    //fAvgNbrs += (float)nCount / (float)nExpmaps;
  }

  ROS_INFO("Computed %zu patches", expMaps.size());


  //     float fTotalTime = _RMSTUNE_accum_time(2);
  //     float EPS = nExpmaps / fTotalTime;
  //     std::cerr << "[expmapCL] computed " << nExpmaps << " expmaps in "  << fTotalTime << "s" << std::endl;
  //     std::cerr << "    average nbr count: " << fAvgNbrs << std::endl;
  //     std::cerr << "    expmaps per second " << EPS << std::endl;

}

bool Surface_Modeler::initExpMapGenerator(){

  ros::Time start = ros::Time::now();

  // HACK: write and read to file
  //  saveAsObj("temp.obj", false); // add version without normals
  //  expMapMeshObject.ReadMeshOBJ("temp.obj");

  rms::VFTriangleMesh mesh_;
  copyToMesh(mesh_);


  expMapMeshObject.SetMesh(mesh_);
  expMapMeshObject.GetExpMapGen()->SetUseSquareCulling(true);

  //  rms::ExpMapGenerator *gen = expMapMeshObject.GetExpMapGen();
  //  rms::VFTriangleMesh &mesh = expMapMeshObject.GetMesh();
  //  printf("read mesh with %i vertices and %i triangles. max id: %i \n", mesh.GetVertexCount(), mesh.GetTriangleCount(), mesh.GetMaxVertexID());

  // ros::Time start_init = ros::Time::now();
  //UV_Patch p = getPatchAround(0,0,0.0001);
  //ROS_INFO("first expmap: %f ms", (ros::Time::now()-start_init).toSec()*1000.0);

  //  start_init = ros::Time::now();
  // p = getPatchAround(0,0,0.05);
  //  ROS_INFO("second expmap: %f ms", (ros::Time::now()-start_init).toSec()*1000.0);

  ROS_INFO("Moving mesh to expMap: %f ms", (ros::Time::now()-start).toSec()*1000.0);

  return true;
}


/**
  * @param mask   vertices with uv-coordinates have value 255, 0 otherwise (CV_8UC1)
  * @param uv_map if not zero, point contains uv-coordinates of corresponding vertex (CV_32FC2)
  * @param min_dist if non-NULL, min_dist will contain minimal pixel distance of point without uv-coords to center of patch
  */
/// representation of vertices with uv-coordinates
void Surface_Modeler::visualizePatch(const UV_Patch& patch, cv::Mat& mask, cv::Mat& uv_map){

  if (mask.cols != cell_cnt_x || mask.rows != cell_cnt_y || mask.type() != CV_8UC1){
    mask =  cv::Mat(cell_cnt_y,cell_cnt_x, CV_8UC1);
  }

  if (uv_map.cols != cell_cnt_x || uv_map.rows != cell_cnt_y || uv_map.type() != CV_32FC2){
    uv_map =  cv::Mat(cell_cnt_y,cell_cnt_x, CV_32FC2);
  }

  mask.setTo(0);
  uv_map.setTo(std::numeric_limits<float>::max());

  cv::Point p;
  cv::Point2f uv;

  cv::Point center = expMapId2gridPos(patch.center_id);
  // ROS_INFO("center: %i %i", center.x, center.y);

  for (uint i=0; i<patch.neighbours.size();++i){
    p = expMapId2gridPos(patch.neighbours[i].first);
    uv = patch.neighbours[i].second;
    mask.at<uchar>(p.y,p.x) = 255;
    uv_map.at<cv::Vec2f>(p.y,p.x) = cv::Vec2f(uv.x,uv.y);
  }


}


void Surface_Modeler::visualizePatch_col(const UV_Patch& patch, cv::Mat& img){

  if (img.cols != cell_cnt_x || img.rows != cell_cnt_y || img.type() != CV_8UC3){
    img =  cv::Mat(cell_cnt_y,cell_cnt_x, CV_8UC3);
  }

  img.setTo(0);


  cv::Point p;
  cv::Point2f uv;
  for (uint i=0; i<patch.neighbours.size();++i){
    p = expMapId2gridPos(patch.neighbours[i].first);
    uv = patch.neighbours[i].second;
    // printf("u,v: %f %f \n", uv.x,uv.y);

    float l = sqrt(uv.dot(uv));

    int c = int(ceil(l*100))%3;

    cv::Vec3b col;

    if (c==0) col = cv::Vec3b(0,255,0);
    if (c==1) col = cv::Vec3b(0,255,255);
    if (c==1) col = cv::Vec3b(255,255,0);

    img.at<cv::Vec3b>(p.y,p.x) = col;
  }

  //mark center
  //  p = expMapId2gridPos(patch.center_id);
  //  cv::circle(img, p,2,CV_RGB(255,0,0),-1);



}

void Surface_Modeler::expMapTestRun(){

  //  for (int x= model_3d.width-1; x>=0; --x)
  //   for (int y =0 ; y<  model_3d.height; ++y){
  //     UV_Patch patch =  getPatchAround(cv::Point(x,y));
  //   }

  UV_Patch patch =  getPatchAround(cv::Point(0,0));

}

UV_Patch Surface_Modeler::getPatchAround(cv::Point grid_pos, float dist){

  // ROS_INFO("computing patch around %i %i", grid_pos.x, grid_pos.y);

  UV_Patch patch;

  //ros::Time time_a = ros::Time::now();
  if (!(grid_pos.x >=0 && grid_pos.x < cell_cnt_x) || !(grid_pos.y >=0 && grid_pos.y < cell_cnt_y)){
    return patch;
  }

  int vID = gridPos2expMapid(grid_pos);

  rms::ExpMapGenerator *gen = expMapMeshObject.GetExpMapGen();
  rms::VFTriangleMesh &mesh = expMapMeshObject.GetMesh();

  Wml::Vector3f vVertex, vNormal;
  mesh.GetVertex(vID, vVertex, &vNormal);
  rms::Frame3f vSeedFrame(vVertex, vNormal);
  //ROS_INFO("patch a: %f ms", (ros::Time::now()-time_a).toSec()*1000.0);

  std::vector<unsigned int> vIdx;
  std::vector<float> vU, vV;

  //  rms::Frame3f vNearestFrame(vSeedFrame);
  //  assert(expMapMeshObject.FindNearestFrame( vSeedFrame.Origin(), vNearestFrame ));
  //  if ( (vSeedFrame.Origin() - vNearestFrame.Origin()).Length() > 0.001f )
  //    vSeedFrame = vNearestFrame;




  //ros::Time time_b = ros::Time::now();
  gen->SetSurfaceDistances( vSeedFrame.Origin(), 0.0f, dist, &vSeedFrame);
  //ROS_INFO("Surface computation: %f ms", (ros::Time::now()-time_b).toSec()*1000.0);


  gen->GetVertexUVs(vIdx, vU, vV);

  //ros::Time time_c = ros::Time::now();

  patch.center_id = vID; // id in zero-based coordinates
  uint nCount = vIdx.size();
  patch.neighbours.resize(nCount);
  for ( unsigned int k = 0; k < nCount; ++k ) {
    patch.neighbours[k] = std::make_pair(vIdx[k],cv::Point2f(vU[k],vV[k]));
  }

  //ROS_INFO("patch c: %f ms", (ros::Time::now()-time_c).toSec()*1000.0);
  return patch;
}


#endif
