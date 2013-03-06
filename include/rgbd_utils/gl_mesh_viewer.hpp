#ifndef GL_MESH_VIEWER_H
#define GL_MESH_VIEWER_H

#include <GL/glew.h>
#include <QtOpenGL>
#include <QtGui/QMainWindow>
#include <QApplication>

#include "rgbd_utils/type_definitions.h"
#include "rgbd_utils/meshing.h"
#include "rgbd_utils/path_planning.h"
#include "rgbd_utils/pinch_detection.h"
#include "rgbd_utils/projector_calibrator.h"



#include <QGLWidget>

class GL_Mesh_Viewer : public QGLWidget
{
  Q_OBJECT

private:

  int frame_nr;

  /// storage for one texture
  GLuint texture[1];
  cv::Mat texture_cv;

  std::vector<Line_collection> height_lines;

  /// size of output image
  // int img_width, img_height;
  float grid_min_x, grid_min_y, grid_width, grid_height;

  Cloud_n normals;

  bool draw_map;

  //float light_z_pos;

  std::vector<cv::Vec3i> triangles;
  cv::Mat uv_values;
  float uv_inv_scale;// inverse size of patch size (used to scale uv-Coordinate to [0,1]
  int patch_center_id;

  void renderFullScreenImage();


  void renderTracks();
  bool render_wireframe;


  bool render_map_image;

  QGLFramebufferObject *fbo;

  // cv::Mat dist_params; /// projector's distortion parameters

  Calibration proj_calibration;/// projector's full calibration

//  void initOffscreenFrameBuffer();
//  bool offscreen_rendering_initialized;
//  GLuint FramebufferName, renderedTexture,depthrenderbuffer;
//  GLenum DrawBuffers[2];



public:
  GL_Mesh_Viewer( QWidget* parent);
  ~GL_Mesh_Viewer();


  void showImage(const cv::Mat img);


  std::map<int,Ant>* ants;


  void initMapSize(float min_x, float min_y, float width, float height);


  void drawMapImage(bool draw_map);

  cv::Mat proj_matrix;


  cv::Mat world2Projector;
  cv::Mat cam_internals;


  bool show_texture;

  void setNormals(const Cloud_n& normals);

  /// use updateHeightLines
  void set_height_lines(const std::vector<Line_collection>& height_lines);

  void LoadGLTextures();

  void toggleDistortion(bool correct_distortion);


  void setProjectorCalibration(Calibration& proj_calib);


  Object_tracker<Grasp,Track<Grasp> >* grasp_tracker;
  Object_tracker<Playing_Piece,Track<Playing_Piece> >* piece_tracker;
  Object_tracker<Fingertip,Track<Fingertip> >* fingertip_tracker;


  void toggleWireframe(bool do_wireframe);

public Q_SLOTS:
  // void setScale(int newscale);
  // void drawMesh(const pcl::PolygonMesh& mesh);
  // GLuint createMeshList(const visualization_msgs::Marker& mesh_marker);

  //     void setMesh(const pcl::PolygonMesh* mesh_);


public:


  /**
  *
  * @param triangles
  * @param uv_values
  * @param scale
  * @see showExpMapTexture, getTriangles (rgbd_utils)
  */
  /// copy information that will be used during showExpMapTexture
  void storeExpMapInfo(const std::vector<cv::Vec3i>& triangles, const cv::Mat& uv_values, float uv_inv_scale, int center_id);

  QPixmap getMapImage(int w, int h);

protected:
  bool map_initalized;

  GLenum glew_state;


  bool setUpMapImage();


  bool setUpProjectorImage_openCV(const cv::Mat& world2projector, const cv::Mat& cam_internal);

  bool setUpProjectorImage();

  void drawSceneDisortion();

  void drawScene();

  void drawList(GLuint list_id);

  void drawMesh();
  void drawMeshStrip();
  void drawMeshWithTexture();


  /// remove information on expMap
  void removeExpMapInfo();

  /// use expMap-Information to show texture
  void showExpMapTexture();



  /**
 * @see drawAnt
 */
  /// draws all ants
  void drawAnts();

  /**
 * @param ant
 * @see drawAnts
 */
  ///Draws a specific ant
  void drawAnt(Ant* ant);

  void drawPath(Ant* ant);


  // void drawPathNoGl();

  void drawHeightLines();

  void initializeGL();
  void paintGL();
  void resizeGL( int w, int h );

  cv::Point2f simulateGlPipeline(float x, float y, float z);




  bool undo_distortion;


  GLuint full_screen_texture_id;
  cv::Mat full_screen_image;

  bool render_into_framebuffer;


public:
  int w_,h_;
  bool show_fullscreenimage;
  bool withDistortion(){return undo_distortion;}

  /// @todo make private
  pcl::PolygonMesh mesh;
  Cloud cloud; // belongs to mesh
};

#endif // GL_MESH_VIEWER_H
