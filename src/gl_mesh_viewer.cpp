

#include "rgbd_utils/gl_mesh_viewer.hpp"

//using namespace projector_calibration;
using namespace std;


//#include "projector_calibration/glfractal.h"


void  glVectorToCvMat(const GLdouble* v, cv::Mat &M){

  M = cv::Mat(4,4,CV_64FC1);
  for (uint i=0; i<16; ++i){
    M.at<double>(i%4,i/4) = v[i];
  }
}

void eigenMatrix2GlVector(const Eigen::Matrix4d& matrix, GLdouble* v){
  for (uint x=0; x<4; ++x)
    for (uint y=0; y<4; ++y)
      v[x*4+y] = matrix(y,x);
}


void eigenAffine2GlVector(const Eigen::Affine3f& trafo, GLdouble* v){
  for (uint x=0; x<4; ++x)
    for (uint y=0; y<4; ++y)
      v[x*4+y] = trafo(y,x);
}


void cvMatToglVector(const cv::Mat &M, GLdouble* v){
  assert(M.cols == 4 && M.rows == 4);
  for (uint x=0; x<4; ++x)
    for (uint y=0; y<4; ++y)
      v[x*4+y] = M.at<double>(y,x);
}

void cvMatAffineToglVector(const cv::Mat &M, GLdouble* v){
  assert(M.cols == 4 && M.rows == 3);
  for (uint x=0; x<4; ++x)
    for (uint y=0; y<3; ++y)
      v[x*4+y] = M.at<double>(y,x);

  v[3] = v[3+4]= v[3+8] = 0; v[3+12] = 1;

}




/**
* Constructor that creates a GLFractal widget
*/
GL_Mesh_Viewer::GL_Mesh_Viewer( QWidget* parent)
  : QGLWidget(parent)
{


  frame_nr = 0;

  initializeGL();
  show_texture = false;
  w_ = h_ = -1;
  draw_map = false;
  undo_distortion = false;

  glew_state = 1;

  full_screen_texture_id = 0;

  // withDistortion(false); // render scene without distortion correction

  undo_distortion = false;
  render_map_image = false;

  toggleWireframe(false); // render primitives with faces

  map_initalized = false;
  // offscreen_rendering_initialized = false;

  // FramebufferName = renderedTexture = depthrenderbuffer = 0;
  // DrawBuffers[0] = GL_COLOR_ATTACHMENT0;

}

void GL_Mesh_Viewer::setNormals(const Cloud_n& normals){this->normals = normals;}


GL_Mesh_Viewer::~GL_Mesh_Viewer()
{
}


void GL_Mesh_Viewer::showImage(const cv::Mat img){
  assert(img.type() == CV_8UC3);

  glGenTextures(1, &full_screen_texture_id);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glBindTexture(GL_TEXTURE_2D, texture[0]);

  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

  // ROS_INFO("creating new fullscreen texture with id %i", full_screen_texture_id);
  //  cv::imwrite("texture.png",img);

  glTexImage2D(GL_TEXTURE_2D, 0, 3, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);

  img.copyTo(full_screen_image);

  //  ROS_INFO("drawing image of size %i %i on screen with size %i %i", img.cols, img.rows,h_,w_);

  show_fullscreenimage = true;
}

void GL_Mesh_Viewer::toggleDistortion(bool correct_distortion){
  undo_distortion = correct_distortion;
  show_fullscreenimage = false;
  resizeGL(w_,h_);
}


void GL_Mesh_Viewer::LoadGLTextures() {

  ROS_INFO("Loading texture");

  glGenTextures(1, &texture[0]);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture

  //  texture_cv = cv::imread("imgs/text.bmp");
  texture_cv = cv::imread("imgs/nonpot.bmp");

  assert(texture_cv.type() == CV_8UC3);
  // cv::cvtColor(texture_cv, texture_cv, CV_BGR2RGB);

  // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x size from image, y size from image,
  // border 0 (normal), rgb color data, unsigned byte data, and finally the data itself.
  glTexImage2D(GL_TEXTURE_2D, 0, 3, texture_cv.cols, texture_cv.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_cv.data);


}

/// draw height lines as red GL_LINES
/**
*  @todo: color should depend on height
*/
void GL_Mesh_Viewer::drawHeightLines(){
  if (height_lines.size() == 0) return;


  glClear(GL_DEPTH_BUFFER_BIT);

  glLineWidth(5.0);

  if (render_wireframe){
    glColor3f(1,0,0);
  }else{
    glColor3f(1,1,1);
  }


  glBegin(GL_LINES);
  for (uint i =0; i<height_lines.size(); ++i){
    for (uint j=0; j<height_lines[i].size(); ++j){
      PointPair* pp = &height_lines[i][j];

      glVertex3f(pp->first[0],pp->first[1],pp->first[2]);
      glVertex3f(pp->second[0],pp->second[1],pp->second[2]);
    }
  }

  glEnd();

}


void GL_Mesh_Viewer::drawAnts(){

  //ROS_INFO("Drawing %zu ants", ants->size());

  if (!ants) return;

  for (std::map<int,Ant>::iterator it = ants->begin(); it != ants->end(); ++it){
    drawPath(&it->second);
  }

  for (std::map<int,Ant>::iterator it = ants->begin(); it != ants->end(); ++it){
    drawAnt(&it->second);
  }

}


/**
* @todo store cloud
* @param ant
*/
void GL_Mesh_Viewer::drawAnt(Ant* ant){

//   ROS_INFO("Drawing ant with id %i", ant->getId());

  if (ant->getState() == ANT_AT_GOAL){
    return;
  }


   glClear(GL_DEPTH_BUFFER_BIT);


  if (ant->getState() == ANT_NOT_INITIALIZED){
    ROS_WARN("Can't show uninitialized ant");
    return;
  }

  cv::Point pos = ant->getPosition();
  cv::Point next_pos = ant->getNextPosition();

  pcl_Point P = cloud.at(pos.x, pos.y);
  pcl_Point P_next = cloud.at(next_pos.x, next_pos.y);// next position on path

  float frac = ant->getStepFraction();

  // ROS_INFO("pos: %i, frac: %f", ant->getPosInPath(),frac);
  pcl_Point inter = interpolate(P,P_next,frac);


  GLUquadricObj * quadric = gluNewQuadric();
  gluQuadricDrawStyle(quadric, GLU_FILL);

  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslatef(inter.x,inter.y,inter.z);
  glColor3f(1,0,0);
  gluSphere(quadric,0.02,30,30); // sphere with 3cm radius
  glPopMatrix();




}



/// Visualize Path
/**
* @todo: render as cylinder strip (gluCylinder)
*/
void GL_Mesh_Viewer::drawPath(Ant* ant){

  if (ant->getState() == ANT_AT_GOAL){
    return;
  }

  assert(cloud.width > 1 && cloud.height > 1);

  glColor3f(0,1,0);
  glLineWidth(5.0);

  // bool with_colors = (pathColors.size() == path.size());
  // cv::Vec3b* col;

  glBegin(GL_LINE_STRIP);

  int size = ant->getPathLenth();

  // ROS_INFO("Path length: %i", size);

  for (int i=0; i<size; ++i){

    cv::Point pos = ant->getPositionAt(i);

    pcl_Point p = cloud.at(pos.x, pos.y);

    //  ROS_INFO("Path point: %f %f %f", p.x,p.y,p.z);
    //  if (with_colors){
    //   col = &pathColors[i];
    //   glColor3f(col->val[0]/255.0, col->val[1]/255.0, col->val[2]/255.0);
    //  }

    glVertex3f(p.x,p.y,p.z);
  }

  glEnd();


}


/**
*
* @param triangles  list of triangles on surface with uv-coordinates for each corner
* @param uv_values  map of all uv-coordinates
* @param uv_inv_scale inverse of minimal geodesic distance
* @param center_id  id of center point of patch
*/
void GL_Mesh_Viewer::storeExpMapInfo(const std::vector<cv::Vec3i>& triangles, const cv::Mat& uv_values, float uv_inv_scale,int center_id){
  this->triangles = triangles;
  this->uv_values = uv_values;
  this->uv_inv_scale = uv_inv_scale;

  patch_center_id = center_id;

  // Cloud cloud;
  // pcl::fromROSMsg(mesh.cloud, cloud);
  // ROS_INFO("cloud: %zu", cloud.size());
  // this->patch_center.x = center_id%cloud.width;
  // this->patch_center.y = center_id/cloud.width;
  // ROS_INFO("centerid: %i, x: %i y: %i, width: %i", center_id, this->patch_center.x,this->patch_center.y,cloud.width);

}

void GL_Mesh_Viewer::removeExpMapInfo(){ this->triangles.clear();}


void GL_Mesh_Viewer::showExpMapTexture(){

  if (triangles.empty())
    return;


  assert(uv_values.type() == CV_32FC2);

  glEnable(GL_TEXTURE_2D);

  if (texture_cv.cols != 256)
    LoadGLTextures();

  //  Cloud cloud;
  //  pcl::fromROSMsg(mesh.cloud, cloud);

  glBindTexture(GL_TEXTURE_2D, texture[0]);   // choose the texture to use.

  glColor3f(1,1,1);

  glBegin(GL_TRIANGLES);

  // cv::Point patch_center(patch_center_id%cloud.width,patch_center_id/cloud.width);
  // ROS_INFO("center: %i %i", patch_center.x , patch_center.y);

  for (uint i=0; i<triangles.size(); i++){

    pcl_Point pts[3];
    float us[3];
    float vs[3];

    bool all_inside = true;

    for (uint j = 0; j<3; ++j){

      int idx = triangles[i][j];


      int x = idx%cloud.width;
      int y = idx/cloud.width;

      cv::Vec2f uv = uv_values.at<cv::Vec2f>(y,x);

      us[j] = ((uv.val[0]*uv_inv_scale)+1)/2;
      vs[j] = ((uv.val[1]*uv_inv_scale)+1)/2;

      // only draw triangle if all points are within the [0,1]^2-area
      if (us[j] < 0 || 1 < us[j] || vs[j] < 0 || 1 < vs[j]){
        all_inside = false;
        break;
      }

      pts[j] = cloud.points[idx];
    }

    if (all_inside){

      for (uint j=0; j<3; ++j){

        //    float diff = norm(sub(pts[j],pts[(j+1)%3]));
        //    if (diff>0.02){
        //     ROS_INFO("Large edge (i=%i): %f %f %f, %f %f %f",i,pts[j].x, pts[j].y, pts[j].z,pts[(j+1)%3].x, pts[(j+1)%3].y, pts[(j+1)%3].z );
        //    }

        glTexCoord2f(us[j],vs[j]);
        glVertex3f(pts[j].x, pts[j].y, pts[j].z);
      }
    }


  }

  glEnd();

  glDisable(GL_TEXTURE_2D);

}


void GL_Mesh_Viewer::drawMeshWithTexture(){

  glEnable(GL_TEXTURE_2D);

  if (texture_cv.cols != 256)
    LoadGLTextures();


  //  Cloud cloud;
  //  pcl::fromROSMsg(mesh.cloud, cloud);

  glBindTexture(GL_TEXTURE_2D, texture[0]);   // choose the texture to use.

  //ROS_INFO("w.h: %i %i", cloud.width, cloud.height);


  glBegin(GL_TRIANGLES);

  for (uint i=0; i<mesh.polygons.size(); i++){
    for (uint j = 0; j<3; ++j){

      int idx = mesh.polygons[i].vertices[j];

      pcl_Point p = cloud.points[idx];

      int x = idx%cloud.width;
      int y = idx/cloud.width;

      // ROS_INFO("x,y: %i %i u,v: %f %f", x,y,x/(1.0*(cloud.width-1)), y/(1.0*(cloud.height-1)));

      glTexCoord2f(x/(1.0*(cloud.width-1)),y/(1.0*(cloud.height-1)));
      glVertex3f(p.x, p.y, p.z);
    }
  }

  glEnd();


  glDisable(GL_TEXTURE_2D);

  // // Front Face (note that the texture's corners have to match the quad's corners)
  //
  // float x = 0.1;
  // float y = 0.1;
  //
  // // glColor3f(1,0,0);
  //
  // glTexCoord2f(0.0f, 0.0f);
  // glVertex3f(-x, -y,  0.0f);  // Bottom Left Of The Texture and Quad
  //
  // glTexCoord2f(0.0f, 1.0f);
  // glVertex3f( x, -y,  0.0f);  // Bottom Right Of The Texture and Quad
  //
  // glTexCoord2f(1.0f, 1.0f);
  // glVertex3f( x,  y,  0.0f);  // Top Right Of The Texture and Quad
  //
  // glTexCoord2f(1.0f, 0.0f);
  // glVertex3f(-x,  y,  0.0f);  // Top Left Of The Texture and Quad



}


void GL_Mesh_Viewer::toggleWireframe(bool do_wireframe){

  render_wireframe = do_wireframe;

  if (do_wireframe){
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  }else{
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  }

}


/**
* @todo check for holes in the mesh
*/
/// Draw mesh with Triangle_Strip
void GL_Mesh_Viewer::drawMeshStrip(){

  timing_start("meshstrip");

  glLineWidth( 3.0f ); // in case of wireframe mode


  if (cloud.width == 0){
    ROS_INFO("no mesh given");
    return;
  }

  //  ROS_INFO("Mesh size: %i %i",mesh.cloud.width,mesh.cloud.height);

  //  Cloud cloud;
  //  pcl::fromROSMsg(mesh.cloud, cloud);

  pcl_Point p;

  //  ROS_INFO("cloud size: %i %i",cloud.width,cloud.height);


  for (uint x = 0; x<cloud.width-1; ++x){

    glBegin(GL_TRIANGLE_STRIP);

    for (uint y = 0; y<cloud.height; ++y){
      p = cloud.at(x,y);
      glColor3f(p.r/255.0, p.g/255.0, p.b/255.0);
      glVertex3f(p.x, p.y, p.z);


      p = cloud.at(x+1,y);
      glColor3f(p.r/255.0, p.g/255.0, p.b/255.0);
      glVertex3f(p.x, p.y, p.z);
    }

    glEnd();

  }

  timing_end("meshstrip");

}



void GL_Mesh_Viewer::drawMesh(){


  //  // 0.06 ms
  //  Cloud cloud;
  //  pcl::fromROSMsg(mesh.cloud, cloud);


  bool normals_valid = (normals.size() == cloud.size());

  glBegin(GL_TRIANGLES);


  std::vector<pcl::Vertices>::iterator it = mesh.polygons.begin();
  std::vector<pcl::Vertices>::iterator end = mesh.polygons.end();
  pcl_Point* p;
  pcl_Normal* n;

  int undrawn_triangle_cnt = 0;

  for (;it != end; ++it){


    // TODO: compute normals for all points
    // HACK: use 1,0,0 for points without normals
    if (normals_valid){
      bool nan = false;
      for (uint i=0; i<3; ++i){
        n = &normals.points[it->vertices[i]];
        if  (n->normal_x != n->normal_x){
          undrawn_triangle_cnt++;
          nan = true;
          break;
        }
      }
      if (nan) continue;
    }

    for (uint i=0; i<3; ++i){
      p = &cloud.points[it->vertices[i]];
      glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);

      if (normals_valid){
        n = &normals.points[it->vertices[i]];
        glNormal3f(n->normal_x, n->normal_y, n->normal_z);

        assert(n->normal_x == n->normal_x);

        //     ROS_INFO("pos: %f %f %f, normal: %f %f %f",p->x, p->y, p->z,n->normal_x, n->normal_y, n->normal_z);
      }

      glVertex3f(p->x, p->y, p->z);
    }

    //  p = &cloud.points[it->vertices[1]];
    //  glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);
    //  glVertex3f(p->x, p->y, p->z);
    //
    //  p = &cloud.points[it->vertices[2]];
    //  glColor3f(p->r/255.0, p->g/255.0, p->b/255.0);
    //  glVertex3f(p->x, p->y, p->z);
  }

  glEnd();


  // ROS_INFO("Couldn't draw %i of %zu triangles because of missing normals", undrawn_triangle_cnt,mesh.polygons.size());


}



void GL_Mesh_Viewer::drawList(GLuint list_id){
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable( GL_DEPTH_TEST );
  // glLoadIdentity();
  // glTranslatef(0.0, 0.0, -10.0);
  // // set the zoom according to the scale variable
  // glScalef(scale, scale, scale);
  // // set the image rotation up according to xRot, yRot, zRot
  // glRotatef( xRot, 1.0, 0.0, 0.0);
  // glRotatef( yRot, 0.0, 1.0, 0.0);
  // glRotatef( zRot, 0.0, 0.0, 1.0);

  glCallList(list_id);
}





void GL_Mesh_Viewer::initMapSize(float min_x, float min_y, float width, float height){

  map_initalized = true;

  grid_min_x = min_x;
  grid_min_y = min_y;
  grid_width = width;
  grid_height = height;
}


void GL_Mesh_Viewer::set_height_lines(const std::vector<Line_collection>& height_lines){
  this->height_lines = height_lines;
}


cv::Point2f GL_Mesh_Viewer::simulateGlPipeline(float x, float y, float z){


  ROS_INFO("simulating Pipeline: %f %f %f",x,y,z);

  GLdouble v[16];

  cv::Mat MV(4,4,CV_64FC1);
  glGetDoublev(GL_MODELVIEW_MATRIX,v);
  glVectorToCvMat(v,MV);


  cout << "Modelview Matrix: " << MV << endl;

  cv::Mat P(4,4,CV_64FC1);
  glGetDoublev(GL_PROJECTION_MATRIX,v);
  glVectorToCvMat(v,P);

  cout << "GL_PROJECTION " << endl << P << endl;


  cv::Mat Proj = P * MV;

  cout << "Full Matrix from OpenGL" << endl << Proj << endl;









  // ROS_INFO("GL_PROJECTION");
  // for (uint x_=0; x_<4; ++x_){
  //  for (uint y=0; y<4; ++y)
  //   cout << v[y*4+x_] << " ";
  //  cout << endl;
  // }


  cv::Mat Pos(4,1,CV_64FC1);
  Pos.at<double>(0) = x;
  Pos.at<double>(1) = y;
  Pos.at<double>(2) = z;
  Pos.at<double>(3) = 1;



  cv::Mat local = MV*Pos;


  cout << "local frame: (MV*Pos) = " << local << endl;

  cv::Mat x_clip = P*local;

  cout << "After projection (P*local): " << x_clip << endl;




  cv::Mat x_ndc =  x_clip / x_clip.at<double>(3);


  cout << "ndc: " << x_ndc << endl;


  cout << "viewport " << endl;
  glGetDoublev(GL_VIEWPORT,v);
  for (uint i=0; i<4; ++i)
    cout << v[i] << " ";
  cout << endl;

  float x_img = v[0];
  float y_img = v[1];
  float w = v[2];
  float h = v[3];


  cv::Point2f res;

  res.x = w/2*x_ndc.at<double>(0)+(x_img+w/2);
  res.y = h/2*x_ndc.at<double>(1)+(y_img+h/2);


  //  res.x = (dc.at<double>(0)+1)/2*w_;
  //  res.y = (dc.at<double>(1)+1)/2*h_;

  cout << "screen: " << res.x << "  " << res.y << endl;

  //  cv::Mat cor = proj_matrix*Pos;

  //  cor /= cor.at<double>(2);

  //  ROS_INFO("original: %f %f %f", cor.at<double>(0),cor.at<double>(1),cor.at<double>(2));


  return res;
}


bool GL_Mesh_Viewer::setUpMapImage(){

  if (!map_initalized)
    return false;

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable( GL_DEPTH_TEST );

  glMatrixMode(GL_MODELVIEW_MATRIX);
  glLoadIdentity();
  glRotatef(90,0,0,1);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  //  ROS_INFO("seting up map image: %f %f %f %f",grid_min_x,grid_min_y,grid_width,grid_height);

  glOrtho(grid_min_y+grid_height,grid_min_y,grid_min_x+grid_width,grid_min_x,-5,5);

  glViewport(0,0,w_,h_);

  return true;
}


bool GL_Mesh_Viewer::setUpProjectorImage_openCV(const cv::Mat& world2projector, const cv::Mat& cam_internal){


  if (cam_internal.cols != 3){
    ROS_WARN("NO CAM MATRIX");
    return false;
  }



  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable( GL_DEPTH_TEST );

  GLdouble model_view[16]; // transformation from world to projector frame


  cvMatAffineToglVector(world2projector,model_view);
  //  eigenAffine2GlVector(world2projector,model_view);
  glMatrixMode(GL_MODELVIEW_MATRIX);
  glLoadIdentity();
  glMultMatrixd(model_view);

  cv::Mat P(4,4,CV_64FC1);
  //  glGetDoublev(GL_MODELVIEW_MATRIX,model_view);
  //  glVectorToCvMat(model_view,P);

  //  cout << "GL_MODELVIEW_MATRIX " << endl << P << endl;



  //  cv::Mat foo;
  //  glVectorToCvMat(model_view,foo);
  //  cout << "model_view: " << endl << foo << endl;


  // projection into NDC (internal calibration of camera)

  int w,h;// size of render area

  if (render_into_framebuffer){
    w = fbo->width();
    h = fbo->height();
  }else{
    w = w_;
    h = h_;
  }


  Eigen::Matrix4d frustrum;
  build_opengl_projection_for_intrinsics(frustrum,cam_internal,w,h,0.1,10); // last two parameters: range of z (near and far plane


  //  cout << "frustrum" << endl;
  //  printMatrix(frustrum);

  GLdouble fru[16];
  eigenMatrix2GlVector(frustrum,fru);


  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,w,h,0,-10,10);
  glMultMatrixd(fru);

  //  GLdouble proj[16];
  //  glGetDoublev(GL_PROJECTION,proj);
  //  glVectorToCvMat(proj,P);

  // scaling from NDC onto image coordinates
  glViewport(0,0,w,h);


  return true;
}


bool GL_Mesh_Viewer::setUpProjectorImage(){


  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable( GL_DEPTH_TEST );

  GLdouble model_view[16];


  if (proj_matrix.cols != 4){
    ROS_INFO("GL_Mesh_Viewer::setUpProjectorImage: No projection matrix given!");
    return false;
  }

  for (uint x=0; x<3; ++x){
    for (uint y=0; y<4; ++y){

      if (x < 2)
        model_view[y*4+x] = proj_matrix.at<double>(x,y);
      else{
        model_view[y*4+(x+1)] = proj_matrix.at<double>(x,y);
        model_view[y*4+x] = 0;
      }
    }
  }

  model_view[10] = 1; // MV(3,3) = 1


  glMatrixMode(GL_MODELVIEW_MATRIX);
  glLoadIdentity();
  glMultMatrixd(model_view);


  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,w_,h_,0,-20,20);

  // ROS_INFO("image set up: %i %i",w_,h_);

  glViewport(0,0,w_,h_);

  return true;

}


void GL_Mesh_Viewer::drawSceneDisortion(){

  // ROS_INFO("drawing WITH distortion correction");

  glEnable(GL_TEXTURE_2D);

  if (!fbo->isValid()){
    fbo = new QGLFramebufferObject(w_,h_);
  }


  glViewport( 0, 0, w_, h_ );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  glFrustum(-1.0,1.0,-2.0,2.0, 1, 3);
  glMatrixMode( GL_MODELVIEW );


  timing_start("render to framebuffer");

  fbo->bind();

  render_into_framebuffer = true; // use size of invisible framebuffer during rendering
  drawScene();
  render_into_framebuffer = false;

  fbo->release();
  timing_end("render to framebuffer");

  //  char fn[100];
  //  sprintf(fn,"buffer_img%i.png",frame_nr++);
  //  QImage img = fbo->toImage();
  //  img.save(fn);


  glBindTexture(GL_TEXTURE_2D, fbo->texture()); // use created texture during painting

  // draw points with uv coordinate not within [0,1] black
  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER  );
  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER  );
  float color[] = { 0.0f, 0.0f, 0.0f, 1.0f };
  glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color );


  glColor3f(1,1,1);

  glDisable(GL_DEPTH_TEST );

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0f, w_, h_, 0.0f, 0.0f, 1.0f);

  glViewport(0,0,w_,h_);


  int step = 10;

  float cx = proj_calibration.c_x();
  float cy = proj_calibration.c_y();


  cv::Point2f distorted;
  distorted = proj_calibration.distortPoint(cv::Point2f(cx,cy));

  // ROS_INFO("center point: %f %f to %f %f",cx,cy,distorted.x,distorted.y);

  for (float y = 0; y <= h_+step; y+= step){
    glBegin(GL_TRIANGLE_STRIP);

    for (float x = 0; x <= w_+step; x+=step){

      distorted = proj_calibration.distortPoint(cv::Point2f(x,h_-y));
      glTexCoord2f(x/w_,y/h_);
      glVertex3f(distorted.x,distorted.y,0);

      float y_next = y+step;

      distorted = proj_calibration.distortPoint(cv::Point2f(x,h_-y_next));

      glTexCoord2f(x/w_,y_next/h_);
      glVertex3f(distorted.x,distorted.y,0);

    }

    glEnd();

  }
  glBindTexture(GL_TEXTURE_2D,0);



  // enable to overlay a visualization of the distortion
  bool show_distortion_graphic = false;

  if (show_distortion_graphic){


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, w_, h_, 0.0f, 0.0f, 1.0f);

    glViewport(0,0,w_,h_);

    glDisable(GL_DEPTH_TEST);

    //      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);

    glLineWidth(1);
    glPointSize(1);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    int step_ = 30;
    // render points (very short lines are not rendered
    glBegin(GL_POINTS);
    for (int y = 0; y <= h_; y+= step_){
      for (int x = 0; x <= w_; x+=step_){
        glColor3f(0,1,1);
        glVertex3f(x,y,0);
      }
    }
    glEnd();

    // mark center of distortion
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(proj_calibration.c_x(),proj_calibration.c_y(),0);
    glEnd();

    glBegin(GL_LINES);

    float dist_sum = 0;
    int cnt = 0;

    float max_dist = 0;

    for (int y = 0; y <= h_; y+= step_){
      for (int x = 0; x <= w_; x+= step_){

        cv::Point2f res,undis;
        res = proj_calibration.distortPoint(cv::Point2f(x,y));
        // undis = proj_calibration.unDistortPoint(cv::Point2f(res.x,res.y));
        // ROS_INFO("pt: %i %i, dist: %f %f, undis: %f %f, (err: %f)", x,y,res.x,res.y,undis.x,undis.y,dist(cv::Point2f(x,y),undis));

        //res = proj_calibration.unDistortPoint(cv::Point2f(x,y));

        glColor3f(1,0,0);
        glVertex3f(x,y,0);

        glColor3f(0,1,1);
        glVertex3f(res.x,res.y,0);

        dist_sum += dist(cv::Point2f(x,y),res);
        max_dist = max(max_dist, dist(cv::Point2f(x,y),res));
        cnt++;
      }
    }

    glEnd();

    // ROS_INFO("Mean dist: %f, max: %f", dist_sum/cnt, max_dist);

    return;

  }


}


void GL_Mesh_Viewer::renderFullScreenImage(){

  if (full_screen_texture_id < 1){
    ROS_WARN("NO fullscreen texture initalized!");
    return;
  }

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // ROS_INFO("drawing fullscrenn texture with id %i",full_screen_texture_id);

  glEnable(GL_TEXTURE_2D);

  glColor3f(1,1,1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0f, w_, h_, 0.0f, 0.0f, 1.0f);

  glViewport(0,0,w_,h_);

  if ( full_screen_image.cols != w_ || full_screen_image.rows != h_){
    ROS_WARN("Rendering image of size %i %i on screen with size %i %i", full_screen_image.cols,full_screen_image.rows, w_,h_);
  }

  glBegin(GL_QUADS);

  glTexCoord2f(0,1);
  glVertex3f(0,h_,0);

  glTexCoord2f(1,1);
  glVertex3f(w_,h_,0);

  glTexCoord2f(1,0);
  glVertex3f(w_,0,0);

  glTexCoord2f(0,0);
  glVertex3f(0,0,0);

  glEnd();

}


void GL_Mesh_Viewer::renderTracks(){

  // ROS_INFO("Drawing sphere");

  GLUquadricObj *quadric = gluNewQuadric();
  gluQuadricDrawStyle(quadric, GLU_FILL );

  glClear(GL_DEPTH_BUFFER_BIT);

  glMatrixMode( GL_MODELVIEW );


  //  ROS_INFO("drawing spheres for %zu piece tracks", piece_tracker->tracks.size());

  for (PieceTrack_it it = piece_tracker->tracks.begin(); it != piece_tracker->tracks.end(); ++it){
    // if (it->second.state == Track_Active)
    //it->second.visualizeOnImage(current_col_img,getColor(it->first),TT_FINGERTIP);
    pcl_Point center = it->second.last_detection()->position_world;
    cv::Scalar color =  getColor(it->first);

    glColor3f(color.val[0]/255.0,color.val[1]/255.0,color.val[2]/255.0);

    ROS_INFO("Object %i has size %f",it->first,it->second.last_detection()->area);

    // ROS_INFO("pos in world: %f %f %f", center.x,center.y,center.z);

    glPushMatrix();
    glTranslatef(center.x,center.y,center.z);
    gluSphere(quadric,0.03,30,30); // sphere with 3cm radius
    glPopMatrix();


    glLineWidth(20);
    glBegin(GL_LINE_STRIP);

    for (uint i=0; i < it->second.detections.size(); ++i){
      pcl_Point &p = it->second.detections[i].position_world;
      glVertex3f(p.x,p.y,p.z+0.01);
    }

    glEnd();

    // ROS_INFO("Visualizing Object at: %f %f %f (visualized at %f %f)", center.x,center.y, center.z, px.x,px.y);
  }



  for (FingerTrack_it it = fingertip_tracker->tracks.begin(); it != fingertip_tracker->tracks.end(); ++it){
    if (it->second.state == Track_Active){
      pcl_Point center = it->second.last_detection()->position_world;

      cv::Scalar color =  getColor(it->first);
      glColor3f(color.val[0]/255.0,color.val[1]/255.0,color.val[2]/255.0);

      glPushMatrix();
      glTranslatef(center.x,center.y,center.z);
      gluSphere(quadric,0.03,30,30);
      glPopMatrix();

      glLineWidth(20);
      glBegin(GL_LINE_STRIP);

      for (uint i=0; i < it->second.detections.size(); ++i){
        pcl_Point &p = it->second.detections[i].position_world;
        glVertex3f(p.x,p.y,p.z);
      }

      glEnd();
    }
  }

  for (GraspTrack_it it = grasp_tracker->tracks.begin(); it != grasp_tracker->tracks.end(); ++it){
    if (it->second.state == Track_Active){
      pcl_Point center = it->second.last_detection()->position_world;

      cv::Scalar color =  getColor(it->first);
      glColor3f(color.val[0]/255.0,color.val[1]/255.0,color.val[2]/255.0);

      glPushMatrix();
      glTranslatef(center.x,center.y,center.z);

      gluPartialDisk(quadric, 0.02,0.04,12,4,0,360);

      glPopMatrix();

    }
  }








  //  glPushMatrix();




}


void GL_Mesh_Viewer:: drawScene(){


  timing_start("paintgl");

  if (render_map_image){
    if (!setUpMapImage()){
      ROS_WARN("Can't show map if grid size is not defined!");
      return;
    }
  }else{
    if (!setUpProjectorImage_openCV(world2Projector,cam_internals)){
      ROS_WARN("NO OPENCV cam matrix");
      return;
    }
  }


  if (show_texture){
    // ROS_INFO("showing texture");
    showExpMapTexture();
  }else{
    drawMeshStrip();
  }

  //  // visualize objects and gestures
  renderTracks();


  if (height_lines.size() > 0){
    drawHeightLines();
  }





  // drawing path of ant as GlLine
  // drawPath();

  //ROS_INFO("drawing ants");
   drawAnts();

  // if (path.size() > 0)
  //  drawPath();
  //  if (show_texture)
  // else {
  //   drawMeshWithTexture();
  //  else
  //   drawMesh();
  // }

  timing_end("paintgl");

}


QPixmap GL_Mesh_Viewer::getMapImage(int w, int h){

  render_map_image = true; // used to select the correct OpenGL-matrices during painting

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();


  int w_old = w_;
  int h_old = h_;

  QPixmap pixmap = renderPixmap(w,h);
  render_map_image = false;

  h_ = h_old; w_ = w_old;

  resizeGL(w_old,h_old);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();


  return pixmap;


}


void GL_Mesh_Viewer::paintGL()
{


  if (show_fullscreenimage){
    renderFullScreenImage();
    return;
  }


  if (render_map_image){
    ROS_INFO("rendering nmap");
    drawScene();
    return;
  }


  if (undo_distortion){
    drawSceneDisortion();
    return;
  }else{
    drawScene();
  }

}


void GL_Mesh_Viewer::setProjectorCalibration(Calibration& proj_calib){
  proj_calib.copyTo(proj_calibration);
}


/*--------------------------------------------------------------------------*/
/**
* Set up the OpenGL rendering state, and define display list
*/

void GL_Mesh_Viewer::initializeGL()
{
  glClearColor( 0.0, 0.0, 0.0, 0.0 ); // Let OpenGL clear to black
  glShadeModel( GL_SMOOTH ); // we want smooth shading . . . try GL_FLAT if you like

}

/*--------------------------------------------------------------------------*/
/**
* Set up the OpenGL view port, matrix mode, etc.
*/
void GL_Mesh_Viewer::resizeGL( int w, int h )
{
  w_ = w;
  h_ = h;

  // use larger framebuffer to improve quality of texture before distortion
  int scale = 1;
  makeCurrent();
  fbo = new QGLFramebufferObject(scale*w_,scale*h_);


  glViewport( 0, 0, (GLint)w, (GLint)h );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  glFrustum(-1.0,1.0,-2.0,2.0, 1, 3);
  glMatrixMode( GL_MODELVIEW );
}



