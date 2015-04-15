#ifndef _SGPOINTSGET_H_INCLUDED
#define _SGPOINTSGET_H_INCLUDED

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <GeometryHandler/GeometryHandle.h>
#include <math.h>

#define PI 3.141592

namespace grasp {

class SgPointsRenderer;

class SgPointsGet: public cnoid::SgVisitor {
public:
  std::vector<cnoid::SgShape*> shape;
  virtual void visitShape(cnoid::SgShape* shape) {
    this->shape.push_back(shape);
  }
};

class SgPointsRenderer: public cnoid::SgCustomGLNode {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SgPointsRenderer(pcl::PointCloud<pcl::PointXYZ>* pcd) {
    setRenderingFunction(boost::bind(&SgPointsRenderer::renderPoints, this));
    this->pcd = pcd;
  }

  void renderPoints() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable (GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBegin (GL_POINTS);

    float d = 0, r = 0, g = 0, b = 0;

    // Cyan→Green→Yellow→Red→Magenta→Blue→Cyan
    // 水色(0.7m)→緑色→黄色(t.7m)→赤色→赤紫色(2t.7m)→青色→水色
    float t = 4;
    for (int i = 0; i < pcd->points.size(); i++) {
      d = sqrt((pcd->points[i].x) * (pcd->points[i].x) + (pcd->points[i].y) * (pcd->points[i].y)) - 0.7;
      if (d < t) {
        r = 1 - cos((d / t) * (PI / 2));
        g = 1;
        b = 1 - sin((d / t) * (PI / 2));
      } else if (d < 2 * t) {
        d -= t;
        r = 1;
        g = 1 - sin((d / t) * (PI / 2));
        b = 1 - cos((d / t) * (PI / 2));
      } else {
        d -= 2 * t;
        r = 1 - sin((d / t) * (PI / 2));
        g = 1 - cos((d / t) * (PI / 2));
        b = 1;
      }
      glColor3f(r, g, b);
      glVertex3d(pcd->points[i].x, pcd->points[i].y, pcd->points[i].z);
    }
    /*
    for (int i = 0; i < pcd->points.size(); i++) {
      d = sqrt((pcd->points[i].x) * (pcd->points[i].x) + (pcd->points[i].y) * (pcd->points[i].y));
      if (d < 0.95) {
        r = 1;
        g = 0;
        b = 0;
      } else {
        r = 1;
        g = 1;
        b = 1;
      }
      glColor3f(r, g, b);
      glVertex3d(pcd->points[i].x, pcd->points[i].y, pcd->points[i].z);
    }
    */
    /* 色空間見本
     for(int i=0; i<100; i++) {
     for(int j=0; j<100; j++) {
     for(int k=0; k<100; k++) {
     r = (float)i/100;
     g = (float)j/100;
     b = (float)k/100;
     glColor3f(r,g,b);
     glVertex3d((float)i/10,(float)j/10,(float)k/10);
     }
     }
     }
     */
    /*
     float t=5;
     float x,y;
     for(int l=1; l<11; l++) {
     for(int j=0; j<360; j++) {
     x = (float)(i-200)/10;
     y = (float)(j-200)/10;
     glColor3f(r,g,b);
     glVertex3d(x,y,0);
     }
     }
     */
    glEnd();
    glPopAttrib();
  }
  virtual void accept(cnoid::SgVisitor& visitor) {
    cnoid::SgCustomGLNode::accept(visitor);
    SgLastRenderer(this, true);
  }
  static SgPointsRenderer* SgLastRenderer(SgPointsRenderer* sg, bool isWrite) {
    static SgPointsRenderer* last;
    if (isWrite) last = sg;
    return last;
  }
  pcl::PointCloud<pcl::PointXYZ>* pcd;
};

typedef boost::intrusive_ptr<SgPointsRenderer> SgPointsRendererPtr;
}

#endif
