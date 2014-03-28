#ifndef ACCESSORIES_H
#define ACCESSORIES_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include "CGAL/Aff_transformation_3.h"
#include "CGAL/Cartesian.h"
#include "types.h"
using namespace cv;
using namespace pcl;

namespace Accessories
{
int DrawImage(IplImage* image);
double get_time();
void FindBoundingBox(const PointCloud<PointXYZRGBA>::Ptr Cloud,PointXYZRGBA& min,PointXYZRGBA& max);
void FindBoundingBox(const PointCloud<PointXYZRGBA>::Ptr Cloud,PointIndices& indices,PointXYZ& min,PointXYZ& max);
double deg2rad(double angle);
double rad2deg(double angle);
double angularDiffDeg(double a,double b);
void XYZRGB2IPL(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud,IplImage* frameRGB);
void transformMat2Values(const CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& transform, double& rx,double& ry,double& rz,double& tx,double& ty,double& tz);
double point2plane(Point3D upperPoint2Cam,Plane3D upperPlane);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & rawInput, float radius, int polynomial_order);

}

#endif
