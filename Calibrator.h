/*
 * Calibrator.h
 *
 *  Created on: Jun 28, 2013
 *      Author: mohammad
 */

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_
#include <opencv2/opencv.hpp>

#include "Segment2D.h"
#include "CGAL/Aff_transformation_3.h"
#include <vector>
#include <pcl/point_types.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include "Defines.h"
#include "Landmark.h"
using std::vector;
using namespace pcl;
class Calibrator {
public:
  Calibrator(const PointCloud<PointXYZRGBA>::ConstPtr &Cloud);
  Calibrator();
  virtual ~Calibrator();
  bool Calibrate();
  void SetCloud(const PointCloud<PointXYZRGBA>::ConstPtr &Cloud);
  void Debug3D (visualization::PCLVisualizer& viewer);
  void DrawLandmarkOnImage(IplImage *frameRGB);
  void Plane2RotationMatrix(const ModelCoefficients::Ptr coefficients,CGAL::Aff_transformation_3<CGAL::Cartesian<double> > &rotation,const Landmark &landmark);
  ModelCoefficients::Ptr getPlane();
  const Landmark& getLandmark();
  void calibrationAccepted();
  void SetPosition(CAMERAPOSITION pos);
  CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& getTransformationMatrix();
  void setThreshold(float value);
  const vector<Segment3D>& get3DLines();
private:
  PointCloud<PointXYZRGBA>::ConstPtr Cloud;
  void XYZRGB2IPL(const PointCloud<PointXYZRGBA>::ConstPtr &cloud,IplImage* frameRGB);
  void DebugCostumImage(IplImage* frameRGB);


  /* @brief extracts one surface out of the cloud points
         * @param Data: the raw cloud point with surface within
         * @param Surface: the surface points
         * @param Outliers: any point except surface
         */
  void extractSurface(const PointCloud<PointXYZRGBA>::ConstPtr Data,ModelCoefficients::Ptr coefficients, PointCloud<PointXYZRGBA>::Ptr Surface, PointCloud<PointXYZRGBA>::Ptr Outliers);
  void UEdge(const IplImage& frameRGB,int Threshold,IplImage& edgeImage);
  void extractLines(IplImage *frameRGB,std::vector<Segment3D > &Lines);
  bool find3DLineFor2D(const Segment2D& Segment,const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud,Segment3D& seg3D);
  void CGAL2PCLLines(const vector<Segment3D> &Lines,vector<vtkSmartPointer<vtkDataSet> > & visualLines);
  void FindLandmark(vector<Landmark > &Landmarks);
  void extractIndicesFromCloud(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const vector<PointIndices>& Indices,int n,PointCloud<PointXYZRGBA>::Ptr& extractedCloud);
  void extractIndicesFromCloud(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const boost::shared_ptr<const vector<int> >& Indices,PointCloud<PointXYZRGBA>::Ptr& extractedCloud);
  double LandmarkRate(Landmark &landmark);
  void DrawLandmark(pcl::visualization::PCLVisualizer& viewer,const Landmark& landmark,double r,double g,double b);
  IplImage DrawField(cv::Mat& field);
  vector<vtkSmartPointer<vtkDataSet> > visualLines;
  vector<Segment3D> lines;
  vector<Segment2D> Lines2D;
  vector<Landmark > Landmarks;
  Landmark landmark;
  vector<Point3D> Intersection;
  Eigen::Vector3f Vec1,Vec2,Vec3;
  ModelCoefficients::Ptr coefficients;
  IplImage* frameRGB;
  CGAL::Aff_transformation_3<CGAL::Cartesian<double> > transformationMatrix;
  CAMERAPOSITION Position;
  IplImage *frameYUV;
  IplImage *frameU;
  IplImage *frameCannyU;
  IplImage *frameVerticalSobel;
  IplImage *frameHorizontalSobel;
  CvSize size;
  float edgeThreshold;

};

#endif /* CALIBRATOR_H_ */
