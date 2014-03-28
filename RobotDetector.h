/*
 * RobotDetector.h
 *
 *  Created on: Jun 28, 2013
 *      Author: mohammad
 */

#ifndef ROBOTDETECTOR_H_
#define ROBOTDETECTOR_H_
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include "pcl/visualization/pcl_visualizer.h"
//#include <pcl/people/ground_based_people_detection_app.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Landmark.h"
#include "Segment2D.h"
#include <QPoint>
#include "Defines.h"
#include "cluster.h"
#include "pcl/ModelCoefficients.h"
using namespace std;
class RobotDetector {
public:
	RobotDetector(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud);
	RobotDetector();
	virtual ~RobotDetector();

	void Detect();
	void setCalibrationConfigs(const pcl::ModelCoefficients::Ptr coefficients,const CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& transformationMatrix);
	void Debug3D(pcl::visualization::PCLVisualizer& viewer);
	void SetCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud);
    std::vector<pcl::PointXYZ> GetBlueRobotsToField();
    std::vector<pcl::PointXYZ> GetRedRobotsToField();
    std::vector<pcl::PointXYZ> GetBlackRobotsToField();
    std::vector<pcl::PointXYZ> GetWhiteRobotsToField();
    std::vector<pcl::PointXYZ> GetBallToField();
    std::vector<pcl::PointXYZ> GetRefereeToField();
    pcl::PointIndices GetRobotCloud();
    pcl::ModelCoefficients::Ptr getUpperPlane();
    Segment3D getFOVLines();
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCluserPointCloud(int i);
    const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >& getBoundingBoxes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RobotCloud;
private:

    void PerformCluster(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud, std::vector<Cluster >& clusters,float clusterThreshold,int minPoints,int maxPoints);
	void extractIndicesFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const vector<pcl::PointIndices>& Indices,int n,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& extractedCloud);
	void extractIndicesFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const boost::shared_ptr<const vector<int> >& Indices,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& extractedCloud);

	void RemoveFromSurface(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const Plane3D& plane,double dist,pcl::PointIndices& Inliers, pcl::PointIndices& Outliers);

    ObjectId guessColor(const pcl::PointXYZRGBA& pix,ObjectGuess guess);
    void stickID(std::vector<Cluster>& clusters);
	void DrawClustersIn2D(cv::Mat& image,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ClustersCloud);
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & rawInput, float radius, int polynomial_order);
	double FindRotationFromBoundingBox(const Point3D &min,const Point3D &max);
    void RemoveNoise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RobotCloud,std::vector<Cluster >& clusters);
	void SelectClusterPoint();
    void BallFinder(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const pcl::PointIndices& Inliers);
	IplImage DrawField(cv::Mat& matField);
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Cloud;
	pcl::ModelCoefficients::Ptr coefficients;
	Landmark landmark;
	CGAL::Aff_transformation_3<CGAL::Cartesian<double> > transformationMatrix;

    std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> > boxes;
	// Create classifier for people detection:  
//	pcl::people::PersonClassifier<pcl::RGB> person_classifier;

	// People detection app initialization:
//	pcl::people::GroundBasedPeopleDetectionApp<pcl::PointXYZRGBA> people_detector;    // people detection object
//	std::vector<pcl::people::PersonCluster<pcl::PointXYZRGBA> > clusters;   // vector containing persons clusters

    pcl::PointIndices OutlierIndices,InlierIndices;
    std::vector<Cluster> clusters;
    Plane3D upperPlane,leftPlane,rightPlane,lowerPlane;
    Plane3D fieldPlane;
};

#endif /* ROBOTDETECTOR_H_ */
