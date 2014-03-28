/*
 * Accessories.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: nao
 */

#include "Accessories.h"

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/search/search.h>
#include "pcl/filters/filter.h"
#include "math.h"
#include "sys/time.h"

using namespace cv;
using namespace std;
using namespace pcl;


int Accessories::DrawImage(IplImage* image)
{
	cvNamedWindow( "Image", CV_WINDOW_AUTOSIZE );
	cvShowImage( "Image", image );
	return cvWaitKey(1000);
}

double Accessories::get_time()
{
	struct timeval t;
	gettimeofday(&t, NULL);
	double d = t.tv_sec + (double) t.tv_usec/1000000;
	return d;
}

double Accessories::point2plane(Point3D upperPoint2Cam,Plane3D upperPlane)
{
return (upperPlane.a() * upperPoint2Cam.x() + upperPlane.b() * upperPoint2Cam.y() + upperPlane.c() * upperPoint2Cam.z() + upperPlane.d())/
        sqrt(upperPlane.a()*upperPlane.a() + upperPlane.b()*upperPlane.b() + upperPlane.c()*upperPlane.c());
}
void Accessories::FindBoundingBox(const PointCloud<PointXYZRGBA>::Ptr Cloud,PointXYZRGBA& min,PointXYZRGBA& max)
{
    double minX=9999999, maxX=-1;
    for(int i=0;i<Cloud->size();i++)
    {
        if(Cloud->at(i).x < minX)
        {
            min = Cloud->at(i);
            minX = Cloud->at(i).x;
        }
        if(Cloud->at(i).x > maxX)
        {
            max = Cloud->at(i);
            maxX = Cloud->at(i).x;
        }
    }

}
void Accessories::FindBoundingBox(const PointCloud<PointXYZRGBA>::Ptr Cloud,PointIndices& indices,PointXYZ& min,PointXYZ& max)
{
    double minX=9999999, maxX=-9999999;
    double minY=9999999, maxY=-9999999;
    double minZ=9999999, maxZ=-9999999;
//    for(int i=0;i<indices.indices.size();i++)
//    {
//        PointXYZRGBA p = Cloud->at(indices.indices[i]);
//        if(p.y < minY)
//        {
//            min = PointXYZ(p.x,p.y,p.z);
//            minY = p.y;
//        }
//        if(p.y > maxY)
//        {
//            max = PointXYZ(p.x,p.y,p.z);
//            maxY = p.y;
//        }
//    }

    for(int i=0;i<indices.indices.size();i++)
    {
        PointXYZRGBA p = Cloud->at(indices.indices[i]);
        if(p.x < minX)
            minX = p.x;
        if(p.y < minY)
            minY = p.y;
        if(p.z < minZ)
            minZ = p.z;

        if(p.x > maxX)
            maxX = p.x;
        if(p.y > maxY)
            maxY = p.y;
        if(p.z > maxZ)
            maxZ = p.z;
    }
    min = PointXYZ(minX,minY,minZ);
    max = PointXYZ(maxX,maxY,maxZ);

}
double Accessories::deg2rad(double angle)
{
	return angle * M_PI/180;
}
double Accessories::rad2deg(double angle)
{
	return angle * 180/M_PI;
}
void Accessories::XYZRGB2IPL(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud,IplImage* frameRGB)
{
    for(int h = 0; h < Cloud->height; h++)
    {
        for(int w = 0; w < Cloud->width; w++)
        {
            int rgbI = h*Cloud->width*3 + w*3;
            int grayI = h*Cloud->width + w;
            frameRGB->imageData[rgbI] =  Cloud->at(grayI).r;
            frameRGB->imageData[rgbI + 1]  =  Cloud->at(grayI).g;
            frameRGB->imageData[rgbI + 2] =  Cloud->at(grayI).b;
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
Accessories::smoothPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & rawInput, float radius, int polynomial_order)
{
	PointCloud<PointXYZRGBA>::Ptr input(new PointCloud<PointXYZRGBA>);
	vector<int> ind;
	removeNaNFromPointCloud(*rawInput,*input,ind);
	pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA> mls;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (radius);
	mls.setSqrGaussParam (radius*radius);
	mls.setPolynomialFit (polynomial_order > 1);
	mls.setPolynomialOrder (polynomial_order);

	mls.setInputCloud (input);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);
	mls.process(*output);
	return (output);
}
double Accessories::angularDiffDeg(double a,double b)
{
    double diff = a - b;
    if ( min(std::abs(diff),360 - std::abs(diff)) == std::abs(diff))
        return diff;
    else
        return 360 - diff;
}
void Accessories::transformMat2Values(const CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& transform, double& rx,double& ry,double& rz,double& tx,double& ty,double& tz)
{
    tx = transform.m(0,3);
    ty = transform.m(1,3);
    tz = transform.m(2,3);

    rx = atan2(transform.m(2,0),transform.m(2,1));
    ry = acos(transform.m(2,2));
    rz = -atan2(transform.m(0,2),transform.m(1,2));
}
