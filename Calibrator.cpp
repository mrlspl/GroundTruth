/*
 * Calibrator.cpp
 *
 *  Created on: Jun 28, 2013
 *      Author: mohammad
 */

#include "Calibrator.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Accessories.h"
#include <iostream>
#include "string.h"
#include <CGAL/Line_3.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace CGAL;
Calibrator::Calibrator(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud):Cloud((pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr )Cloud),coefficients(new pcl::ModelCoefficients)
{
    size.width = WIDTH;
    size.height = HEIGHT;
    frameRGB = cvCreateImage(size,IPL_DEPTH_8U,3);
    frameYUV =cvCreateImage(size,IPL_DEPTH_8U,3);
    frameU =cvCreateImage(size,IPL_DEPTH_8U,1);
    frameCannyU =cvCreateImage(size,IPL_DEPTH_8U,1);
    frameVerticalSobel=cvCreateImage(size,IPL_DEPTH_8U,1);
    frameHorizontalSobel=cvCreateImage(size,IPL_DEPTH_8U,1);
    edgeThreshold = 0.1;
}

Calibrator::Calibrator():coefficients(new pcl::ModelCoefficients)
{
    size.width = WIDTH;
    size.height = HEIGHT;
    frameRGB = cvCreateImage(size,IPL_DEPTH_8U,3);
    frameYUV =cvCreateImage(size,IPL_DEPTH_8U,3);
    frameU =cvCreateImage(size,IPL_DEPTH_8U,1);
    frameVerticalSobel=cvCreateImage(size,IPL_DEPTH_8U,1);
    frameHorizontalSobel=cvCreateImage(size,IPL_DEPTH_8U,1);
    frameCannyU =cvCreateImage(size,IPL_DEPTH_8U,1);
    edgeThreshold = 0.1;
}
Calibrator::~Calibrator() {
}

bool Calibrator::Calibrate()
{
    Landmarks.clear();
    lines.clear();
    Lines2D.clear();
    Intersection.clear();
    //  std::vector<pcl::PointIndices > Clusters;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr extractedInliers (new pcl::PointCloud<pcl::PointXYZRGBA>),
            Outliers(new pcl::PointCloud<pcl::PointXYZRGBA>);

    extractSurface(Cloud,coefficients,extractedInliers,Outliers);
    Accessories::XYZRGB2IPL(Cloud,frameRGB);
    extractLines(frameRGB,lines);
    FindLandmark(Landmarks);
    //  cout << " landmarks : " << Landmarks.size()<<endl;
    CGAL2PCLLines(lines,visualLines);

    //for(int i = 0; i < lines.size(); i++ )
    //{
//      DrawLandmarkOnImage(frameRGB);
    //	}

    //  if(Accessories::DrawImage(frameRGB) != -1)
    //    {
    //      Plane2RotationMatrix(coefficients,transformationMatrix,landmark);
    //      return true;
    //    }
    //  return false;
}

void Calibrator::DrawLandmarkOnImage(IplImage* frameRGB)
{
    if(Landmarks.size())
    {
        CvPoint p1,p2,p3,p4;
        p1.x = landmark.getFirstLine2D().source().x();
        p1.y = landmark.getFirstLine2D().source().y();
        p2.x = landmark.getFirstLine2D().target().x();
        p2.y = landmark.getFirstLine2D().target().y();
        p3.x = landmark.getLastLine2D().source().x();
        p3.y = landmark.getLastLine2D().source().y();
        p4.x = landmark.getLastLine2D().target().x();
        p4.y = landmark.getLastLine2D().target().y();
        //int r = i*50%255;
        //int g = 50+i*50%255;
        //int b = 100+i*50%255;
        cvLine( frameRGB, p1, p2, CV_RGB(0,0,255), 5, 1 );
        cvLine( frameRGB, p3, p4, CV_RGB(0,0,255), 5, 1 );
        double ix=landmark.getIntersection2D().x(),iy = landmark.getIntersection2D().y();
        cvCircle(frameRGB, cvPoint(ix,iy),5,CV_RGB(0,0,255),3 );
    }
    for(int i=0;i<Lines2D.size();i++)
    {
        CvPoint p1,p2,p3,p4;
        p1.x = Lines2D[i].source().x();
        p1.y = Lines2D[i].source().y();
        p2.x = Lines2D[i].target().x();
        p2.y = Lines2D[i].target().y();
        p3.x = Lines2D[i].source().x();
        p3.y = Lines2D[i].source().y();
        p4.x = Lines2D[i].target().x();
        p4.y = Lines2D[i].target().y();
        //int r = i*50%255;
        //int g = 50+i*50%255;
        //int b = 100+i*50%255;
        cvLine( frameRGB, p1, p2, CV_RGB(0,0,255), 1, 1 );
        cvLine( frameRGB, p3, p4, CV_RGB(0,0,255), 1, 1 );
    }
//    for(int i=0;i<frameRGB->width*frameRGB->height*3;i+=3)
//    {
//        frameRGB->imageData[i] = (frameCannyU->imageData[i/3])%255;
//        frameRGB->imageData[i+1] = (frameCannyU->imageData[i/3])%255;
//        frameRGB->imageData[i+2] = (frameCannyU->imageData[i/3])%255;
//    }
}

void Calibrator::SetCloud(const PointCloud<PointXYZRGBA>::ConstPtr &Cloud)
{
    this->Cloud = Cloud;
}
ModelCoefficients::Ptr Calibrator::getPlane()
{
    return coefficients;
}

const Landmark& Calibrator::getLandmark()
{
    return landmark;
}
CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& Calibrator::getTransformationMatrix()
{
    return transformationMatrix;
}

void Calibrator::Plane2RotationMatrix(const pcl::ModelCoefficients::Ptr coefficients,CGAL::Aff_transformation_3<CGAL::Cartesian<double> > &rotation,const Landmark& landmark)
{

    Plane3D plane(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);

    //turning normal vector into rotation matrix
    //http://math.stackexchange.com/questions/61547/rotation-of-a-vector-distribution-to-align-with-a-normal-vector
    Point3D p0(0,0,0),p1(0,0,1),p2(1,0,1);
    Point3D P0 = plane.projection(p0);
    Point3D P1 = plane.projection(p1);
    Point3D P2 = plane.projection(p2);
    Vector3D normalVector = normal(P0,P2,P1);

    double Nx=normalVector[0]/sqrt(normalVector.squared_length()),Ny=normalVector[1]/sqrt(normalVector.squared_length()),Nz=normalVector[2]/sqrt(normalVector.squared_length());
    //	Eigen::Matrix3f m1(3,3),m2(3,3),rotationMatrix(3,3);

    //	m1(0,0) = Nz; 		m1(0,1) = 0; 		m1(0,2) = Nx;
    //	m1(1,0) = 0; 		m1(1,1) = Nz; 		m1(1,2) = Ny;
    //	m1(2,0) = -Nx; 		m1(2,1) = -Ny; 		m1(2,2) = Nz;
    //
    //	m2(0,0) = Ny*Ny; 	m2(0,1) = -Nx*Ny; 	m2(0,2) = 0;
    //	m2(1,0) = -Nx*Ny; 	m2(1,1) = Nx*Nx; 	m2(1,2) = 0;
    //	m2(2,0) = 0; 		m2(2,1) = 0; 		m2(2,2) = 0;
    //
    //	rotationMatrix = m1 + (1/(1+Nz)) * m2;
    //	rotation = CGAL::Aff_transformation_3<CGAL::Cartesian<double> >
    //			(rotationMatrix(0,0),rotationMatrix(0,1),rotationMatrix(0,2),
    //			 rotationMatrix(1,0),rotationMatrix(1,1),rotationMatrix(1,2),
    //			 rotationMatrix(2,0),rotationMatrix(2,1),rotationMatrix(2,2));

    Vector3D vec,rightVec,leftVec; //left of the camera or right
    Segment3D firstVec = landmark.getFirstLine3D(),secondVec = landmark.getLastLine3D();
    //finding the line with most right point. this is how we say the line is horizontal
    if((firstVec.source().x() > secondVec.source().x() && firstVec.source().x() > secondVec.target().x()) || (firstVec.target().x() > secondVec.source().x() && firstVec.target().x() > secondVec.target().x()))
    {
        rightVec = firstVec.to_vector();
        leftVec = secondVec.to_vector();
        if(secondVec.source().x() < secondVec.target().x())
            leftVec = Segment3D(secondVec.target(),secondVec.source()).to_vector();//just in this case (middle line)
    }
    else
    {
        rightVec = secondVec.to_vector();
        leftVec = firstVec.to_vector();
        if(firstVec.source().x() < firstVec.target().x())
            leftVec = Segment3D(firstVec.target(),firstVec.source()).to_vector();//just in this case (middle line)
    }

    if(Position == TOP_LEFT || Position == BOTTOM_RIGHT)
        vec = rightVec;
    else if(Position == TOP_RIGHT || Position == BOTTOM_LEFT || Position == TOP_MIDDLE)
        vec = leftVec;

    double landmarkX,landmarkY;
    if(Position == TOP_LEFT)
    {
        landmarkX = TOP_LEFT_L_X;
        landmarkY = TOP_LEFT_L_Y;
    }
    else if(Position == BOTTOM_RIGHT)
    {
        landmarkX = BOTTOM_RIGHT_L_X;
        landmarkY = BOTTOM_RIGHT_L_Y;
    }
    else if(Position == TOP_RIGHT)
    {
        landmarkX = TOP_RIGHT_L_X;
        landmarkY = TOP_RIGHT_L_Y;
    }
    else if(Position == BOTTOM_LEFT)
    {
        landmarkX = BOTTOM_LEFT_L_X;
        landmarkY = BOTTOM_LEFT_L_Y;
    }
    else if(Position == TOP_MIDDLE)
    {
        landmarkX = TOP_MIDDLE_T_X;
        landmarkY = TOP_MIDDLE_T_Y;
    }
    double x = vec.x()/sqrt(vec.squared_length());
    double y = vec.y()/sqrt(vec.squared_length());
    double z = vec.z()/sqrt(vec.squared_length());
    Eigen::Vector3f vec1(x,y,z),vec2(Nx,Ny,Nz);
    Eigen::Vector3f vec3 = vec2.cross(vec1);
    //  if(Position == TOP_MIDDLE)
    //      vec3 = vec1.cross(vec2);
    //	cout << " length of vectors : " << sqrt(Nx*Nx+Ny*Ny+Nz*Nz) << " , " << sqrt(x*x+y*y+z*z) << " , " <<sqrt(vec3(0)*vec3(0)+vec3(1)*vec3(1)+vec3(2)*vec3(2)) << endl;
    //Vec1 = vec1;
    //Vec2 = vec2;
    //Vec3 = vec3;

    rotation = CGAL::Aff_transformation_3<CGAL::Cartesian<double> >
            (x,vec3(0),Nx,
             y,vec3(1),Ny,
             z,vec3(2),Nz);

    //	Eigen::Matrix3f rotateZ
    //	(vec3(0),x,Nx,
    //			vec3(1),y,Ny,
    //			vec3(2),z,Nz);
    //	vec1 = vec1 * rotateZ,vec2 = vec2 * rotateZ, vec3 = vec3 * rotateZ;
    //	Eigen::Matrix3f rotateX
    //	(vec1(0),vec2(0),vec3(0),
    //	 vec1(1),vec2(1),vec3(1),
    //	 vec1(2),vec2(2),vec3(2));
    rotation = rotation.inverse();
    Point3D rotatedPoint = landmark.getIntersection3D().transform(rotation);
    cout << " landmark point : " << rotatedPoint << endl;
    rotation = CGAL::Aff_transformation_3<CGAL::Cartesian<double> >
            (rotation.m(0,0),rotation.m(0,1),rotation.m(0,2),landmarkX - rotatedPoint.x(),
             rotation.m(1,0),rotation.m(1,1),rotation.m(1,2),landmarkY - rotatedPoint.y(),
             rotation.m(2,0),rotation.m(2,1),rotation.m(2,2),-rotatedPoint.z());

}
void Calibrator::setThreshold(float value)
{
    edgeThreshold = value;
}

void Calibrator::extractIndicesFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const boost::shared_ptr<const vector<int> >& Indices,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& extractedCloud)
{
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract(true);
    extract.setInputCloud (Cloud);
    extract.setIndices (Indices);
    extract.setNegative (false);
    extract.filter (*extractedCloud);
}

void Calibrator::extractIndicesFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& Cloud,const vector<pcl::PointIndices>& Indices,int n,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& extractedCloud)
{
    std::vector<pcl::PointIndices>::const_iterator it = Indices.begin()+n;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        extractedCloud->points.push_back (Cloud->points[*pit]);
    extractedCloud->width = extractedCloud->points.size ();
    extractedCloud->height = 1;
    extractedCloud->is_dense = true;
}
double Calibrator::LandmarkRate(Landmark &landmark)
{
    return sqrt(CGAL::squared_distance(landmark.getIntersection3D(),Point3D(0,0,0)));

}
void Calibrator::DrawLandmark(pcl::visualization::PCLVisualizer& viewer,const Landmark& landmark,double r,double g,double b)
{
    Segment3D s1 = landmark.getFirstLine3D();
    Segment3D s2 = landmark.getLastLine3D();

    double xa1=s1.source().x(),ya1=s1.source().y(),za1 = s1.source().z();
    double xa2=s1.target().x(),ya2=s1.target().y(),za2 = s1.target().z();
    double xb1=s2.source().x(),yb1=s2.source().y(),zb1 = s2.source().z();
    double xb2=s2.target().x(),yb2=s2.target().y(),zb2 = s2.target().z();

    pcl::PointXYZ p1a(xa1,ya1,za1),p1b(xa2,ya2,za2);
    pcl::PointXYZ p2a(xb1,yb1,zb1),p2b(xb2,yb2,zb2);
    static int i=0;
    i++;
    string l("line");
    char a = i*2;
    l = l + a;
    viewer.addLine (p1a,p1b,l);
    viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, l);
    a+=1;
    l = l + a ;
    viewer.addLine (p2a,p2b,l);
    viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, l);

}
void Calibrator::Debug3D (pcl::visualization::PCLVisualizer& viewer)
{
    //viewer.setPenWidth(3);
    //	for(int i=0;i<lines.size();i++)
    //	{
    //		Eigen::Vector4f p1,p2;
    //
    //		p1[0] = lines[i].source().x();
    //		p1[1] = lines[i].source().y();
    //		p1[2] = lines[i].source().z();
    //		p2[0] = lines[i].target().x();
    //		p2[1] = lines[i].target().y();
    //		p2[2] = lines[i].target().z();
    //		string l("line");
    //		char a = i;
    //		l = l + a;
    //		pcl::PointXYZ pa(p1[0],p1[1],p1[2]),pb(p2[0],p2[1],p2[2]);
    //
    //		viewer.addLine (pa,pb,l);
    //	}
    for(int i = 0 ; i < Landmarks.size(); i++)
        DrawLandmark(viewer,Landmarks[i],0,0,1);

    DrawLandmark(viewer,landmark,1,0,0);
    Plane3D rotPlane(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
    rotPlane = rotPlane.transform(transformationMatrix);
    pcl::ModelCoefficients model;
    model.values.resize(4);
    model.values[0] = rotPlane.a();
    model.values[1] = rotPlane.b();
    model.values[2] = rotPlane.c();
    model.values[3] = rotPlane.d();
    viewer.addPlane(model);

    Point3D p0(0,0,0),p1(0,0,1),p2(1,0,0);
    Plane3D pt(p0,p1,p2);
    Plane3D pResult = pt.transform(transformationMatrix);
    //	double tmpx = pResult.x(),tmpy = pResult.y(),tmpz = pResult.z();
    double tmpa = pResult.a(),tmpb = pResult.b(),tmpc = pResult.c(),tmpd = pResult.d();
    double a = pt.a(),b = pt.b(),c = pt.c(),d = pt.d();
    pcl::ModelCoefficients rotatedPlane;
    pcl::ModelCoefficients plane;
    rotatedPlane.values.resize(4);
    rotatedPlane.values[0] = tmpa;
    rotatedPlane.values[1] = tmpb;
    rotatedPlane.values[2] = tmpc;
    rotatedPlane.values[3] = tmpd;

    plane.values.resize(4);
    plane.values[0] = a;
    plane.values[1] = b;
    plane.values[2] = c;
    plane.values[3] = d;

    //	pcl::PointXYZ p(3,3,0);
    //	pcl::PointXYZ presult(tmpx,tmpy,tmpz);

    Point3D rotatedPoint = Intersection[0].transform(transformationMatrix);
    double xx= rotatedPoint.x();
    double yy= rotatedPoint.y();
    double zz= rotatedPoint.z();
    pcl::PointXYZ presult(xx,yy,zz);
    viewer.addSphere(presult,0.2,1,0,0,"sphere");
    //	viewer.addSphere(presult,0.3,0,1,0,"sphere2");
    //	viewer.addPlane(plane,"plane1");
    //	viewer.addPlane(rotatedPlane,"plane2");
    //	pcl::PointXYZ P1(Vec1(0)*3,Vec1(1)*3,Vec1(2)*3),P2(Vec2(0)*3,Vec2(1)*3,Vec2(2)*3),P3(Vec3(0)*3,Vec3(1)*3,Vec3(2)*3),P0(0,0,0);
    //	viewer.addArrow(P1,P0,1,0,0,"arrow1");
    //	viewer.addArrow(P2,P0,0,1,0,"arrosdw2");
    //	viewer.addArrow(P3,P0,0,0,1,"arrowsd3");

}



void Calibrator::DebugCostumImage(IplImage* frameRGB)
{
    string winName("Display window");
    char s = '0';
    winName = winName + s;
    cvNamedWindow( winName.data(), CV_WINDOW_AUTOSIZE );
    cvShowImage( winName.data(), frameRGB );
    cvWaitKey(0);


}


/* @brief extracts one surface out of the cloud points
 * @param Data: the raw cloud point with surface within
 * @param Surface: the surface points
 * @param Outliers: any point except surface
 */
void Calibrator::extractSurface(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Data,pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Surface, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Outliers)
{
    //	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    double tim = Accessories::get_time();
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> vg;
    //  cout << "time 1 " << Accessories::get_time() - tim << endl;
    tim  = Accessories::get_time();
    vg.setInputCloud(Data);
    //  cout << "time 2 " << Accessories::get_time() - tim << endl;
    tim  = Accessories::get_time();
    vg.setLeafSize (0.05f, 0.05f, 0.05f);
    //  cout << "time 3 " << Accessories::get_time() - tim << endl;
    tim  = Accessories::get_time();
    vg.filter (*Outliers);
    //  cout << "time 4 " << Accessories::get_time() - tim << endl;
    tim  = Accessories::get_time();
    //  std::cout << "PointCloud after filtering has: " << Outliers->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (30);
    seg.setDistanceThreshold (0.03);
    int i=0, nr_points = (int) Outliers->points.size ();
    //while (Outliers->points.size () > 0.3 * nr_points)
    //	{
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (Outliers);
    //  cout << "time 5 " << Accessories::get_time() - tim << endl;
    //  tim  = Accessories::get_time();

    seg.segment (*inliers, *coefficients);
    //  cout << "time 6 " << Accessories::get_time() - tim << endl;
    //  tim  = Accessories::get_time();

    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
        //break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (Outliers);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*Surface);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*Outliers);
    //		*Outliers = *cloud_f;
    //	}

}

void Calibrator::UEdge(const IplImage& frameU,int Threshold,IplImage& edgeImage)
{
    const int step = 1;
    int currentU, binaryI, yuvI, previousU, rawI;
    for(int x=0;x < frameU.width;x+=step)
    {
        rawI = x*frameU.height;
        previousU = frameU.imageData[rawI];
        for(int y = 1; y < frameU.height;y+=step)
        {
            binaryI = x*frameU.height+y;
            currentU = frameU.imageData[binaryI];
            edgeImage.imageData[binaryI] = int(currentU - previousU > Threshold) * 255; // green's U value is much less than white's
            previousU = currentU;
        }
    }
}

void Calibrator::extractLines(IplImage* frameRGB,vector<Segment3D> &Lines)
{
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;

    //	IplImage *frameY =cvCreateImage(size,IPL_DEPTH_8U,1);
    //	IplImage *frameCannyY =cvCreateImage(size,IPL_DEPTH_8U,1);
    //  IplImage *frameCannyAnd =cvCreateImage(size,IPL_DEPTH_8U,1);

    cvCvtColor(frameRGB,frameYUV,CV_RGB2YUV);
    for(int x = 0;x < size.width;x++)
        for(int y = 0;y < size.height;y++)
        {
            int rgbI = y*size.width*3 + x*3;
            int grayI = y*size.width + x;
            //			frameY->imageData[grayI] = frameYUV->imageData[rgbI];
            frameU->imageData[grayI] = frameYUV->imageData[rgbI+1];
            //			frameU->imageData[grayI] = frameRGB->imageData[rgbI+1];
        }

    cvCanny( frameU, frameCannyU, 255*edgeThreshold, edgeThreshold*255*3, 3 );
//    cvSobel(frameCannyU,frameHorizontalSobel,1,0);
    cvSobel(frameCannyU,frameVerticalSobel,0,1);

    //	UEdge(*frameU,30,*frameCannyU);
    //	Accessories::DrawImage(frameCannyU);
    lines = cvHoughLines2(frameCannyU,storage,CV_HOUGH_PROBABILISTIC,1,(CV_PI/180.0)*0.3,80,20,10);

    //  cout << "lines number : " << lines->total << endl;
    for(int i = 0; i < lines->total; i++ )
    {
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);


        //cvLine( frameRGB, line[0],line[1], CV_RGB(255,0,0), 1, 1 );
        Segment2D l(Point2D(line[0].x,line[0].y),Point2D(line[1].x,line[1].y));
        Segment3D Line;
        bool lineIsValid= find3DLineFor2D(l,Cloud,Line);
        if((squared_distance(Line.source(),Point3D(0,0,0)) > 3*3 && squared_distance(Line.target(),Point3D(0,0,0)) > 3*3) || !lineIsValid || Line.target().z() < CAM_MIN_HEIGHT|| Line.source().z() < CAM_MIN_HEIGHT)
            continue;
        bool merged = false;
        for(int i = 0;i < Lines.size();i++)
            if(Lines2D[i].isCollinear(l,3))
            {
                Lines[i].mergeWithCollinear(Line);
                Lines2D[i].mergeWithCollinear(l);
                merged = true;
                break;
            }
        if(!merged)
        {
            Lines.push_back(Line);
            Lines2D.push_back(l);
        }

    }

    for(int i=0; i<Lines2D.size();)
    {
        bool merged = false;
        for(int j=i+1;j<Lines2D.size();)
            if(Lines[i].isCollinear(Lines[j],0.1))
            {
                Lines[i].mergeWithParallel(Lines[j]);
                Lines2D[i].mergeWithParallel(Lines2D[j]);
                Lines.erase(Lines.begin()+j);
                Lines2D.erase(Lines2D.begin()+j);
                merged = true;
            }
            else
                j++;

        if(!merged)
            i++;
    }

    //  cout << " lines number after filtering : " << Lines.size() << endl;
    //	for(int i=0;i<Lines2D.size();i++)
    //	{
    //		CvPoint p1;
    //		p1.x = Lines2D[i].source().x();
    //		p1.y = Lines2D[i].source().y();
    //		CvPoint p2;
    //		p2.x = Lines2D[i].target().x();
    //		p2.y = Lines2D[i].target().y();
    //		cvLine( frameRGB, p1,p2, CV_RGB(255,0,0), 1, 1 );
    //	}
    //	Accessories::DrawImage(frameRGB);
    //  cout << " num lines after : " << Lines.size() << endl;

}
const vector<Segment3D>& Calibrator::get3DLines()
{
    return lines;
}

void Calibrator::FindLandmark(vector<Landmark > &Landmarks)
{
    double Threshold = 0.2;
    double minDist = 10000;
    Point3D p;
    Point2D p2d;
    for(int i=0;i<lines.size();i++)
        for(int j=i+1;j<lines.size();j++)
        {
            if(lines[i].Intersection(lines[j],Plane3D(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]),p,p2d,Threshold))
            {
                Landmark L (lines[i],lines[j],p,p2d);
                L.setFirstLine2D(Lines2D[i]);
                L.setLastLine2D(Lines2D[j]);
                Landmarks.push_back(L);
                Intersection.push_back(p);
                //            cout << " landmark dist : " << LandmarkRate(L)  << endl;
                if(LandmarkRate(L) < minDist)
                {
                    minDist = LandmarkRate(L);
                    landmark = L;
                    landmark.setIsOn(true);
                }
            }
        }
    //  cout << "intersection : " << landmark.getIntersection3D() << endl;
    //  cout << "Landmark first Line : " << landmark.getFirstLine3D().source() << " ; " << landmark.getFirstLine3D().target() << endl;
    //  cout << "Landmark last Line : " << landmark.getLastLine3D().source() << " ; " << landmark.getLastLine3D().target() << endl;
    //  cout << "Landmarks extracted : " << Landmarks.size() << endl;

}
void Calibrator::CGAL2PCLLines(const vector<Segment3D> &Lines,vector<vtkSmartPointer<vtkDataSet> > &visualLines)
{
    for(int i=0;i<Lines.size();i++)
    {
        Eigen::Vector4f p1,p2;
        p1[0] = Lines[i].source().x();
        p1[1] = Lines[i].source().y();
        p1[2] = Lines[i].source().z();
        p2[0] = Lines[i].target().x();
        p2[1] = Lines[i].target().y();
        p2[2] = Lines[i].target().z();
        //		pcl::PointXYZ p1(x1,y1,z1),p2(x2,y2,z2);
        visualLines.push_back(pcl::visualization::createLine(p1,p2));
    }
}

bool Calibrator::find3DLineFor2D(const Segment2D& Segment,const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud,Segment3D& seg3D)
{
     Point3D source(100,100,100),target(101,101,101);
//#define DONT_USE_RANSAC
#ifdef DONT_USE_RANSAC
      int sign = Segment.source().x() < Segment.target().x() ? 1 : -1;
      ExactLine2D Line = Segment.supporting_line();
      bool NanRemains = true;

      for(int x = Segment.source().x() ; x != Segment.target().x();x += sign)
        {

          double y = Line.y_at_x(x);
          if(x >Cloud->width || y>Cloud->height)
            continue;
          //		int i = y*Cloud->width + x;
          pcl::PointXYZRGBA p = Cloud->at(x,y);
          if(!isnan(!p.x) && !isnan(p.y) && !isnan(p.z))
            {
              source = Point3D(p.x,p.y,p.z);
              NanRemains = false;
              break;
            }
        }

      for(int x = Segment.target().x() ; x != Segment.source().x();x -= sign)
        {

          double y = Line.y_at_x(x);
          if(x >Cloud->width || y>Cloud->height)
            continue;
          //		int i = y*Cloud->width + x;
          pcl::PointXYZRGBA p = Cloud->at(x,y);
          if(!isnan(!p.x) && !isnan(p.y) && !isnan(p.z))
            {
              target = Point3D(p.x,p.y,p.z);
              NanRemains = false;
              break;

            }
        }
      seg3D = Segment3D(source,target);
      return !NanRemains;

#else
    int sign = Segment.source().x() < Segment.target().x() ? 1 : -1;
    ExactLine2D Line = Segment.supporting_line();

    bool NanRemains = true;
    vector<int> indices;
    for(int x = Segment.source().x() ; x != Segment.target().x();x += sign)
    {

        double y = Line.y_at_x(x);
        if(x >Cloud->width || y>Cloud->height)
            continue;
        int i = y*Cloud->width + x;
        pcl::PointXYZRGBA p = Cloud->at(i);
        if(!isnan(!p.x) && !isnan(p.y) && !isnan(p.z))
        {
            indices.push_back(i);
            NanRemains = false;
        }
    }
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cl(new pcl::PointCloud<pcl::PointXYZRGBA>(*Cloud,indices));
    if(indices.size() < 5)
        return false;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr lineCloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*Cloud,indices));
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> vg;
    ModelCoefficients lineCoefficients;

//    vg.setInputCloud();
//    vg.setLeafSize (0.05f, 0.05f, 0.05f);
//    vg.filter (*lineCloud);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    pcl::PointIndices LineInliers;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (30);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (lineCloud);

    seg.segment(LineInliers, lineCoefficients);
    if(!lineCoefficients.values.size())
        return false;
    cout << " line coef : " << lineCoefficients.values.size() << endl;
    ExactLine3D line3D(Point3D(lineCoefficients.values[0],
                       lineCoefficients.values[1],
            lineCoefficients.values[2]),
            Direction_3<CGAL::Cartesian<double> >(lineCoefficients.values[3],lineCoefficients.values[4],lineCoefficients.values[5]));
    double minDistSq = 0.05*0.05;
    for(int x = Segment.source().x() ; x != Segment.target().x();x += sign)
    {
        double y = Line.y_at_x(x);
        if(x >Cloud->width || y>Cloud->height)
            continue;
        int i = y*Cloud->width + x;
        pcl::PointXYZRGBA p = Cloud->at(x,y);
        if(!isnan(!p.x) && !isnan(p.y) && !isnan(p.z) /*&& sqrt(CGAL::squared_distance(line3D,Point3D(p.x,p.y,p.z))) < 0.05*/)
        {
            source = Point3D(p.x,p.y,p.z);
            break;
        }
    }
    for(int x = Segment.target().x() ; x != Segment.source().x();x -= sign)
    {

        double y = Line.y_at_x(x);
        if(x >Cloud->width || y>Cloud->height)
            continue;
        int i = y*Cloud->width + x;
        pcl::PointXYZRGBA p = Cloud->at(x,y);
        if(!isnan(!p.x) && !isnan(p.y) && !isnan(p.z) /*&& sqrt(CGAL::squared_distance(line3D,Point3D(p.x,p.y,p.z))) < 0.05*/)
        {
            target = Point3D(p.x,p.y,p.z);
            break;
        }
    }
    CGAL::Object obj1 = CGAL::intersection(ExactLine3D(Point3D(0,0,0),source),Plane3D(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]));
    CGAL::Object obj2 = CGAL::intersection(ExactLine3D(Point3D(0,0,0),target),Plane3D(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]));
    const Point3D* p1 = CGAL::object_cast<Point3D > (&obj1);
    const Point3D* p2 = CGAL::object_cast<Point3D > (&obj2);
    seg3D = Segment3D(*p1,*p2);

    return !NanRemains;
#endif
//    for(int i=0;i< LineInliers->indices.size();i++)
//    {
//        if (cl->at(i).x > maxX)
//            maxP = cl->at(i);
//        if (cl->at(i).x < minX)
//            minP = cl->at(i);
//    }
//    seg3D = Segment3D(Point3D(minP.x,minP.y,minP.z),Point3D(maxP.x,maxP.y,maxP.z));
//    seg3D = Segment3D(Point3D(1,2,3),Point3D(4,5,6));

//    if (inliers->indices.size () == 0)
//    {
//        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//        return;
//        //break;
//    }

    //  // Extract the planar inliers from the input cloud
    //  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    //  extract.setInputCloud (Outliers);
    //  extract.setIndices (inliers);
    //  extract.setNegative (false);

    //  // Get the points associated with the planar surface
    //  extract.filter (*Surface);

    //  // Remove the planar inliers, extract the rest
    //  extract.setNegative (true);
    //  extract.filter (*Outliers);
}
void Calibrator::SetPosition(CAMERAPOSITION pos)
{
    Position = pos;
}
void Calibrator::calibrationAccepted()
{
    Plane2RotationMatrix(coefficients,transformationMatrix,landmark);
}
