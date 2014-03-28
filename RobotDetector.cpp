/*
 * RobotDetector.cpp
 *
 *  Created on: Jun 28, 2013
 *      Author: mohammad
 */

#include "RobotDetector.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include "Calibrator.h"
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include "Accessories.h"
#include "Defines.h"
#include "ColorModelConversions.h"

using namespace pcl;
using namespace cv;

RobotDetector::RobotDetector(const PointCloud<PointXYZRGBA>::ConstPtr &Cloud):
    Cloud((PointCloud<pcl::PointXYZRGBA>::ConstPtr)Cloud),RobotCloud(),
    upperPlane()
{
    double y = -cos(DEG2RAD(FOV_V/2.0));
    double z = -sin(DEG2RAD(FOV_V/2.0));
    Vector3D v(Point3D(0,0,0),Point3D(0,y/(y+z),z/(y+z)));
    upperPlane = Plane3D(Point3D(0,0,0),v);

    y = cos(DEG2RAD(FOV_V/2));
    z = -sin(DEG2RAD(FOV_V/2));
    v= Vector3D(Point3D(0,0,0),Point3D(0,y/(y+z),z/(y+z)));
    leftPlane = Plane3D(Point3D(0,0,0),v);

    double x = cos(DEG2RAD(FOV_H/2));
    z = -sin(DEG2RAD(FOV_H/2));
    v = Vector3D(Point3D(0,0,0),Point3D(x/(x+z),0,z/(x+z)));
    rightPlane = Plane3D(Point3D(0,0,0),v);

    x = -cos(DEG2RAD(FOV_H/2));
    z = -sin(DEG2RAD(FOV_H/2));
    v = Vector3D(Point3D(0,0,0),Point3D(x/(x+z),0,z/(x+z)));
    leftPlane = Plane3D(Point3D(0,0,0),v);

}

RobotDetector::RobotDetector()
{
    double y = -cos(DEG2RAD(FOV_V/2.0));
    double z = -sin(DEG2RAD(FOV_V)/2.0);
    Vector3D v(Point3D(0,0,0),Point3D(0,y/(y+z),z/(y+z)));
    upperPlane = Plane3D(Point3D(0,0,0),v);

    y = cos(DEG2RAD(FOV_V/2));
    z = -sin(DEG2RAD(FOV_V/2));
    v = Vector3D(Point3D(0,0,0),Point3D(0,y/(y+z),z/(y+z)));
    leftPlane = Plane3D(Point3D(0,0,0),v);

    double x = cos(DEG2RAD(FOV_H/2));
    z = -sin(DEG2RAD(FOV_H/2));
    v = Vector3D(Point3D(0,0,0),Point3D(x/(x+z),0,z/(x+z)));
    rightPlane = Plane3D(Point3D(0,0,0),v);

    x = -cos(DEG2RAD(FOV_H/2));
    z = -sin(DEG2RAD(FOV_H/2));
    v = Vector3D(Point3D(0,0,0),Point3D(x/(x+z),0,z/(x+z)));
    leftPlane = Plane3D(Point3D(0,0,0),v);
}
RobotDetector::~RobotDetector() {
}

Segment3D RobotDetector::getFOVLines()
{
    //    ExactLine3D left = CGAL::intersection(leftPlane,fieldPlane);
    //    ExactLine3D right = CGAL::intersection(rightPlane,fieldPlane);
    //    ExactLine3D bottom = CGAL::intersection(lowerPlane,fieldPlane);
    //    left = left.transform(transformationMatrix);
    //    right = right.transform(transformationMatrix);
    //    bottom = bottom.transform(transformationMatrix);
    //    ExactLine2D left = CGAL::intersection(leftPlane,fieldPlane);
    //    ExactLine3D right = CGAL::intersection(rightPlane,fieldPlane);
    //    ExactLine3D bottom = CGAL::intersection(lowerPlane,fieldPlane);

    //    CGAL::intersection(bottom,left);
}

void RobotDetector::Detect()
{
    clusters.clear();
    OutlierIndices.indices.clear();
    InlierIndices.indices.clear();

    //  PointCloud<PointXYZRGBA>::Ptr extractedInliers (new PointCloud<PointXYZRGBA>),
    //      Outliers(new PointCloud<PointXYZRGBA>),
    //      ShowCluster(new PointCloud<PointXYZRGBA>);


    RemoveFromSurface(Cloud,fieldPlane,0.12,InlierIndices, OutlierIndices);
    RobotCloud = pcl::PointCloud<PointXYZRGBA>::Ptr(new PointCloud<PointXYZRGBA>(*Cloud,OutlierIndices.indices));
    //    RobotCloud = Accessories::smoothPointCloud(RobotCloud,0.01,3);
//    BallFinder(Cloud, InlierIndices);
    int startI = clusters.size();

    PerformCluster(RobotCloud, clusters,ROBOT_CLUSTER_THRESHOLD,ROBOT_MIN_POINT_NUM,ROBOT_MAX_POINT_NUM);
//    cout << "number of balls : " << startI << " number of robots : " << clusters.size() - startI << endl;
    for(int i=startI;i<clusters.size();i++)
        clusters[i].Cloud = RobotCloud;

    RemoveNoise(RobotCloud,clusters);

    stickID(clusters);

    SelectClusterPoint();
    double tx,ty,tz,rx,ry,rz;
    Accessories::transformMat2Values(transformationMatrix,rx,ry,rz,tx,ty,tz);
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == RED_ROBOT || clusters[i].clusterID == BLUE_ROBOT|| clusters[i].clusterID == BLACK_ROBOT|| clusters[i].clusterID == WHITE_ROBOT )
            cout << clusters[i].clusterID << " , " << clusters[i].representativePointToField<< " , " <<rx << " "<<ry<< " " << rz<< " " << tx<< " " << ty<< " " << tz<<endl;

}
void RobotDetector::BallFinder(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const PointIndices& Inliers)
{
    PointIndices ballIndices;
    for(int i=0;i<Inliers.indices.size();i++)
        if(guessColor(Cloud->at(Inliers.indices[i]) ,BALL_FIELD) == ORANGE_BALL)
            ballIndices.indices.push_back(Inliers.indices[i]);
    pcl::PointCloud<PointXYZRGBA>::Ptr ballCloud(new PointCloud<PointXYZRGBA>(*Cloud,ballIndices.indices));
    PerformCluster(ballCloud,clusters,BALL_CLUSTER_THRESHOLD,BALL_MIN_POINT_NUM,BALL_MAX_POINT_NUM);
    for(int i=0;i<clusters.size();i++)
        clusters[i].Cloud = ballCloud;
}

pcl::PointIndices RobotDetector::GetRobotCloud()
{
    return OutlierIndices;
}
const pcl::PointCloud<PointXYZRGBA>::Ptr RobotDetector::getCluserPointCloud(int i)
{
    if(clusters.size())
        return pcl::PointCloud<PointXYZRGBA>::Ptr(new PointCloud<PointXYZRGBA>(*clusters[i].Cloud,clusters[i].pointIndices.indices));
    else
        return pcl::PointCloud<PointXYZRGBA>::Ptr(new PointCloud<PointXYZRGBA>());
}

pcl::ModelCoefficients::Ptr RobotDetector::getUpperPlane()
{
    pcl::ModelCoefficients::Ptr p(new pcl::ModelCoefficients);
    p->values.resize(4);
    //    cout << "UPPER : " <<upperPlane << endl;
    p->values[0] = upperPlane.a();
    p->values[1] = upperPlane.b();
    p->values[2] = upperPlane.c();
    p->values[3] = upperPlane.d();
    return p;
}
void RobotDetector::stickID(std::vector<Cluster >& clusters)
{
//     cout << " refresh " << endl;
    for(int i=0;i<clusters.size();i++)
    {
        Cluster& clust = clusters[i];
        if(clust.clusterGuess == REFEREE_GUESS)
        {
            clust.clusterID = REFEREE;
            continue;
        }
        ObjectGuess guess = clust.clusterGuess;

        for(int j =0;j<clust.pointIndices.indices.size();j++)
            clust.colors[guessColor(clust.Cloud->at(clust.pointIndices.indices[j]),guess)]++;

        int maxColor = 0;
        int maxID =0;
//        cout << " **** " << endl;
        for(int j=1;j<clust.colors.size();j++)
        {
//            cout << " color : " << clust.colors[j] << endl;
            if(clust.colors[j] > maxColor)
            {
                maxColor = clust.colors[j];
                maxID = j;
            }
        }
        ObjectId ID = (ObjectId)maxID;
        clust.clusterID = ID;
        if(clust.clusterGuess == ROBOT &&
          (clust.colors[RED_ROBOT] > MIN_COLOR_PIXELS || clust.colors[BLUE_ROBOT] > MIN_COLOR_PIXELS))
        {
            if(clust.colors[BLUE_ROBOT] > clust.colors[RED_ROBOT]  )
                clust.clusterID = BLUE_ROBOT;
            else
                clust.clusterID = RED_ROBOT;
        }


        //        cout <<"detected ID : " << ID << endl;
        //        if(ID != UNKNOWN_OBJECT && ID != GREEN_FIELD)
        //        cout << " cluster ID : ";
        //        if(ID == BLUE_ROBOT)
        //            cout << " BLUE ROBOT "<< endl;
        //        else if(ID == RED_ROBOT)
        //            cout << " RED_ROBOT "<< endl;
        //        else if(ID == ORANGE_BALL)
        //            cout << " ORANGE_BALL  "<< endl;
        //        else if(ID == REFEREE)
        //            cout << " REFEREE "<< endl;
        //        else if(ID == UNKNOWN_OBJECT)
        //            cout << " UNKNOWN_OBJECT "<< endl;

        //        if(ID != UNKNOWN_OBJECT && ID != GREEN_FIELD)
        //        for(int j=1;j<clust.colors.size();j++)
        //            cout << " num color : " << clust.colors[j] << endl;
    }
}

ObjectId RobotDetector::guessColor(const pcl::PointXYZRGBA& pix,ObjectGuess guess)
{
    int h;unsigned char s,l;
    Point3D p(pix.x,pix.y,pix.z);
    ColorModelConversions::fromRGBtoHSL(pix.r,pix.g,pix.b,h,s,l);
    if(!(s <= UPPER_S && s >= LOWER_S && l <= UPPER_L && l >= LOWER_L))
    {
//        return UNKNOWN_OBJECT;
        if(guess == ROBOT)
        {
            if(l >= UPPER_L)
                return WHITE_ROBOT;
            else if(l <= LOWER_L)
                return BLACK_ROBOT;
        }
        else
            return UNKNOWN_OBJECT;
    }
    if(guess == ROBOT)
    {
        if(CGAL::squared_distance(p,fieldPlane) < TORSO_MIN_HEIGHT*TORSO_MIN_HEIGHT || CGAL::squared_distance(p,fieldPlane) > TORSO_MAX_HEIGHT*TORSO_MAX_HEIGHT)
            return UNKNOWN_OBJECT;
        double redDist = std::abs(Accessories::angularDiffDeg(h , RED_H));
        double blueDist =std::abs(Accessories::angularDiffDeg(h , BLUE_H));
        if(redDist < blueDist && redDist < HUE_THRESHOLD)
            return RED_ROBOT;
        else if(blueDist < redDist && blueDist < HUE_THRESHOLD)
            return BLUE_ROBOT;
    }
    else if(guess == BALL_FIELD)
    {
        if(std::abs(h - ORANGE_H) < HUE_THRESHOLD && s > 20 && l > 20 && l < 80)
            return ORANGE_BALL;
        else if(std::abs(h - GREEN_H) < HUE_THRESHOLD && s > 20 && l > 10 && l < 90)
            return GREEN_FIELD;
    }
    return UNKNOWN_OBJECT;
}

std::vector<pcl::PointXYZ> RobotDetector::GetBlueRobotsToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == BLUE_ROBOT)
            p.push_back(clusters[i].representativePointToField);
    return p;
}

std::vector<pcl::PointXYZ> RobotDetector::GetRedRobotsToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == RED_ROBOT)
            p.push_back(clusters[i].representativePointToField);
    return p;
}
std::vector<pcl::PointXYZ> RobotDetector::GetBlackRobotsToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == BLACK_ROBOT)
            p.push_back(clusters[i].representativePointToField);
    return p;
}

std::vector<pcl::PointXYZ> RobotDetector::GetWhiteRobotsToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == WHITE_ROBOT)
            p.push_back(clusters[i].representativePointToField);
    return p;
}

std::vector<pcl::PointXYZ> RobotDetector::GetBallToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == ORANGE_BALL)
            p.push_back(clusters[i].representativePointToField);
    return p;
}

std::vector<pcl::PointXYZ> RobotDetector::GetRefereeToField()
{
    std::vector<pcl::PointXYZ> p;
    for(int i=0;i<clusters.size();i++)
        if(clusters[i].clusterID == REFEREE)
            p.push_back(clusters[i].representativePointToField);
    return p;
}

void RobotDetector::SelectClusterPoint()
{
int counter=0;
    for(int i=0;i<clusters.size();i++)
    {
        Accessories::FindBoundingBox(clusters[i].Cloud,clusters[i].pointIndices,clusters[i].boundingBox.first,clusters[i].boundingBox.second);
        counter=0;
        double oneThirdBottom = (clusters[i].boundingBox.second.y - clusters[i].boundingBox.first.y)*2/3;
        oneThirdBottom +=clusters[i].boundingBox.first.y;
        pcl::PointXYZ p(0,0,0);

        for(int j=0;j < clusters[i].pointIndices.indices.size();j++)
        {
            pcl::PointXYZRGBA& currentpoint = clusters[i].Cloud->at(clusters[i].pointIndices.indices[j]);
            if(currentpoint.y > oneThirdBottom)
            {
            p.x += currentpoint.x;
            p.y += currentpoint.y;
            p.z += currentpoint.z;
            counter++;
            }
        }
        p.x /= counter;
        p.y /= counter;
        p.z /= counter;
//        p.x = (clusters[i].boundingBox.first.x + clusters[i].boundingBox.second.x)/2;
//        p.y = (clusters[i].boundingBox.first.y + clusters[i].boundingBox.second.y)/2;
//        p.z = (clusters[i].boundingBox.first.z + clusters[i].boundingBox.second.z)/2;
        clusters[i].representativePointToCam = p;
        Point3D pp(p.x,p.y,p.z);
        pp = pp.transform(transformationMatrix);
        clusters[i].representativePointToField = pcl::PointXYZ(pp.x(),pp.y(),pp.z());

    }
}
const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >& RobotDetector::getBoundingBoxes()
{
    boxes.clear();
    for(int i=0;i<clusters.size();i++)
        boxes.push_back(clusters[i].boundingBox);
    return boxes;
}

void RobotDetector::RemoveNoise(pcl::PointCloud<PointXYZRGBA>::Ptr RobotCloud,std::vector<Cluster>& clusters)
{
    double min;
    double max;
    PointXYZRGBA minPoint;
    PointXYZRGBA maxPoint;
    double maxSquaredDistToUpperPlane = 0.03;

    double maxBallHeight = 0.1,minBallHeight = 0;
//    cout << " robots before filtering : " << clusters.size() << endl;
    for(int i=0;i<clusters.size();)
    {
        min = 100000;
        max = -100000;

        for(int j=0;j < clusters[i].pointIndices.indices.size() ; j++)
        {
            pcl::PointXYZRGBA& currentpoint = clusters[i].Cloud->at(clusters[i].pointIndices.indices[j]);
            if(currentpoint.y < min)
            {
                min = currentpoint.y;
                minPoint = currentpoint;
            }
            if(currentpoint.y > max)
            {
                max = currentpoint.y;
                maxPoint = currentpoint;
            }
        }
        Point3D upperPoint2Cam(minPoint.x,minPoint.y,minPoint.z);
        Point3D upperPoint2Field(minPoint.x,minPoint.y,minPoint.z);
        Point3D cgalMax(maxPoint.x,maxPoint.y,maxPoint.z);
        upperPoint2Field = upperPoint2Field.transform(transformationMatrix);
        cgalMax = cgalMax.transform(transformationMatrix);

        double upperDist = Accessories::point2plane(upperPoint2Cam,upperPlane);

//        cout <<"robot pos : " << upperDist << endl;
//        if(cgalMax.z() > ON_THE_FIELD_THRESHOLD)
//        {
////              cout << "Removing a cluster because cgalMax.z() > ON_THE_FIELD_THRESHOLD i.e " << cgalMax.z() << " > " << ON_THE_FIELD_THRESHOLD << endl;
//            clusters.erase(clusters.begin() + i);
//        }
        if(abs(upperPoint2Field.x()) > In_THE_FIELD_X_THRESHOLD || abs(upperPoint2Field.y()) > In_THE_FIELD_Y_THRESHOLD)
        {
//            cout << "Removing a cluster because abs(upperPoint2Field.x()) > In_THE_FIELD_X_THRESHOLD || abs(upperPoint2Field.y()) > In_THE_FIELD_Y_THRESHOLD i.e " <<abs(upperPoint2Field.x()) <<" > " <<In_THE_FIELD_X_THRESHOLD << " || " << abs(upperPoint2Field.y()) << " > " <<  In_THE_FIELD_Y_THRESHOLD<< endl;
            clusters.erase(clusters.begin() + i);
        }
        else if(abs(upperPoint2Field.x()) < 1.5 && abs(upperPoint2Field.y()) > FIELD_WIDTH_M/2.0 - 0.2)
        {
//                        cout << "Removing a cluster because abs(upperPoint2Field.x()) > In_THE_FIELD_X_THRESHOLD && abs(upperPoint2Field.y()) > In_THE_FIELD_Y_THRESHOLD i.e " << endl;
            clusters.erase(clusters.begin() + i);
        }
        else if(upperPoint2Field.z() > MAX_ROBOT_HEIGHT_M )
        {
            clusters[i].clusterGuess = REFEREE_GUESS;
            i++;
        }
        else if(upperDist < maxSquaredDistToUpperPlane)
        {
            //            cout << " removed : " << endl;
            //            cout << "upperDist : " << upperDist << endl;
            //            cout << "upper : " << upperPoint2Field << endl;
            //            cout << "lower : " << cgalMax << endl;
//                        cout << "Removing a cluster because upperDist < maxSquaredDistToUpperPlan i.e " << upperDist << " < " << maxSquaredDistToUpperPlane << endl;
            clusters.erase(clusters.begin() + i);
        }
        else if(upperPoint2Field.z() < maxBallHeight && upperPoint2Field.z() > minBallHeight)
        {
            clusters[i].clusterGuess = BALL_FIELD;
            i++;
        }
        else if(upperPoint2Field.z() < MIN_ROBOT_HEIGHT_M)
        {
//            cout << "Removing a cluster because upperPoint2Field.z() < minRobotHeight i.e " << upperPoint2Field.z() << " < " << MIN_ROBOT_HEIGHT_M<< endl;
            clusters.erase(clusters.begin() + i);
        }
        else
        {
            clusters[i].clusterGuess = ROBOT;
            i++;
            //            cout << "upperDist : " << upperDist << endl;
            //            cout << "upper : " << upperPoint2Field << endl;
            //            cout << "lower : " << cgalMax << endl;
        }

    }
//        cout << " robots after filtering : " << clusters.size() << endl;
}

double RobotDetector::FindRotationFromBoundingBox(const Point3D &p1,const Point3D &p2)
{

    double length = p1.x() - p2.x();
    cout << "p1 : " << p1 << endl;
    cout << "p2 : " << p2 << endl;
    length = abs(length);
    cout << "LENGTH : " << length << endl;
    double ratio = float(length-ROBOT_DEPTH_M)/(ROBOT_WIDTH_M-ROBOT_DEPTH_M);

    ratio = abs(ratio) > 1 ? ratio/abs(ratio):ratio;
    return asin(ratio);
}

void RobotDetector::SetCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &Cloud)
{
    this->Cloud = Cloud;
}

void RobotDetector::setCalibrationConfigs(const pcl::ModelCoefficients::Ptr coefficients,const CGAL::Aff_transformation_3<CGAL::Cartesian<double> >& transformationMatrix)
{
    this->coefficients = coefficients;
    fieldPlane = Plane3D(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
    this->transformationMatrix = transformationMatrix;
    //    upperPlane = upperPlane.transform(transformationMatrix);
}
void RobotDetector::RemoveFromSurface(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const Plane3D& plane,double dist,PointIndices& Inliers, PointIndices& Outliers)
{
    double squaredDist = dist*dist;
    for(int x=0;x<Cloud->width;x++)
        for(int y=0;y<Cloud->height;y++)
        {
            PointXYZRGBA p = Cloud->at(x,y);
            if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
            {
                if(CGAL::squared_distance(plane,Point3D(p.x,p.y,p.z)) < squaredDist )
                    Inliers.indices.push_back(y*Cloud->width + x);
                else if(p.x*p.x + p.y*p.y + p.z*p.z < MAX_POINTCLOUD_DIST*MAX_POINTCLOUD_DIST)
                    Outliers.indices.push_back(y*Cloud->width + x);
            }
        }
}

void RobotDetector::PerformCluster(const PointCloud<PointXYZRGBA>::ConstPtr& RobotCloud, std::vector<Cluster >& clusters,float clusterThreshold,int minPoints,int maxPoints)
{
    std::vector<PointIndices > clustersIndices;
    //    cout << " robotcloud size : " << RobotCloud->size() << endl;
    if(RobotCloud->size() < minPoints)
        return;
    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZRGBA>::Ptr tree (new search::KdTree<PointXYZRGBA>);
    tree->setInputCloud (RobotCloud);

    EuclideanClusterExtraction<PointXYZRGBA> ec;
    ec.setClusterTolerance (clusterThreshold);
    ec.setMinClusterSize (minPoints);
    ec.setMaxClusterSize (maxPoints);
    ec.setSearchMethod (tree);
    ec.setInputCloud (RobotCloud);
    ec.extract (clustersIndices);

    for(int i=0;i<clustersIndices.size();i++)
        clusters.push_back(Cluster(clustersIndices[i]));
}

void RobotDetector::extractIndicesFromCloud(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const boost::shared_ptr<const vector<int> >& Indices,PointCloud<PointXYZRGBA>::Ptr& extractedCloud)
{
    ExtractIndices<PointXYZRGBA> extract(true);
    extract.setInputCloud (Cloud);
    extract.setIndices (Indices);
    extract.setNegative (false);
    extract.filter (*extractedCloud);

}

void RobotDetector::extractIndicesFromCloud(const PointCloud<PointXYZRGBA>::ConstPtr& Cloud,const vector<PointIndices>& Indices,int n,PointCloud<PointXYZRGBA>::Ptr& extractedCloud)
{
    std::vector<PointIndices>::const_iterator it = Indices.begin()+n;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        extractedCloud->points.push_back (Cloud->points[*pit]);
    extractedCloud->width = extractedCloud->points.size ();
    extractedCloud->height = 1;
    extractedCloud->is_dense = true;
}
