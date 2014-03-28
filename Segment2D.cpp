/*
 * Line3D.cpp
 *
 *  Created on: Jul 4, 2013
 *      Author: nao
 */

#include "Segment2D.h"
#include <math.h>
#include <pcl/pcl_base.h>
using std::cout;
using std::endl;
Segment3D::Segment3D(Point3D p1,Point3D p2):ExactSegment3D( p1, p2)
{

}

Segment3D::Segment3D():ExactSegment3D(Point3D(0,0,0),Point3D(0,0,0))
{}
Segment3D::~Segment3D() {
}

bool Segment3D::isCollinear(const Segment3D &otherLine, double Threshold) const
{
	ExactLine3D l = supporting_line();
	double d1 = squared_distance(l,otherLine.source());
	double d2 = squared_distance(l,otherLine.target());
	return sqrt(d1+d2)< Threshold;

}

void Segment3D::mergeWithCollinear(const Segment3D& otherLine)
{
	Point3D p[4];
	Point3D minX,minY,minZ,maxX,maxY,maxZ;
	double minXVal = 1000000000;
	double maxXVal = -1000000000;
	double minYVal = 1000000000;
	double maxYVal = -1000000000;
	double minZVal = 1000000000;
	double maxZVal = -1000000000;
	p[0] = otherLine.source();
	p[1] = otherLine.target();
	p[2] = this->source();
	p[3] = this->target();
	for(int i=0;i<4;i++)
	{
		if(p[i].x() < minXVal)
		{
			minX = p[i];
			minXVal = minX.x();
		}
		if(p[i].y() < minYVal)
		{
			minY = p[i];
			minYVal = minY.y();
		}
		if(p[i].z() < minZVal)
		{
			minZ = p[i];
			minZVal = minZ.y();
		}

		if(p[i].x() > maxXVal)
		{
			maxX = p[i];
			maxXVal = maxX.x();
		}
		if(p[i].y() > maxYVal)
		{
			maxY = p[i];
			maxYVal = maxY.y();
		}
        if(p[i].z() < minZVal)
        {
            minZ = p[i];
            minZVal = minZ.y();
        }
        if(p[i].z() > maxZVal)
        {
            maxZ = p[i];
            maxZVal = maxZ.y();
        }

	}

	if(direction().dx() < direction().dy() && direction().dz() < direction().dy())
		*this = Segment3D(minY,maxY);
	else if(direction().dy() < direction().dx() && direction().dz() < direction().dx())
		*this = Segment3D(minX,maxX);
	else
		*this = Segment3D(minZ,maxZ);
}
void Segment3D::mergeWithParallel(const Segment3D& otherLine)
{
//	if(source().x() > target().x() && otherLine.source().x() > otherLine.target().x())
//	{
		Segment3D p(Point3D((otherLine.source().x() + source().x())/2,(otherLine.source().y() + source().y())/2,(otherLine.source().z() + source().z())/2),
				Point3D((otherLine.target().x() + target().x())/2,(otherLine.target().y() + target().y())/2,(otherLine.target().z() + target().z())/2));
		*this = p;
//	}
//	else
//	{
//		Segment3D p(Point3D((otherLine.target().x() + source().x())/2,(otherLine.target().y() + source().y())/2,(otherLine.target().z() + source().z())/2),
//						Point3D((otherLine.source().x() + target().x())/2,(otherLine.source().y() + target().y())/2,(otherLine.source().z() + target().z())/2));
//		*this = p;
//	}

}

bool Segment3D::Intersection(const Segment3D &otherLine,const Plane3D& pl, Point3D& intersection, Point2D& intersection2D,double Threshold)
{
    Threshold = Threshold*Threshold;
//    ExactLine3D l1(pl.projection(source()),pl.projection(target()));
//    ExactLine3D l2(pl.projection(otherLine.source()),pl.projection(otherLine.target()));
    Point2D twoD1(pl.projection(source()).x(),pl.projection(source()).y());
    Point2D twoD2(pl.projection(target()).x(),pl.projection(target()).y());
    Point2D twoD3(pl.projection(otherLine.source()).x(),pl.projection(otherLine.source()).y());
    Point2D twoD4(pl.projection(otherLine.target()).x(),pl.projection(otherLine.target()).y());
    ExactLine2D l1(twoD1,
                   twoD2),
            l2(twoD3,
               twoD4);

	CGAL::Object obj = CGAL::intersection(l1,l2);
    const Point2D *p2 = CGAL::object_cast<Point2D >(&obj);
    if(!p2)
        return false;

    intersection2D = *p2;

    Point3D near1, near2;
    if(squared_distance(twoD1,*p2) < squared_distance(twoD2,*p2))
        near1 = pl.projection(source());
    else
        near1 = pl.projection(target());

    if(squared_distance(twoD3,*p2) < squared_distance(twoD4,*p2))
        near2 = pl.projection(otherLine.source());
    else
        near2 = pl.projection(otherLine.target());

    Point3D p3 = supporting_line().projection(Point3D(p2->x(),p2->y(),(near1.z()+near2.z())/2.0));

	double d1 = squared_distance(*this,p3);
	double d2 = squared_distance(otherLine,p3);
	double p2p11 = squared_distance(source(),p3);
	double p2p12 = squared_distance(target(),p3);
	double p2p21 = squared_distance(otherLine.source(),p3);
	double p2p22 = squared_distance(otherLine.target(),p3);
	bool intersectionOnBoth = (d1 < Threshold && d2 < Threshold) || (has_on(p3) && otherLine.has_on(p3));
	bool intersectionOnA = d1 < Threshold && (p2p11 < Threshold || p2p12 < Threshold);
	bool intersectionOnB = d2 < Threshold && (p2p21 < Threshold || p2p22 < Threshold);

	intersection = p3;
//    cout << "intersections : " << intersectionOnBoth << intersectionOnA<<intersectionOnB<<endl;
//    cout << "int point " << p3 << endl;
//    cout << " d2 : " << d2 << " , " << p2p21  << " , " << p2p22 <<endl;
//    cout << " source : " << otherLine.source() << " target : " << otherLine.target() << endl;
	if (intersectionOnBoth || (intersectionOnA && intersectionOnB))
		return true;
	else
        return false;

}

Segment2D::Segment2D(Point2D p1,Point2D p2):ExactSegment2D( p1, p2)
{

}
Segment2D::Segment2D():ExactSegment2D( Point2D(0,0),Point2D(0,0))
{}

bool Segment2D::isCollinear(const Segment2D &otherLine, double Threshold) const
{
	ExactLine2D l = supporting_line();
	double d1 = squared_distance(l,otherLine.source());
	double d2 = squared_distance(l,otherLine.target());
	return sqrt((d1+d2)/2)< Threshold;
}

void Segment2D::mergeWithCollinear(const Segment2D& otherLine)
{
	Point2D p[4];
	Point2D minX,minY,minZ,maxX,maxY,maxZ;
	double minXVal = 1000000000;
	double maxXVal = -1000000000;
	double minYVal = 1000000000;
	double maxYVal = -1000000000;
	p[0] = otherLine.source();
	p[1] = otherLine.target();
	p[2] = this->source();
	p[3] = this->target();
	for(int i=0;i<4;i++)
	{
		if(p[i].x() < minXVal)
		{
			minX = p[i];
			minXVal = minX.x();
		}
		if(p[i].y() < minYVal)
		{
			minY = p[i];
			minYVal = minY.y();
		}

		if(p[i].x() > maxXVal)
		{
			maxX = p[i];
			maxXVal = maxX.x();
		}
		if(p[i].y() > maxYVal)
		{
			maxY = p[i];
			maxYVal = maxY.y();
		}

	}

	if(direction().dx() < direction().dy())
		*this = Segment2D(minY,maxY);
	else
		*this = Segment2D(minX,maxX);
}

void Segment2D::mergeWithParallel(const Segment2D& otherLine)
{
//	if(source().x() > target().x() && otherLine.source().x() > otherLine.target().x() )
//	{
		Segment2D p(Point2D((otherLine.source().x() + source().x())/2,(otherLine.source().y() + source().y())/2),
				Point2D((otherLine.target().x() + target().x())/2,(otherLine.target().y() + target().y())/2));
		*this = p;
//	}
//	else
//	{
//		Segment2D p(Point2D((otherLine.target().x() + source().x())/2,(otherLine.target().y() + source().y())/2),
//						Point2D((otherLine.source().x() + target().x())/2,(otherLine.source().y() + target().y())/2));
//		*this = p;
//	}
}
