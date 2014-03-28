/*
 * Line3D.h
 *
 *  Created on: Jul 4, 2013
 *      Author: nao
 */

#ifndef LINE3D_H_
#define LINE3D_H_

#include <CGAL/Line_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Cartesian.h>
#include "types.h"

class Segment3D : public ExactSegment3D
{
public:
	Segment3D(Point3D p1,Point3D p2);
	Segment3D();
	virtual ~Segment3D();

	/*
	 * @param otherLine: line we're comparing this with
	 * @param Threshold: range 0-1, 0: is exactly the same, 1: 90 degrees difference
	 */
	bool isCollinear(const Segment3D &otherLine, double Threshold) const;

	void mergeWithCollinear(const Segment3D &otherLine);
	void mergeWithParallel(const Segment3D& otherLine);
    bool Intersection(const Segment3D &otherLine,const Plane3D& plane,Point3D& intersection, Point2D& intersection2D,double Threshold);
};

class Segment2D : public ExactSegment2D
{
public:
	Segment2D(Point2D p1,Point2D p2);
	Segment2D();
	//virtual ~Segment2D();

	/*
	 * @param otherLine: line we're comparing this with
	 * @param Threshold: range 0-1, 0: is exactly the same, 1: 90 degrees difference
	 */
	bool isCollinear(const Segment2D &otherLine, double Threshold) const;

	void mergeWithCollinear(const Segment2D &otherLine);
	void mergeWithParallel(const Segment2D &otherLine);
	//bool Intersection(const Segment2D &otherLine,Point2D& intersection,double Threshold);
};

#endif /* LINE3D_H_ */
