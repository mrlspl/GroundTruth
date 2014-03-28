#ifndef LANDMARK_H
#define LANDMARK_H
#include "Segment2D.h"
class Landmark
{
public:
Landmark(const Segment3D &firstLine, const Segment3D &lastLine,const Point3D& p,const Point2D& p2d);
Landmark();
//Landmark(const Segment2D &firstLine, const Segment2D &lastLine,const Point2D& p);
const Segment3D& getFirstLine3D() const;
const Segment3D& getLastLine3D() const ;
const ExactSegment2D& getFirstLine2D() const ;
const ExactSegment2D& getLastLine2D() const;
const Point2D& getIntersection2D() const;
void setFirstLine2D(const ExactSegment2D& first);
void setLastLine2D(const ExactSegment2D& last);
const Point3D& getIntersection3D() const;
void setIsOn(bool isOn);
bool getIsOn() const;
//Point2D getIntersection2D();

private:
Segment3D lines3D[2];
Point3D intersection3D;
ExactSegment2D lines2D[2];
Point2D intersection2D;
bool isOn;
};

#endif //LANDMARK_H
