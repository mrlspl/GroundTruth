#include "Landmark.h"
Landmark::Landmark(const Segment3D &firstLine, const Segment3D &lastLine,const Point3D& p,const Point2D& p2d):intersection3D(p),intersection2D(p2d),isOn(false)
{
//    lines3D.resize(2);
    lines3D[0] = firstLine;
    lines3D[1] = lastLine;
}

Landmark::Landmark():intersection3D(Point3D(0,0,0))
{
}
/*
Landmark::Landmark(const Segment2D &firstLine, const Segment2D &lastLine,const Point2D& p)
{

lines2D[0] = firstLine;
lines2D[1] = lastLine;
intersection2D = p;
}*/
const Segment3D& Landmark::getFirstLine3D() const
{
return lines3D[0];
}
const Segment3D& Landmark::getLastLine3D() const
{

return lines3D[1];
}
const ExactSegment2D& Landmark::getFirstLine2D() const
{
return lines2D[0];
}
const ExactSegment2D& Landmark::getLastLine2D() const
{
return lines2D[1];
}
const Point2D& Landmark::getIntersection2D() const
{
    return intersection2D;
}
void Landmark::setFirstLine2D(const ExactSegment2D& first)
{
lines2D[0] = first;
}
void Landmark::setLastLine2D(const ExactSegment2D& last)
{
lines2D[1] = last;
}
const Point3D& Landmark::getIntersection3D() const
{
return intersection3D;
}
/*Point2D Landmark::getIntersection2D()
{
return intersection2D;
}*/
void Landmark::setIsOn(bool isOn)
{
    this->isOn = isOn;
}
bool Landmark::getIsOn() const
{
    return isOn;
}
