#include "cluster.h"

Cluster::Cluster(const pcl::PointIndices& pointIndices):representativePointToCam(0,0,0),representativePointToField(0,0,0)
{
    this->pointIndices = pointIndices;
    colors.resize(NumberObjectID);
    boundingBox.first = pcl::PointXYZ(0,0,0);
    boundingBox.second = pcl::PointXYZ(0,0,0);
}
