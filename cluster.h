#ifndef CLUSTER_H
#define CLUSTER_H
#include <pcl/PointIndices.h>
#include "Defines.h"
#include <pcl/point_types.h>
#include <utility>
#include <pcl/point_cloud.h>
class Cluster
{
public:
    Cluster(const pcl::PointIndices& pointIndices);

    pcl::PointIndices pointIndices;
    ObjectId clusterID;
    ObjectGuess clusterGuess;
    std::vector<int> colors;
    pcl::PointXYZ representativePointToCam;
    pcl::PointXYZ representativePointToField;
    std::pair<pcl::PointXYZ ,pcl::PointXYZ > boundingBox;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud;
};

#endif // CLUSTER_H
