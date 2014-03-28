#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H
#include <QWidget>
#include <QVTKWidget.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/ModelCoefficients.h"
#include <pcl/visualization/cloud_viewer.h>
#include "Segment2D.h"
class PointCloudViewer: public QVTKWidget
{
    Q_OBJECT
public:
    PointCloudViewer(QWidget *parent);


private:
    pcl::visualization::PCLVisualizer visualizer;
    bool firstCloudSet,firstShapeSet,firstBoxesSet,firstLineSet;

public slots:
    void showPlane(pcl::ModelCoefficients::Ptr plane);
    void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Cloud);
    void ShowBoxes(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >&);
    void ShowLines(const std::vector<Segment3D>& );
    void windowToJPG(vtkRenderWindow *rw, const char *filename);
    void clearScreen();
};

#endif // POINTCLOUDVIEWER_H
