#include "pointcloudviewer.h"
#include "vtkPolyLine.h"
#include "vtkSmartPointer.h"
#include <vtkRenderWindow.h> // invalid static_cast error
#include <vtkJPEGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <QCoreApplication>
using namespace pcl;
PointCloudViewer::PointCloudViewer(QWidget *parent) : QVTKWidget(parent),firstCloudSet(false),firstShapeSet(false),firstBoxesSet(false),firstLineSet(false)
{
    vtkSmartPointer<vtkRenderWindow> renderWindow = visualizer.getRenderWindow();
    SetRenderWindow(renderWindow);
    visualizer.setCameraPosition(-1,-1,-1,0,90,0);
}

void PointCloudViewer::showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Cloud)
{

    if(!firstCloudSet)
    {
        firstCloudSet = true;
        visualizer.addPointCloud(Cloud);
    }
    else
        visualizer.updatePointCloud(Cloud);

    update();
//    static int i=0;
//        windowToJPG(visualizer.getRenderWindow(),QString("/home/nao/Desktop/" + QString::number(i++) + ".jpeg").toStdString().data());
        visualizer.spinOnce(100);
}

void PointCloudViewer::windowToJPG(vtkRenderWindow *rw, const char *filename)
{
    vtkWindowToImageFilter *filter = vtkWindowToImageFilter::New();
    filter->SetInput(rw);
    vtkJPEGWriter *jw = vtkJPEGWriter::New();
    jw->SetInput(filter->GetOutput());
    jw->SetFileName(filename);
    jw->Write();
    jw->Delete();
    filter->Delete();
}

void PointCloudViewer::showPlane(pcl::ModelCoefficients::Ptr plane)
{
    firstShapeSet = true;
    visualizer.addPlane(*plane);
    update();
}

void PointCloudViewer::ShowBoxes(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >& boxes)
{
    for(int i=0;i<boxes.size();i++)
        visualizer.addCube(boxes[i].first.x,boxes[i].second.x,boxes[i].first.y,boxes[i].second.y,boxes[i].first.z,boxes[i].second.z,1,0,0,QString::number(i).toStdString());
    firstBoxesSet = true;

    update();

}
void PointCloudViewer::clearScreen()
{
    if(firstShapeSet || firstBoxesSet || firstCloudSet)
        visualizer.removeAllShapes();
}
void PointCloudViewer::ShowLines(const std::vector<Segment3D>& Lines)
{
    for(int i=0;i<Lines.size();i++)
    {
        const char* s =(QString("Line")+QString::number(i)).toStdString().data();
        visualizer.addLine(PointXYZ(Lines[i].source().x(),Lines[i].source().y(),Lines[i].source().z()),
                           PointXYZ(Lines[i].target().x(),Lines[i].target().y(),Lines[i].target().z()),
                           1,0,0,s);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, s);
    }
    firstLineSet = true;

    update();
}
