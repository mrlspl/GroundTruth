#include "depthdevice.h"
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include "Calibrator.h"
#include "RobotDetector.h"
#include "Accessories.h"
#include <iostream>
#include <QObject>
#include <QApplication>
using namespace pcl::search;
using namespace openni_wrapper;
using namespace pcl;
using namespace std;


DepthDevice::DepthDevice(string ID,QWidget* parent): QFrame(parent),
    ImageLabel(Scene,this),Calibrated(false),BoundToFunction(false),ID(ID),interface(0),pointCloudViewer(this),scale(1)
{
    //      QObject::connect(this,SIGNAL(UpdateScene(const QGraphicsScene&)),&ImageLabel,SLOT(showScene(const QGraphicsScene&)));

    QObject::connect(this,SIGNAL(UpdateImage(const QImage&,ConstLineVector)),&ImageLabel,SLOT(showImage(const QImage&,ConstLineVector)),Qt::BlockingQueuedConnection);
    QObject::connect(&ImageLabel,SIGNAL(clicked(QMouseEvent*)),this,SLOT(clickedOnImage(QMouseEvent*)));
    QObject::connect(&buttons[0],SIGNAL(released()),this,SLOT(ButtonOneReleased()));
    QObject::connect(&buttons[1],SIGNAL(released()),this,SLOT(ButtonTwoReleased()));
    QObject::connect(&buttons[2],SIGNAL(released()),this,SLOT(ButtonThreeReleased()));
    QObject::connect(&buttons[3],SIGNAL(released()),this,SLOT(ButtonFourReleased()));
    QObject::connect(&buttons[4],SIGNAL(released()),this,SLOT(ButtonFiveReleased()));
    QObject::connect(&buttons[5],SIGNAL(released()),this,SLOT(ButtonSixReleased()));
    QObject::connect(&SwitchButton,SIGNAL(released()),this,SLOT(OnSwitchRelease()));
    QObject::connect(&CalibratedButton,SIGNAL(released()),this,SLOT(CalibratedButtonReleased()));
    QObject::connect(this,SIGNAL(ShowPlane(pcl::ModelCoefficients::Ptr)),&pointCloudViewer,SLOT(showPlane(pcl::ModelCoefficients::Ptr)),Qt::BlockingQueuedConnection);
    QObject::connect(this,SIGNAL(ShowCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)),&pointCloudViewer,SLOT(showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)),Qt::BlockingQueuedConnection);
    QObject::connect(this,SIGNAL(ShowBoxes(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >&)),&pointCloudViewer,SLOT(ShowBoxes(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >&)),Qt::BlockingQueuedConnection);
    QObject::connect(this,SIGNAL(clearScreen()),&pointCloudViewer,SLOT(clearScreen()),Qt::BlockingQueuedConnection);
    QObject::connect(this,SIGNAL(ShowLines(const std::vector<Segment3D>&)),&pointCloudViewer,SLOT(ShowLines(const std::vector<Segment3D>&)),Qt::BlockingQueuedConnection);
    QObject::connect(&thresholdSlider,SIGNAL(valueChanged(int)),this,SLOT(thresholdValueChange(int)));
    ImageLabel.setGeometry(30,0,WIDTH,HEIGHT);
    CvSize size;
    size.width = WIDTH;
    size.height = HEIGHT;
    frameRGB = cvCreateImage(size,IPL_DEPTH_8U,3);
    buttons[0].setGeometry(0,0,10,10);
    buttons[1].setGeometry(10,0,10,10);
    buttons[2].setGeometry(20,0,10,10);
    buttons[3].setGeometry(0,10,10,10);
    buttons[4].setGeometry(10,10,10,10);
    buttons[5].setGeometry(20,10,10,10);
    for(int i=0;i<POSITION_NUMBER;i++)
        buttons[i].setParent(this);
    CalibratedButton.setGeometry(0,30,20,20);
    CalibratedButton.setParent(this);
    SwitchButton.setGeometry(0,60,20,20);
    SwitchButton.setParent(this);
//    pointCloudViewer.setGeometry(ImageLabel.geometry().x(),ImageLabel.geometry().y(),ImageLabel.geometry().width()*scale,ImageLabel.geometry().height()*scale);
    pointCloudViewer.setGeometry(ImageLabel.geometry().x(),ImageLabel.geometry().y(),WIDTH,HEIGHT);
    pointCloudViewer.setVisible(false);
    ImageLabel.setVisible(true);
    ImageLabel.scale(scale,scale);
    thresholdSlider.setParent(this);
    thresholdSlider.setGeometry(10,100,20,50);
    thresholdSlider.setMaximum(100);
    thresholdSlider.setMinimum(0);
    clickedPoint.setX(-1);
    clickedPoint.setY(-1);
}

DepthDevice::~DepthDevice()
{
    if(interface)
        delete interface;
}

void DepthDevice::clickedOnImage(QMouseEvent* e)
{
    clickedPoint.setX(e->x());
    clickedPoint.setY(e->y());
}

void DepthDevice::DoProcessings(const PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    if(!Calibrated)
    {
        calibrator.SetCloud(cloud);
        calibrator.Calibrate();
    }
    if(Calibrated)
    {
        robotDetector.setCalibrationConfigs(calibrator.getPlane(),calibrator.getTransformationMatrix());
        robotDetector.SetCloud(cloud);
        robotDetector.Detect();
    }
}

void DepthDevice::cloud_cb_(const PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    DoProcessings(cloud);
    Lines.clear();
    const Landmark& landmark = calibrator.getLandmark();
    Accessories::XYZRGB2IPL(cloud,frameRGB);
    calibrator.DrawLandmarkOnImage(frameRGB);
    QImage rgbTMP((unsigned char*)frameRGB->imageData,
                  WIDTH,
                  HEIGHT,
                  QImage::Format_RGB888);

//    if(landmark.getIsOn())
//        Lines.push_back(QLineF(landmark.getFirstLine2D().source().x(),landmark.getFirstLine2D().source().y(),landmark.getFirstLine2D().target().x(),landmark.getFirstLine2D().target().y()));
//    static int i = 0;
//    rgbTMP.save(QString("/home/nao/Desktop/pic") + QString::number(i++) + QString(".jpeg"));
    emit UpdateImage(rgbTMP,&Lines);
    emit dataUpdated();
//    emit ShowCloud(cloud);
    emit clearScreen();
//    emit ShowCloud(pcl::PointCloud<PointXYZRGBA>::Ptr(new PointCloud<PointXYZRGBA>(*cloud,robotDetector.GetRobotCloud().indices)));
//    emit ShowCloud(robotDetector.getCluserPointCloud(0));
//    emit ShowPlane(calibrator.getPlane());
    s.clear();
    s.push_back(calibrator.getLandmark().getFirstLine3D());
    s.push_back(calibrator.getLandmark().getLastLine3D());
//    emit ShowLines(s);
//    emit ShowPlane(robotDetector.getUpperPlane());
    emit ShowBoxes(robotDetector.getBoundingBoxes());
    if(clickedPoint.x() >= 0)
    {
        PointXYZRGBA p = cloud->at(clickedPoint.x(),clickedPoint.y());
        if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
        {
            Point3D pp(p.x,p.y,p.z);
            pp = pp.transform(calibrator.getTransformationMatrix());
            cout << " pp : " << pp<<endl;
        }
        clickedPoint.setX(-1);
        clickedPoint.setY(-1);
    }

}

void DepthDevice::Start()
{
    if(BoundToFunction)
    {
        Stop();
        delete interface;
    }
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&DepthDevice::cloud_cb_, this, _1);
#ifdef VGA
    interface = new pcl::OpenNIGrabber(string("#") + ID,pcl::OpenNIGrabber::OpenNI_VGA_30Hz,pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
#elif defined(QVGA)
    interface = new pcl::OpenNIGrabber(string("#") + ID,pcl::OpenNIGrabber::OpenNI_QVGA_30Hz,pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
#else
    #error("NO IMAGE SIZE SPECIFIED")
#endif
    interface->registerCallback(f);
    interface->start();
}

void DepthDevice::Stop()
{
    interface->stop();
}
void DepthDevice::SetDeviceID(std::string ID)
{
    this->ID = ID;
}

void DepthDevice::ButtonOneReleased()
{
    Position = TOP_LEFT;
    calibrator.SetPosition(Position);
}

void DepthDevice::ButtonTwoReleased()
{
    Position = TOP_MIDDLE;
    calibrator.SetPosition(Position);
}

void DepthDevice::ButtonThreeReleased()
{
    Position = TOP_RIGHT;
    calibrator.SetPosition(Position);
}

void DepthDevice::ButtonFourReleased()
{
    Position = BOTTOM_LEFT;
    calibrator.SetPosition(Position);
}

void DepthDevice::ButtonFiveReleased()
{
    Position = BOTTOM_MIDDLE;
    calibrator.SetPosition(Position);
}

void DepthDevice::ButtonSixReleased()
{
    Position = BOTTOM_RIGHT;
    calibrator.SetPosition(Position);
}
void DepthDevice::CalibratedButtonReleased()
{
    calibrator.calibrationAccepted();
    Calibrated = true;
}
const vector<QPoint>& DepthDevice::getRefereeToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetRefereeToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}
const vector<QPoint>& DepthDevice::getRedRobotsToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetRedRobotsToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}
const vector<QPoint>& DepthDevice::getBlueRobotsToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetBlueRobotsToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}
const vector<QPoint>& DepthDevice::getWhiteRobotsToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetBlackRobotsToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}

const vector<QPoint>& DepthDevice::getBlackRobotsToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetWhiteRobotsToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}

const vector<QPoint>& DepthDevice::getBallToField()
{
    qtPoints.clear();
    //measures are in CM
    const vector<PointXYZ>& pclPoints = robotDetector.GetBallToField();

    for(int i=0;i<pclPoints.size();i++)
        qtPoints.push_back(QPoint(pclPoints[i].x*M2CM,pclPoints[i].y*M2CM));
    return qtPoints;
}
void DepthDevice::OnSwitchRelease()
{
    pointCloudViewer.setVisible(!pointCloudViewer.isVisible());
    ImageLabel.setVisible(!ImageLabel.isVisible());
}
void DepthDevice::thresholdValueChange(int value)
{
    calibrator.setThreshold(value/100.0);
}
