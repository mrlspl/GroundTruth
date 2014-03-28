#ifndef DEPTHDEVICE_H
#define DEPTHDEVICE_H
#include "MSHLabel/MSHLabel.h"
#include <QPushButton>
#include "Calibrator.h"
#include "RobotDetector.h"
#include "string.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include "Defines.h"
#include "pointcloudviewer.h"
#include <QSlider>
class DepthDevice : public QFrame
{
    Q_OBJECT
public:
    DepthDevice(std::string ID = string("0"),QWidget* parent = 0);
    ~DepthDevice();
    void Start();
    void Stop();
    void SetDeviceID(std::string ID);
    const vector<QPoint>& getClustersToField();
    const vector<QPoint>& getRefereeToField();
    const vector<QPoint>& getRedRobotsToField();
    const vector<QPoint>& getBlueRobotsToField();
    const vector<QPoint>& getWhiteRobotsToField();
    const vector<QPoint>& getBlackRobotsToField();
    const vector<QPoint>& getBallToField();
private:
    void DoProcessings(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    void run();
private:
    std::string ID;
    Calibrator calibrator;
    IplImage* frameRGB;
    RobotDetector robotDetector;
    bool Calibrated,BoundToFunction;
    pcl::Grabber* interface;
    QGraphicsScene Scene;
    MSHLabel ImageLabel;
    QPushButton buttons[POSITION_NUMBER];
    QPushButton CalibratedButton;
    QPushButton SwitchButton;
    CAMERAPOSITION Position;
    PointCloudViewer pointCloudViewer;
    double scale;
    vector<Point3D> cgalPoints;
    vector<QPoint> qtPoints;
    QSlider thresholdSlider;
    vector<Segment3D> s;
    vector<QLineF> Lines;
    QPoint clickedPoint;
signals:
    void UpdateImage(unsigned char* image,int width,int height);
    void UpdateImage(const QImage&,ConstLineVector);
    void UpdateScene(const QGraphicsScene& Scene);
    void ShowPlane(pcl::ModelCoefficients::Ptr plane);
    void ShowCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr Cloud);
    void ShowBoxes(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ> >&);
    void dataUpdated();
    void clearScreen();
    void ShowLines(const std::vector<Segment3D>& );

private slots:
    void ButtonOneReleased();
    void ButtonTwoReleased();
    void ButtonThreeReleased();
    void ButtonFourReleased();
    void ButtonFiveReleased();
    void ButtonSixReleased();
    void CalibratedButtonReleased();
    void OnSwitchRelease();
    void thresholdValueChange(int value);
    void clickedOnImage(QMouseEvent*);

};

#endif // DEPTHDEVICE_H

