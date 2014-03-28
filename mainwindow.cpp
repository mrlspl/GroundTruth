#include "mainwindow.h"
#include "mainwindow_ui.h"
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include "vector"
#include <QPoint>
#include <iostream>

#include "messages.pb.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),fieldView(this)
{
    ui->setupUi(this);

    RefreshDeviceList();

    for(int i=0;i<deviceNumber;i++)
        connect(&Devices[i],SIGNAL(dataUpdated()),this,SLOT(deviceDataUpdated()), Qt::BlockingQueuedConnection);

    fstr.open("/home/nao/Desktop/humanoid.log",ios::app);
}

MainWindow::~MainWindow()
{
    fstr<<std::flush;
    delete ui;
}

void MainWindow::RefreshDeviceList()
{
    openni_wrapper::OpenNIDriver &driver = openni_wrapper::OpenNIDriver::getInstance();
    deviceNumber = driver.getNumberDevices();
    Devices = new DepthDevice[deviceNumber];
    int columns = deviceNumber<=3?deviceNumber:3;
    int raws = deviceNumber<=3?1:2;

    columns = columns == 0 ? 1 : columns;
    raws = raws == 0 ? 1 : raws;

    int cellWidth = this->width() / columns;
    int cellHeight = this->height() / 2 / raws;
    for(int i=0;i<deviceNumber;i++)
    {
        char a[2];
        a[0] = '1' + i;
        a[1] = 0;
        Devices[i].SetDeviceID(a);
        Devices[i].setParent(this);

        Devices[i].setGeometry((i%3)*cellWidth,(i<3?0:1)*cellHeight,cellWidth,cellHeight);
        Devices[i].Start();

    }
    fieldView.DrawField();
    fieldView.setGeometry(50,raws*cellHeight,TOTAL_FIELD_WIDTH,TOTAL_FIELD_HEIGHT);

}

void MainWindow::deviceDataUpdated()
{
    fieldView.Scene.clear();
    pointsBall.clear();
    pointsBlue.clear();
    pointsRed.clear();
    pointsRef.clear();
    pointsWhite.clear();
    pointsBlack.clear();
    fieldView.DrawField();
    FieldData fieldData;
    for(int i=0;i<deviceNumber;i++)
    {
        std::vector<QPoint> pRed = Devices[i].getRedRobotsToField();
        std::vector<QPoint> pBlue = Devices[i].getBlueRobotsToField();
        std::vector<QPoint> pWhite= Devices[i].getWhiteRobotsToField();
        std::vector<QPoint> pBlack = Devices[i].getBlackRobotsToField();
        std::vector<QPoint> pRef = Devices[i].getRefereeToField();
        std::vector<QPoint> pBall = Devices[i].getBallToField();
//        cout << " device ID: " << i;
        for(int j=0;j<pRed.size();j++)
        {
            pointsRed.push_back(QPoint(pRed[j].y()*SCALE + CENTER_X,pRed[j].x()*SCALE + CENTER_Y));
            Robot *robot = fieldData.mutable_robots()->Add();
            robot->mutable_position()->set_x(pRed[j].x()*CM2M);
            robot->mutable_position()->set_y(pRed[j].y()*CM2M);
            robot->set_team(true);
//            cout <<"RED: " << pRed[j].y()<<" , " << pRed[j].x()<< endl;
        }
        for(int j=0;j<pRef.size();j++)
        {
            pointsRef.push_back(QPoint(pRef[j].y()*SCALE + CENTER_X ,pRef[j].x()*SCALE + CENTER_Y));
            Position *ref = fieldData.mutable_referee()->Add();
            ref->set_x(pRef[j].x()*CM2M);
            ref->set_y(pRef[j].y()*CM2M);
        }
        for(int j=0;j<pBlue.size();j++)
        {
            Robot *robot = fieldData.mutable_robots()->Add();
            robot->mutable_position()->set_x(pBlue[j].x()*CM2M);
            robot->mutable_position()->set_y(pBlue[j].y()*CM2M);
            robot->set_team(false);
            pointsBlue.push_back(QPoint(pBlue[j].y()*SCALE + CENTER_X,pBlue[j].x()*SCALE + CENTER_Y));
//            cout << "BLUE: " << pBlue[j].y()<<" , " << pBlue[j].x() << endl;
        }
        for(int j=0;j<pBlack.size();j++)
        {
            pointsBlack.push_back(QPoint(pBlack[j].y()*SCALE + CENTER_X,pBlack[j].x()*SCALE + CENTER_Y));
        }
        for(int j=0;j<pWhite.size();j++)
        {
            pointsWhite.push_back(QPoint(pWhite[j].y()*SCALE + CENTER_X,pWhite[j].x()*SCALE + CENTER_Y));
        }
        for(int j=0;j<pBall.size();j++)
            pointsBall.push_back(QPoint(pBall[j].y()*SCALE + CENTER_X,pBall[j].x()*SCALE + CENTER_Y));
//        cout << endl;
    }
    fieldView.DrawBall(pointsBall);
    fieldView.DrawBlueRobots(pointsBlue);
    fieldView.DrawRedRobots(pointsRed);
    fieldView.DrawBlackRobots(pointsBlack);
    fieldView.DrawWhiteRobots(pointsWhite);
    fieldView.DrawRef(pointsRef);
    fstr<< fieldData.SerializeAsString().size();
    fstr << fieldData.SerializeAsString()<<flush;
//    static int i=0;
//    QPixmap::grabWidget(&fieldView).save(QString("/home/nao/Desktop/field") + QString::number(i++) + QString(".jpeg"));
}
