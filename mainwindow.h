#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "depthdevice.h"
#include "fieldview.h"
#include "boost/shared_ptr.hpp"
#include <fstream>
#define DEVICE_NUMBER
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void RefreshDeviceList();
    FieldView fieldView;
    Ui::MainWindow *ui;
    DepthDevice* Devices;
    int deviceNumber;
    vector<QPoint> pointsRed;
    vector<QPoint> pointsBlue;
    vector<QPoint> pointsWhite;
    vector<QPoint> pointsBlack;
    vector<QPoint> pointsBall;
    vector<QPoint> pointsRef;
    std::ofstream fstr;
public slots:
    void deviceDataUpdated();
};

#endif // MAINWINDOW_H
