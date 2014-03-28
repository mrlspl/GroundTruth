#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include "Calibrator.h"
#include "RobotDetector.h"
//#include "typedefs.h"
#include <iostream>
using namespace pcl::search;
using namespace openni_wrapper;
using namespace pcl;
using namespace std;
#define STREAM_FROM_CAMERA


class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
	Calibrator calibrator;
	RobotDetector robotDetector;
	bool Calibrated;
	void DoProcessings(const PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		//	viewer.showCloud (Cloud);
		if(!Calibrated)
		{
			calibrator.SetCloud(cloud);
			Calibrated  = calibrator.Calibrate();
			robotDetector.setCalibrationConfigs(calibrator.getPlane(),calibrator.getLandmark());
		}
		if(Calibrated)
		{
			robotDetector.SetCloud(cloud);
			robotDetector.Detect();
			boost::function<void (pcl::visualization::PCLVisualizer&)> f =
					boost::bind (&RobotDetector::Debug3D, robotDetector, _1);

			viewer.runOnVisualizationThreadOnce (f);

		}


	}
	void cloud_cb_(const PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
		{


			DoProcessings(cloud);
			//viewer.showCloud (cloud);
			//cv::Mat frameRGB=cv::Mat(cloud->width,cloud->height,CV_8UC3);
			//XYZRGB2IPL(cloud, frameRGB);
			//IplImage *imagen = new IplImage(frameRGB);
			//         pcl::io::savePCDFileASCII ("testXYZRGB.pcd", *cloud);
		}
	}



	void run ()
	{
		Calibrated = false;
#ifdef STREAM_FROM_CAMERA
		OpenNIDriver &driver = OpenNIDriver::getInstance();
		unsigned int deviceNum = driver.getNumberDevices();
		pcl::Grabber** interface = new pcl::Grabber*[deviceNum];
		for(int i=0;i<deviceNum;i++)
		{
			char a[2];
			a[0] = '1' + i;
			a[1] = 0;
			interface[i] = new pcl::OpenNIGrabber(string("#") + string(a),pcl::OpenNIGrabber::OpenNI_VGA_30Hz,pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
		}

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		for(int i=0;i<deviceNum;i++)
		{
			interface[i]->registerCallback(f);
			interface[i]->start();
		}
#else
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("RobotOnCross.pcd", *Cloud);
        //		Cloud = smoothPointCloud(Cloud,0.01,3);
        DoProcessings(Cloud);
        viewer.showCloud (robotDetector.RobotCloud);
        RobotDetector robotDetector(Cloud);
        robotDetector.Detect();
#endif
        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }


#ifdef STREAM_FROM_CAMERA
        for(int i=0;i<deviceNum;i++)
            interface[i]->stop ();
#endif //STREAM_FROM_CAMERA
	}

    pcl::visualization::CloudViewer viewer;
};

int main()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
