GroundTruth
===========

Localization ground-truth system. It uses RGB-D cameras, i.e. Asus Xtion Pro Live and Microsoft Kinect

Build Instruction
===========

You can run the command below to install all of the required libraries at once:

  sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
  sudo apt-get update
  sudo apt-get install build-essential cmake libqt4-dev libopencv-dev libprotobuf-dev libcgal-dev libpcl-all

Note: You are strongly recommended to make install the libpcl from the source
      this library is very buggy in the driver and try to get the lates git revision.
      If you are doing so, then you must run the following on your terminal after the command above:
     
     cd ~/
     git clone https://github.com/PointCloudLibrary/pcl.git
     cd pcl/
     cmake -i
     make
     sudo make install
     cd GroundTruthPath
     cmake -i
     make
      
Notes
===========
- Project is tested on ubuntu 13.04 x86 and x86_64
- Do not try to run the program in Ubuntu 12.10 or older it leads to failure
- Do not try to use any other version of these packages. PCL will panic.
- There is a problem in PCL1.7 when two or more cameras.
  There would be a conflict in RGB images of each camera
  which shows the one that belongs to the other. But it does 
  produce significant defects in the output.
