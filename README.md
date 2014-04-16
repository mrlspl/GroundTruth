GroundTruth
===========

Localization ground-truth system. It uses RGB-D cameras, i.e. Asus Xtion Pro Live and Microsoft Kinect

Build Instruction
===========

Requirements: libqt4, libpcl1.7, libopencv2.3, libprotobuf7, libcgal9
You can run the command below to install all of the required libraries at once:
  sudo apt-get install libqt4-dev libpcl1-dev libopencv-dev libprotobuf7 libcgal9

Note: You are strongly recommended to make install the libpcl from the source
      this library is very buggy in the driver and try to get the lates git revision
Notes
===========
- Project is tested on ubuntu 13.04 x86.

- There is a problem in PCL1.7 when two or more cameras.
  There would be a conflict in RGB images of each camera
  which shows the one that belongs to the other. But it does 
  produce significant defects in the output.
