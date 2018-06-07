# PointGrey Reader
## 1. Download code  
   Enter your catkin work space  
```
cd YOUR_PATH/catkin_ws/src  
  
git clone https://github.com/gaowenliang/ptgrey_reader.git
```
## 2. Install dependency for ptgrey
### 2.1 Install libusb-1.0.21  
```
cd YOUR_PATH/catkin_ws/src/ptgrey_reader/install/usb/  
  
tar jxvf libusb-1.0.21.tar.bz2  
  
cd libusb-1.0.21/  
```  
You can follow the INSTALL file in the folder or follow the command below:  
```
./configure  
make   
sudo make install  
```
### 2.2 Install driver for ptgrey  
The driver support computer with Intel CPUs and NVIDIA TX2.

* For computer with Intel CPU:
```
cd YOUR_PATH/catkin_ws/src/ptgrey_reader/install/amd64/
tar zxvf flycapture2-2.11.3.121-amd64-pkg.tgz
cd flycapture2-2.11.3.121-amd64/
```

* For TX2:
```
cd YOUR_PATH/catkin_ws/src/ptgrey_reader/install/arm64/
tar zxvf flycapture.2.11.3.121_arm64.tar.gz
cd flycapture.2.11.3.121_arm64/
```

Follow the README file in the folder or follow the command below:  
```
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0 -y
```  
FLYCAPTURE2 INSTALLATION:
```
sudo sh install_flycapture.sh
```

```
cd YOUR_PATH/catkin_ws/   
catkin_make
```
Before using that you may need to give the enough authority  
```
sudo gedit cd /etc/udev/rules.d/40-flir.rules
```  
Change all the 0613 or 0664 to 777 in the "40-flir.rules" and save.
Then restart the system.
## 4. Run  
First, you need to run the camera_list to see your camera ID number, and copy it to the launch file.  
```
roscore  
  
rosrun ptgrey_reader camera_list
```
After you change the launch file with the ID number. 

If you want to use muti-camera, you need to excute this command first and roslaunch each camera's launch you need:  
```
sudo -S sh -c 'echo 2048 > /sys/module/usbcore/parameters/usbfs_memory_mb'  
```
