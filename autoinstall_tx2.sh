PKG_PATH=`pwd`
sudo apt-get update
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0 -y
cd $PKG_PATH/install/usb/  
tar jxvf libusb-1.0.21.tar.bz2  
cd libusb-1.0.21/  
./configure  
make   
sudo make install 
cd $PKG_PATH/install/arm64/
tar zxvf flycapture.2.11.3.121_arm64.tar.gz
cd flycapture.2.11.3.121_arm64/include/
sudo mkdir -p /usr/include/flycapture
sudo cp *.h /usr/include/flycapture
cd ../lib/
sudo cp lib* /usr/lib
sudo sed -i -e 's/0613/777/g' /etc/udev/rules.d/40-flir.rules
sudo sed -i -e 's/0613/777/g' /etc/udev/rules.d/40-flir.rules

