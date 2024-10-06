# Wind-Turbine-Express

### Adding OpenCV

You'll need to build OpenCV with the proper dependencies, including QUIRC. Here are simplified steps:

```bash
sudo apt update
sudo apt install build-essential cmake git
sudo apt install libjpeg-dev libpng-dev libtiff-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libv4l-dev libxvidcore-dev libx264-dev
sudo apt install libgtk-3-dev libcanberra-gtk-module
sudo apt install libatlas-base-dev gfortran
sudo apt install python3-dev
```
Inside the aquabot_competitor directory

```bash
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j8
sudo make install
```
