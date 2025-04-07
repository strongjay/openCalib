#!/bin/bash
sudo apt update
sudo apt install cmake build-essential libeigen3-dev libglfw3-dev libglew-dev libpng-dev libepoxy-dev
sudo apt install python3-pip
pip3 install wheel --upgrade

git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make
sudo make install
# 修改 CamkeList.txt
# target_link_libraries(${PROJECT_NAME} ... ${Pangolin_LIBRARIES} )
