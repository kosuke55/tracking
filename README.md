# tracking_ros  

`roslauch tracking kcf_tracker.launch`  

This node requires opencv>=3.3.  

The default for ros melodic is 3.2, you need to build the right version from source.  
Kinetic defaults to 3.3.1, so I recommend that.  

`git clone https://github.com/opencv/opencv_contrib.git`  
`cd opencv_contrib`  
`git checkout -b v3.3.1 3.3.1`  
`git clone https://github.com/opencv/opencv.git`  
`cd opencv`  
`git checkout -b v3.3.1 3.3.1`  
`mkdir build; cd build`  
`cmake -DBUILD_opencv_cudacodec=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules ..`  
`make -j$(nproc`  
`make install`  
