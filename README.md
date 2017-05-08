# Natalnet/LPR RGB-D Reconstruction Toolkit


### Introduction

A set of utility code built by the Natalnet/LPR group for 3D reconstruction
applications using RGB-D (e.g. Microsoft Kinect) cameras.

### Requirements
------------

- OpenCV: www.opencv.org  (version 2.4.13.x)
- PCL: www.pointclouds.org

It is recommended to install both dependencies from source. The following two links will redirect you to their official tutorials to do so.

> [Install OpenCV on Linux](http://docs.opencv.org/2.4.13.2/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation])

> [Compiling PCL from source on Linux](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)

### Building
------------

First, get the newest version of this repository by using the follwing command on your working directory.

```bash
cd /my_working _directory 
```

```bash
git clone https://github.com/natalnet-lpr/rgbd_rtk.git
```

Create a directory called build, and change directory to it.

```bash
mkdir build
``` 
```bash
cd build/
```

Inside of the build directory, run the cmake command so that it will create the necessary makefiles.
```bash
cmake ..
```

Then, compile the rgbt_rtk by using the make command. The -j4 is optional but it is recommended. With it your code will me compiled in 4 parallel tasks.

```bash
make -j4
```

```bash
sudo make install
```

### License


### Authors
