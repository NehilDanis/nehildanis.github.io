---
layout: post
title: Using PCL in Ubuntu 18.04 with C++14 Standards
category: book
---

Last year, Point Cloud Library(PCL) replaced the usage of boost shared pointers with standard pointers. From PCL 11.1 and on they started to support standard shared pointers. See [here](https://github.com/PointCloudLibrary/pcl/blob/master/CHANGES.md#-1110-11052020-) for the change log.

Recently, I used the PCL viewer in one of my projects. You can see the code I used below:
```c++
pcl::visualization::PCLVisualizer::Ptr viewer  = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
viewer->setBackgroundColor (1, 1, 1);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_final (std::make_shared<PointCloudT>(final_cloud), 0, 255, 0);
viewer->addPointCloud<pcl::PointXYZ> (std::make_shared<PointCloudT>(final_cloud), rgb_final, "final");

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_target (target, 255, 0, 0);
viewer->addPointCloud<pcl::PointXYZ> (target, rgb_target, "target");

viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "final");

while (!viewer->wasStopped ())
{
  viewer->spinOnce (100);
}
```

This was working just fine on my computer, the settings are as the following; Ubuntu 18.04 and PCL 1.11.1. At some point
during development I had to switch to another workstation which also has Ubuntu 18.04 operating system.

During the compile process, CMake was able to find PCL so I thought; great, I don't need to install it. Then I get 
an error about the usage of standard shared pointers not being compatible.

A quick search showed that one can get PCL using the following command in Ubuntu 18. Apparently the user before me, used
this command and was able to get PCL ready easily.

```commandline
sudo apt install libpcl-dev
```

However, see [here](https://packages.ubuntu.com/bionic/libpcl-dev), the PCL version is 1.8. We already know that PCL started
supporting the usage of standard smart pointers only recently(PCL 11.1 and upper versions).

The way around is to get the more recent release of PCL and during the compilation of your own project, specify which 
version of PCL needs to be used.

### Build PCL from source

* Go to PCL repository [releases](https://github.com/PointCloudLibrary/pcl/releases). Find the release you want to get 
and change the tag name below accordingly. Download and unzip it, to anywhere you want in your system.
* After that go into the directory that you extracted and do the following:

Note: You can set the DCMAKE_BUILD_TYPE variable to Debug, if you want to be able to see the PCL source code during debugging.

```commandline
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ..
make --j8 (takes around 15 mins)
sudo make install
```
Cograts, you are set! Now all you need to do is changing the CMake file in your project. Friendly advice, take a close 
look to the build folder you created. You can see many information such as the install destination and the version information.

After the compilation, in the build folder you will see a file named *ConfigVersion.cmake The first part of this file's name is your
package name and if you look inside this package using the following command:

```commandline
vim *ConfigVersion.cmake
```

you will see that there is a variable called PACKAGE_VERSION, this parameter shows your packages version. While CMake tries to find
this package it will check this file and see if the existing package is compatible with the version that you are looking for.

So before you write your CMake check these information, to specify the package name and the version correctly :)

### Use PCL in your project

In your CMake file, after you defined the CMake version and give a name to you project, specify the C++ standard you want 
to use in the project. After this step you can as find_package command to find PCL and please don't forget to specify the 
version you want to use. Below you can see a basic CMake file which uses PCL.

```cmake
cmake_minimum_required(VERSION 3.10)
project(your project name)

#specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# also you can also specify the PATH, cmake will check /usr/include directories
# if you install you package somewhere else then you can specify the intall directory as the following
# find_package(PCL 1.11 PATHS /install_path/include/pcl-1.11)
find_package(PCL 1.11)

add_definitions(${PCL_DEFINITIONS})
add_executable(${PROJECT_NAME} your_source_code.c*)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
```

Now, you can go and use your standard shared pointers in your code!

Cheers!


