---
layout: post
title: [CMake] Target Foo requires the language dialect "CXX20" (with compiler extensions), but CMake does not know the compile flags to use to enable it
category: software
---

This morning while I was working on my side project, an interesting CMake issue occured. Well you see, I was trying to use the new Cpp20 feature [concepts](https://en.cppreference.com/w/cpp/language/constraints), hence I had to set the compiler flag CMAKE_CXX_STANDARD to 20, as shown below.

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```
As a result I got the following output.

```bash
[cmake] CMake Error in Foo/CMakeLists.txt:
[cmake]   Target "foo" requires the language dialect "CXX20" (with compiler extensions), but CMake does not know the compile flags to use to enable it.
```
Then I set the same flag to 17, and everything seems to work, except the file which uses concept implementation did not build. From this point I had two ideas, it was either my compiler version causing the issue or the CMake version.

I currently use GCC 8.4. Even though GCC supports some of the Cpp20 features since version 8, Concepts is not one of them. Concepts feature is only [available](https://gcc.gnu.org/gcc-10/changes.html#cxx) since version 10. Also, my CMake version was very old, it was 3.10.2. [Here](https://cmake.org/cmake/help/latest/prop_tgt/CXX_STANDARD.html) in the CMake documentation it is explained that the Cpp20 was only supported since CMake version 3.12. Hence I had to update both my Cpp compiler and the CMake distro.

## How to update your CMake version

As a ROS user, I know that it is never a good idea to meddle with your default CMake version. Usually when one searched for how to upgrade CMake to a newer version, people suggest to uninstall the old version. Well, that will for sure work, but this will identify many ROS files for deletion. Hence this will break your ROS distribution, and will lrequired you to a full reinstall of your ROS distribution to fix it. So let's do things differently. You can have more than one CMake version on your system and you can decide which one to use by exporting the install path on your terminal or adding it directly to your .bashrc. 

* Go to [here](https://cmake.org/download/) to get your desired version of CMake for your distribution. At this stage you have two options you can either get the shall script or build from sources and set the appropriate install prefix for your CMake install. I did the first option, after I downloded [version 3.23.3 bash script](https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1-linux-x86_64.sh), I placed it in a directory where I generally put my third party libraries. 

1. Run the script, it will create its own directory if you dont specify any other path, and will install the new CMake there.
```bash
sudo bash /opt/cmake-3.*your_version*.sh
``` 

2. At this stage if you run the following commands you will still get the information on your old CMake distibution. Because you haven't added you CMake install path to your environment variables.

```terminal
$ which cmake # will give the install path to your current cmake

$ cmake --version # will give the default cmake version
```

3. Add the new CMake install path to your environment variables.
```console
nehil@pavilion:~$ export PATH=$HOME/<yourCMakeInstallPath>/bin:$PATH

nehil@pavilion:~$ export CMAKE_PREFIX_PATH=$HOME/<yourCMakeInstallPath>:$CMAKE_PREFIX_PATH
```
4. Finally if you repeat the stage 2, you will be able to see the new cmake. However to be able to use this version all the time please update your environment variables in bashrc.
