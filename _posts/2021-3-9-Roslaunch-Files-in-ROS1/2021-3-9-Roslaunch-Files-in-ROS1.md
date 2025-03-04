---
layout: post
title: The Importance of Launch Files and Their Basic Usage in ROS1
category: software
---

Sometimes, especially while working on large tasks, you will require to run many ROS nodes at the same time. This can get tedious easily, since for each ROS node you need to start a new terminal tab. In addition to that; you also need to remember to run roscore.

There must be another way! Well, this is basically where the importance of a launch file kicks in. 

### What are launch files?

ROS launch files are basically XML files which contains the information about the nodes that needs to be run. Let me phrase it better, if you have multiple nodes you want to run in your hand, you can create a launch file which can start the master and all these nodes at the same time. Here is a simple launch file example.

```xml
<launch>
    <node
        pkg="package_name"
        type="executable_name"
        name="node_name"
    />
    <node
        pkg="package_name_1"
        type="executable_name_1"
        name="node_name_1"
    />
</launch>
```

To execute the launch file roslaunch command is used, roslaunch checks for roscore before running the nodes. If roscore is not running, then roslaunch starts roscore first. That is why, we don't need to execute roscore, before using roslaunch.

```commandline
roslaunch package_name launch_file_name
```

### Basics of Launch Files

Like every other XML file, launch files also need to have root elements. In ROS launch files the root element is the launch tag. You need to place the nodes you need to launch in between the launch tags.

Some of the nodes may be crucial for your application. If that one particular node stops working, you may either want to restart the node, or even want to terminate all the nodes.

#### Respawning

If a node is crucial for your application's execution and it may terminate prematurely due to software or hardware reasons you may want to restart the node. In this case you need to add the following attribute to your node definition.

```xml
respawn="true"
```
This way if the node somehow stops working, it will restart again and hopefully will not effect the execution of your application.

#### Requiring

If you want the termination of a particular node in your launch file, to result in termination of all the other nodes defined in the same launch file, the you should add the following attribute to your node definition.

```xml
required="true"
```

This way if the particular node terminates, all the other nodes will also terminate.

#### Output

In case of using roslaunch, you won't be able to see the logs in your terminal but in a log file:

_**~/.ros/log/run_id/node_name_number_stdout.log**_

If you want to visualise the outputs of a node in the terminal then you can add the following to the definition of the particular node.

```xml
output="screen"
```

In case you want to visualise the results of all the nodes in the launch file on the terminal then you can execute the roslaunch command in the following way:

```commandline
roslaunch --screen package_name launch_file_name
```

#### Arguments in Launch Files

To make the launch files configurable, launch arguments are supported by roslaunch.
To declare an argument, use the arg element:

```xml
<arg name="arg_name" deault="first_argument"/>
```

There are multiple ways to assign a value to an argument:
* You can simply provide a value to a particular argument in a roslaunch command line:

```commandline
roslaunch package_name launch_file_name arg_name:=arg_value
```

* Or as an alternative you can assign the value to an argument directly in the launch file:

```xml
<arg name="arg_name" default="arg_value"/>
```


or



```xml
<arg name="arg_name" value="arg_value"/>
```

Both does the same job, the only difference is if you use _**value**_ element in the argument definition then you are not allowed to change the value of the argument anymore.
In case of using _**default**_ element you can redefine the value of the argument later on as well.

#### Accessing Another Launch File in Your Launch File

This is a vital task, since in ROS projects usually more than one packages are used, sometimes we also need to start nodes from other packages.
These nodes from other packages may also have some arguments, it would be really convenient to define those parameters whilestarting the node as well. To do so; you can use the _**include**_
tag. Inside of the include tag; name the absolute path of the launch file, that we want to run, needs to be included. For example, recently I used Azure Kinect in one of my projects. 
To be able to receive color cloud from the depth sensor, there were some parameters that need to be defined. Below you can see how this is done;

```xml
<include file="$(find azure_kinect_ros_driver)/launch/driver.launch">
    <arg name="color_enabled" value="true"/>
    <arg name="color_resolution" value="1080P"/>
    <arg name="point_cloud" value="true"/>
    <arg name="rgb_point_cloud" value="true"/>
    <arg name="fps" value="5"/>
</include>
```

#### Using Parameters in a Node

A node may have parameters which is used while performing its functionality. In such cases, it is always better to define
these parameters in the launch file rather than in your source code. It is better because:
* Looks cleaner!
* Time saver! If you want to change these parameters, if you chance them in your source code, you need to build your project again,
which is time consuming. But if you define these parameters in your launch file, you don't need any extra build process.
  
A parameter includes name, type and value(or default) fields and can be defined as follows:

```xml
<param name="parameter_1" type="float" value="81.5"/>
```

This parameter will be place in between the node tags. 

```xml
<node pkg="registration_package" type="registration_executable" name="registration_node">
    <param name="parameter_1" type="float" value="81.5"/>
</node>
```

These parameters can be easily accessed from your source code. You need to use either get_param or param methods. They both 
essentially do the same things. But is you use param method it will also initialize your parameter with a defined value,
in case any value has already been defined for that parameter, in the launch file.

#### Using Same Arguments in Different Nodes

In some cases, nodes may be using the same argument but with different names. In such cases arguments need to be defined
in between launch tags, but outside of any node tag. Let's consider node_1 and node_2 both using and argument which represents 
the voxel_size. Also consider that, for node_1 is it better to call this parameter as source_cloud_downsample_size while 
for node_2 downsample_size is a better fit. In such cases, instead of creating the same parameter two times we can apply the below solution.

```xml
<launch>
    <arg name="voxel_size" default="0.01"/>
    
    <node pkg="registration_package" type="preprocessing_executable" name="preprocessing_node">
        <param name="source_cloud_downsample_size" value="$(arg voxel_size)"/>
    </node>
    <node pkg="registration_package" type="registration_executable" name="registration_node">
        <param name="downsample_size" value="$(arg voxel_size)"/>
    </node>
</launch>
```

So the expressing in the value field of param tag means that the parameter will take the value of an argument called _**voxel_size**_.

### References
* [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/agitr-small-launch.pdf)
