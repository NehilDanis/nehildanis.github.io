---
layout: post
title: The Importance of Launch Files and Their Basic Usage in ROS1
---

Sometimes, especially while working on large tasks, you will require to run many ROS nodes at the same time. This can get tedious easily, since for each ROS node you need to start a new terminal tab. Also don't forget to run roscore, before starting all these nodes!

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

In case of using roslaunch, you won't be able to see the logs in your terminal but a log file:

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
<arg name="arg_name"/>
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

In some cases you may want to access the value of a particular argument in your launch file, such as:

```xml
<launch>
    <arg arg_name="parameter_1" default="12"/>
    <arg arg_name="parameter_2" default="28"/>
    
    <node
        pkg="package_name"
        type="executable_name"
        name="node_name"
        args="$(arg parameter_1)"
    />
    <node
            pkg="package_name_1"
            type="executable_name_1"
            name="node_name_1"
            args="$(arg parameter_2)"
    />
</launch>
```

### References
* [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/agitr-small-launch.pdf)
