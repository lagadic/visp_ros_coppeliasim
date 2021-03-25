# Installation on Ubuntu 20.04

## Summary<a name="summary">

1. [Install system dependencies](#install-system-dep)</br>
1.1. [Install Ubuntu requested packages](#install-system-dep-ubuntu)</br>
1.2. [Install Iit Butterworth Filter](#install-system-dep-butterworth)</br>
1.3. [Install ViSP](#install-system-dep-visp)</br>
2. [Install ROS dependencies](#install-ros-dep)</br>
2.1. [Install ROS noetic](#install-ros-dep-noetic)</br>
2.2. [Install vision_visp package](#install-ros-dep-vision-visp)</br>
2.3. [Install visp_ros package](#install-ros-dep-visp-ros)</br>
2.4. [Install visp_ros_coppeliasim package](#install-ros-dep-visp-ros-coppeliasim)</br>
3. [Install CoppeliaSim](#install-coppeliasim)</br>
3.1. [Download CoppeliaSim](#install-coppeliasim-download)</br>
3.2. [Build ROSInterface for CoppeliaSim](#install-coppeliasim-ros-interface)</br>
4. [Run the Franka simulator](#run-franka-simulator)</br>

## 1. Install system dependencies <a name="install-system-dep"></a>

### 1.1. Install Ubuntu requested packages <a name="install-system-dep-ubuntu"></a>

- Update Ubuntu packages

    ```
    $ sudo apt-get update
    $ sudo apt-get upgrade
    ```

- Install requested 3rd parties for ViSP

    ```
    $ sudo apt-get install libopencv-dev libx11-dev liblapack-dev libeigen3-dev \
           libv4l-dev libzbar-dev libpthread-stubs0-dev libjpeg-dev             \
           libpng-dev libdc1394-dev libpcl-dev
    ```

- Install Orocos-kdl needed for inverse and direct robot arm kinematics computation

    ```
    $ sudo apt-get install liborocos-kdl-dev
    ```

Back to [summary](#summary).

### 1.2. Install Iit Butterworth Filter <a name="install-system-dep-butterworth"></a>

This filter is needed if data from CoppeliaSim to ROS is noisy. The source code is available [here](https://github.com/berndporr/iir1).

To proceed to the installation run:

```
$ sudo add-apt-repository ppa:berndporr/dsp
$ sudo apt install iir1 iir1-dev
```

Back to [summary](#summary).

### 1.3. Install ViSP <a name="install-system-dep-visp"></a>

ViSP last release is packaged for ROS, but at the time this tutorial was written, not for `neotic` distro. Moreover, ViSP last release is not packaged for Ubuntu 20.04.

That's why you have to install [VISP](visp.inria.fr) from source, following the instructions listed [here](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html). 

We resume hereafter these instructions:

- Get ViSP source code

    ```
    $ mkdir -p ~/software/visp
    $ cd ~/software/visp
    $ git clone https://github.com/lagadic/visp.git
    ```

- Create a build forder

    ```
    $ mkdir -p visp-build
    ```

- Execute cmake to configure ViSP

    ```
    $ cd visp-build
    $ cmake ../visp
    ```

- Compile ViSP

    ```
    $ make -j4
    ```

Back to [summary](#summary).

## 2. Install ROS dependencies <a name="install-ros-dep"></a>

### 2.1. Install ROS noetic <a name="install-ros-dep-noetic"></a>

For **Ubuntu 20.04**, the native version of ROS is **noetic**. A tutorial on how to easily install ROS noetic on Ubuntu can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu). Be sure to complete also the [first tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) in order to create a ROS catkin workspace.

At this point we assume that your workspace is in `~/catkin_ws`.

Back to [summary](#summary).

### 2.2. Install vision_visp package <a name="install-ros-dep-vision-visp"></a>

From this metapackage we need `visp_bridge` package in order to be able to build `visp_ros` as described in the next section. To install this package:

- Clone `vision_visp` package in the cartkin workspace, and switch to `noetic` branch

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/lagadic/vision_visp.git --branch noetic
    ```

- Build package

    ```
    $ source /opt/ros/noetic/setup.bash
    $ cd ~/catkin_ws/
    $ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=~/software/visp/visp-build
    ```

Back to [summary](#summary).

### 2.3. Install visp_ros package <a name="install-ros-dep-visp-ros"></a>

- Clone `visp_ros` package in the cartkin workspace

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/lagadic/visp_ros.git
    ```

- Build package

    ```
    $ source /opt/ros/noetic/setup.bash
    $ cd ~/catkin_ws/
    $ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=~/software/visp/visp-build
    ```

Back to [summary](#summary).

### 2.4. Install visp_ros_coppeliasim package <a name="install-ros-dep-visp-ros-coppeliasim"></a>

- Clone `visp_ros_coppeliasim` package in the cartkin workspace

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/lagadic/visp_ros_coppeliasim.git
    ```

- Build package

    ```
    $ source /opt/ros/noetic/setup.bash
    $ cd ~/catkin_ws/
    $ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=~/software/visp/visp-build
    ```

Back to [summary](#summary).

## 3. Install CoppeliaSim <a name="install-coppeliasim"></a>

### 3.1. Download CoppeliaSim <a name="install-coppeliasim-download"></a>

- Download the last edu version of `CoppeliaSim` for Ubuntu 20.04 from [here](https://coppeliarobotics.com/downloads) (e.g., `CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz`).
- Extract the archive content in `~/software` workspace.
- At this point you should have `CoppeliaSim` in `~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/` folder

Back to [summary](#summary).

### 3.2. Build ROSInterface for CoppeliaSim <a name="install-coppeliasim-ros-interface"></a>

Since `CoppeliaSim` comes with a `ROSInterface` build for ROS `melodic` and we are using ROS we need to:

1. Get the last version of  `libPlugin` from [here](https://github.com/CoppeliaRobotics/libPlugin).

    ```
    $ cd ~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04/programming
    $ mv libPlugin/ libPlugin_orig/
    $ git clone https://github.com/CoppeliaRobotics/libPlugin.git --branch coppeliasim-v4.1.0
    ```

2. Get `ROSInterface` node source code

    ```
    $ cd ~/catkin_ws/src/
    $ git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git \
                --branch coppeliasim-v4.1.0 sim_ros_interface 
    ```

3. Build `ROSInterface` node 

    ```
    $ cd ~/catkin_ws
    $ source /opt/ros/noetic/setup.bash
    $ export COPPELIASIM_ROOT_DIR=~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
    $ sudo apt-get install python3-pip xsltproc
    $ pip3 install xmlschema
    $ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
4. Copy `ROSInterface` in the CoppeliaSim directory

    ```
    $ cp devel/lib/libsimExtROSInterface.so ~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
    ```

Back to [summary](#summary).

## 4. Run the Franka simulator <a name="run-franka-simulator"></a>

In what follows, we assume that `CoppeliaSim` is located in `~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04`, that the ROS package `visp_ros_coppeliasim` is located in `~/catkin_ws/src`, and that we are going to run the file related to IBVS AprilTag synchronization (see folder `visp_ros_coppeliasim/src` for more details). 

To properly run the simulator, you will need three terminals, one for the `roscore`, one for `CoppeliaSim`, and one for the ROS node (note that `roscore` should be started before `CoppeliaSim`):

- **In terminal 1 run:**

    ```
    $ source /opt/ros/noetic/setup.bash
    $ roscore
    ```

- **In terminal 2 run:**

    ```
    $ source ~/catkin_ws/devel/setup.bash
    $ cd ~/software/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
    $ LC_NUMERIC=en_US.UTF-8 ./coppeliaSim.sh
    ```

    Now in `CoppeliaSim` GUI, enter menu `"File > Open scene..."` and browse to `~/catkin_ws/src/visp_ros_coppeliasim/coppeliasim_scenes` folder to select `franka_joint_torque_control.ttt`.

    At this point, in `CoppeliaSim` GUI you should see the following feedback:

    ```
    [sandboxScript:info]   Simulator launched, welcome!
    [CoppeliaSim:info]   Default scene was set-up.
    [CoppeliaSim:info]   Loading scene ($HOME/catkin_ws/src/visp_ros_coppeliasim/coppeliasim_scenes/franka_joint_torque_control.ttt).  Serialization version is 22.
    [CoppeliaSim:info]   File was previously written with CoppeliaSim version 4.01.00 (rev 1)
    [CoppeliaSim:info]   Scene opened.
    ROS interface was found.
    ```
    Be sure that ROS interface is found. If this is not the case, it means:

    - either that you forget to `source ~/catkin_ws/devel/setup.bash`, 
    - either that you have an issue that comes from ROS interface build step described [here](#install-coppeliasim-ros-interface).

- **In terminal 3 run:**

    ```
    $ source ~/catkin_ws/devel/setup.bash
    $ rosrun visp_ros_coppeliasim ibvs_april_tag_sync
    ```
    
At this point you should be able to see the following in CoppeliaSim GUI:

![visp_ros_coppeliasim_gui](https://user-images.githubusercontent.com/1412746/112324358-01132a80-8cb3-11eb-84c9-458d8a9fbee1.jpg)

Moreover, in the display opened when running `ibvs_april_tag_sync` node you should also see something similar to:

![visp_ros_coppeliasim_visp_display](https://user-images.githubusercontent.com/1412746/112326267-abd81880-8cb4-11eb-95bc-b0e17047f857.jpg)

You need to click in the display to start the visual-servo.

Back to [summary](#summary).
