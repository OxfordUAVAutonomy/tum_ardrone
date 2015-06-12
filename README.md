# Package modified tum_ardrone

This package is a modified version of the original `tum_ardrone` package from https://github.com/tum-vision/tum_ardrone/tree/hydro-devel

## Installation

In order to install the ROS, the `tum_simulator` package, and the modified `tum_ardrone` package please execute the following steps:

1. Install Ubuntu 12.04.5 LTS. We recommend ubuntu-12.04.5-desktop-i386, as there seem to be some issues with tum_simulator and ubuntu-12.04.5-desktop-amd64. We have also successfully tested this setup in a virtual machine, namely https://www.virtualbox.org/.
    
2. Install ROS Hydro (see http://wiki.ros.org/hydro/Installation/Ubuntu):
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-hydro-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall
```
          
3. Create catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
          

4. Install ardrone_autonomy (see https://github.com/AutonomyLab/ardrone_autonomy/tree/hydro-devel#compile-and-install-from-source):
```
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b hydro-devel
cd ~/catkin_ws
catkin_make
```

5. Install tum_simulator (see http://answers.ros.org/question/193159/how-can-i-install-tum_simulator-on-ros-hydro-the-instructions-are-written-for-fuerte/ and https://github.com/tum-vision/tum_simulator/blob/master/readme.txt):
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/tum-vision/tum_simulator.git
cd ..
rosdep install --from-paths src -i
catkin_make
```
        

6. Install the modified tum_ardrone (see also https://github.com/tum-vision/tum_ardrone/tree/hydro-devel and http://answers.ros.org/question/190476/installing-tum_ardrone-with-catkin_make/)
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/OxfordUAVAutonomy/tum_ardrone.git -b hydro-devel
cd ..
rosdep update
rosdep install --from-paths src -i
catkin_make
```
Note: For installing the original tum_ardrone package, use the following steps:
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/tum-vision/tum_ardrone.git -b hydro-devel
cd ..
rosdep update
rosdep install --from-paths src -i
catkin_make
```
          
7. Add source for catkin workspace (just in case and for convenience)

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```
          
## Launch tum_ardrone

In order to launch the simulator and the autopilot, please use the following commands in the given order
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch 
```

## Troubleshooting

* tum_simulator does seem to have issues on 64-bit systems, that's why we recommend the i386 Ubuntu image
* if catkin_make build fails, check the permissions of the *.cfg files, they should be executable
* if you used the original tum_ardrone and catkin_make build fails, delete ~/catkin_ws/devel/include/tum_ardrone (a parameter in AutopilotParams.cfg was renamed and it seems that catkin does not regenerate AutopilotParamsConfig.h for some reason)


## Licence

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.
