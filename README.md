# rp_project

The project consists in a normal based matcher integrated in the ROS
environment. The aim is to:

-   Extract the normals from a 2D Laser Scan

-   Create a 4D kd tree and use it to run icp with normals

-   Output the transformation computed in the tf topic

-   Build a map of the robot’s sorroundings

# Compile procedure

Clone the repository on github with the following instruction:

    git clone https://github.com/gianni0907/rp_project.git <directory>

where `<directory>` is a local folder that will be created and in which
the files will be put. Go in the `<directory>/src/` folder and
initialize the ros workspace with the following command:

        catkin_init_workspace

A `CMakeList.txt` file will be created in the current folder. Go back to
`<directory>` and execute the build command to compile:

        catkin build -j

and this will take approximately 20 seconds. In the top level directory
the `build` and `devel` folders will be created, and to make the system
recognize our package we need to issue the following command to source
the workspace:

        source devel/setup.bash

# Run procedure

The basic command to run a ros node is

        rosrun <package_name> <node_name>

However, in the git repository there is a launch directory to ease the
run procedure since 5 nodes need to be run. In particular, in the
`<directory>/src/nicp_package/launch` folder there are 2 launch files:

-   `stage.launch` that launches the stage ros simulator

-   `start.launch` that runs the 4 nodes that make up this project

These files can be used with this command

        roslaunch nicp_package <launch_file_name>

# Testing

To test that the icp process is working, one can issue a velocity
command to the robot in the simulator and check that the map is built
correctly. It is possible to give a velocity command by publishing a
message on the `/cmd_vel` topic on a shell, e.g.:

        rostopic pub /cmd_vel geometry_msgs/Twist "linear:
          x: 0.5
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.3" -r 10

The `map_builder` node saves a file `map.txt` in the home directory, and
it is possible to visualize the map with gnuplot by accessing the
gnuplot shell and issuing a command like the following one:

        plot "map.txt" u 1:2 w p pt 7 ps 0.1

It is also possible to examine the node communication infrastructure
with the `rqt_graph` command.
