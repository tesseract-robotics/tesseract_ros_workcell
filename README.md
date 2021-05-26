# tesseract_ros_workcell

This repo contain two main applications application_positioner and application_rail

## Application positioner
It can be launched using the following command `roslaunch twc_application application_positioner.launch`

**Note**: set the argument application to true as follows:
`  <arg name="application" default="true" />`

The application as shown is a robot arm act as a raster for the point in the surface

![application_positioner](workcell1.png)

### Overview

The robot preform two types of motion:

* Free space motion: `tesseract_planning::PlanInstructionType::FREESPACE`
* Linear motion: `tesseract_planning::PlanInstructionType::LINEAR`

At the beginning, the robot preform free space motion from the start position to the first point in the path.

then it perform Linear motion  when moving in
a raster segment(a group points next to each other represent one swap or scan of raster).

then from the end of one raster segment to the beginning of another raster segment.

At the end of the program it preform a frees pace motion to the starting point(Home position).
