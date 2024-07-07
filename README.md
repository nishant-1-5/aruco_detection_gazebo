
# Aruco Marker Detection with TurtleBot3 in Gazebo
This project demonstrates how to detect Aruco markers in a Gazebo simulation using TurtleBot3. The markers will be spawned in the Gazebo world, and the TurtleBot3 will detect them using its camera. (Aruco Dictionary: cv2.aruco.DICT_6X6_100)

## Note
This repo used ROS-Noetic and Gazebo 11 

## Requirements
- ROS
- TurtleBot3
- Gazebo
- OpenCV
- Aruco marker models for Gazebo

## Use
Assuming you already have ROS and gazebo ready to go (if not, refer TurtleBot3 Emanuel, along with gazebo installation tutorial)
1. Clone this repo
2. Follow the instructions mentioned in [this](https://github.com/sacchinbhg/gazebo_aruco_models) repo to spawn aruco markers in gazebo.
3. Use turtlebot3 with camera (I used waffle_pi in demo)
  ```
  export TURTLEBOT3_MODEL=waffle_pi
  ```
4. Build the workspace and source it.
   ```
   catkin_make
   source devel/setup.bash
   ```
5. Edit the world file to spawn aruco markers or run the following commands at run time, in the demo video I have used turtlebot3_empty_world.
   ```
   rosrun gazebo_ros spawn_model -file `rospack find gazebo_aruco_models`/models/marker_0/model.sdf -sdf -model aruco_marker_0 -x 10 -y 0 -z 2 -R 3.14 -P 0 -Y 3.14
   rosrun gazebo_ros spawn_model -file `rospack find gazebo_aruco_models`/models/marker_1/model.sdf -sdf -model aruco_marker_1 -x 10 -y 0 -z 2 -R 3.14 -P 0 -Y 3.14
   ```
 6. Now run the detection node in another terminal.
  ```
  roslaunch aruco_detection aruco_detection.launch
  ``` 
  [Demo](https://youtu.be/Cg8wes0mLWo)
 
  

