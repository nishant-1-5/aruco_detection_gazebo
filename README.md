

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
5. Edit the world file to spawn aruco markers , in the demo video I have used turtlebot3_empty_world, and launch gazebo.
   
```
  <include>
  <uri>model://marker_0</uri>
  <pose>0 0 10 3.14 0 3.14</pose>
  </include>
```
  
 6. Now run the detection node in another terminal.
  ```
  roslaunch aruco_detection aruco_detection.launch
  ``` 
  [Demo](https://youtu.be/Cg8wes0mLWo)
 
  

