1. put this package in your workspace/src

2. change the path of files  
competition I:  
change line 178 in src/icp.cpp to the path of your map pcd file  
change line 14 in launch/comp1.launch to the path of your rosbag  
               ----------------------  
competition II:  
change line 225 to line 235 in src/icp2_n.cpp to the path of your map pcd file  
change line 16 in launch/comp2.launch to the path of your rosbag  
                ----------------------  
competition III:  
change line 153 to line 164 in src/icp3.cpp to the path of your map pcd file  
change line 13 in launch/comp3.launch to the path of your rosbag  

3. after catkin_make and source devel/setup.bash launch the file  
competition I:  
roslaunch localization_309512009 comp1.launch  
                -------------------  
competition II:  
roslaunch localization_309512009 comp2.launch  
                -------------------  
competition III:  
roslaunch localization_309512009 comp3.launch  
