sudo chmod 777 /dev/U*
roslaunch serial_gps gps.launch

# ------------------------------------------create package----------------------------------------------------------

cd ~/mybot_ws/src
# gps_test là tên package
catkin_create_pkg gps_test roscpp std_msgs
# Tạo file node C++
cd ~/mybot_ws/src/gps_test/src
mkdir -p src
cd src
nano gps_compare_node.cpp  # bỏ code vào
# Sửa CMakeLists.txt để build node
cd ~/mybot_ws/src/gps_test
nano CMakeLists.txt        # bỏ code vào         add_executable(gps_compare_node src/gps_compare_node.cpp)
                                   #             target_link_libraries(gps_compare_node ${catkin_LIBRARIES})

# Build workspace
cd ~/mybot_ws
catkin_make
source devel/setup.bash
rosrun gps_test gps_compare_node


# Tạo file .launch để chạy tự động
cd ~/mybot_ws/src/gps_test
mkdir -p launch
nano gps_test.launch                #  <launch>
                                    #    <node pkg="gps_test" type="gps_compare_node" name="gps_compare_node" output="screen"/>
                                    #  </launch>


roslaunch gps_test gps_test.launch



