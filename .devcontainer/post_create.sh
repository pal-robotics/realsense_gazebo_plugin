
# Update the bash PS1 to include git details
cat /home/ubuntu/ros_ws/src/realsense_gazebo_plugin/.devcontainer/bash_prompt.sh >> $HOME/.bashrc

#Build and source the catkin workspace
cd  /home/ubuntu/ros_ws
catkin init
echo "source /home/ubuntu/ros_ws/devel/setup.bash" >> $HOME/.bashrc
