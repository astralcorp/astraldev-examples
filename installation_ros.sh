#!/bin/bash
echo "First we make pm2 daemonized."
pm2 list
ROS_WORKSPACE="/home/$USER/catkin_ws"
PACKAGE_NAME='astral'
PYTHON_FILE='telemetry_datas.py'
SOURCE_PATH="/home/$USER/Desktop/autonomous_drone"
### ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
# Source ROS
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-catkin-tools
sudo apt install -y python-catkin-tools
sudo apt install -y python3-rosdep python3-rosinstall-generator python3-wstool build-essential
pip install distro
sudo rosdep init
rosdep update
mkdir -p ~/catkin_ws/src
sudo apt install -y ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
sudo apt install -y libpcl1 ros-melodic-octomap-*



# Step 1: Create a new ROS package
cd $ROS_WORKSPACE/src && catkin_create_pkg $PACKAGE_NAME --rosdistro melodic
 

mkdir $ROS_WORKSPACE/src/$PACKAGE_NAME/src


# Step 2: Copy the python file to the package
cp $SOURCE_PATH/$PYTHON_FILE $ROS_WORKSPACE/src/$PACKAGE_NAME/src/$PYTHON_FILE

# Step 3: Make the python code executable
chmod +x $ROS_WORKSPACE/src/$PACKAGE_NAME/src/$PYTHON_FILE
 
# Step 4: Build the ROS package
cd $ROS_WORKSPACE && catkin build
 

# Source the environment (you may want to add this to your .bashrc)
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 5: Adding our telemetry script to launch script
PYTHON_CODE=$(cat << 'EOF'
        </include>
        <node name="drone_data_sender" pkg="astral" type="telemetry_datas.py" output="screen">
        </node>
EOF
)

# Use printf and sudo sed to insert the Python code after the </include> tag with proper indentation
printf '%s\n' "$PYTHON_CODE" | sudo sed -e '/<\/include>/ {
    r /dev/stdin
    d
}' /opt/ros/melodic/share/mavros/launch/px4.launch > /tmp/modified_px4.launch

# Move the modified temporary file back to the original file with sudo
sudo mv /tmp/modified_px4.launch /opt/ros/melodic/share/mavros/launch/px4.launch

# Step 6: Update the fcu_url value in the launch file
UPDATED_ARG='<arg name="fcu_url" default="/dev/ttyACM0:921600" />'

# Use sed to replace the desired line in the launch file
sudo sed -i 's#<arg name="fcu_url" default="/dev/ttyACM0:57600" />#'"$UPDATED_ARG"'#' /opt/ros/melodic/share/mavros/launch/px4.launch

echo "Launch file updated."

# Optional: Check if the package was properly created and can be found by ROS
rospack find "$PACKAGE_NAME"

echo "Shell script execution complete."


echo "Close and reopen the terminal to use ROS functions."
