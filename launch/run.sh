catkin_make -C /home/tmn/AirSim/ros;

catkin_make -C /home/$USER/dev_ws;
source /home/$USER/dev_ws/devel/setup.bash

# Launch airsim environment Neighbourhood
bash /home/tmn/EnvAirSim/AirSimNH/LinuxNoEditor/AirSimNH.sh -windowed &

# Lanch the joystick node
sleep 5;
roslaunch airsim_joyctr run.launch;