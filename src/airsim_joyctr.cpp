#include <stdio.h>
#include <vector>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

// Airsim velocity message
#include <airsim_ros_pkgs/VelCmd.h>
// Airsim takeoff message
#include <airsim_ros_pkgs/Takeoff.h>

#include "math_common.h"

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_joyctr");
    ros::NodeHandle nh("~");

    // Start the callbacks
    ros::spin();

    return 0;
}