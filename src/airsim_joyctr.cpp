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
#include <sensor_msgs/Joy.h>
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

#define XBB_A        0
#define XBB_B        1
#define XBB_X        2
#define XBB_Y        3
#define XBB_LEFT     4
#define XBB_RIGHT    5
#define XBB_VIEW     6
#define XBB_MENU     7

#define XBA_YAW      0
#define XBA_VZ       1
#define XBA_VY       3
#define XBA_VX       4
#define XBA_SVYAW    6
#define XBA_SVZ      7


using namespace std;
using namespace Eigen;


struct TeleopAirSim
{
    ros::NodeHandle nh;
    ros::Timer ctr_timer;

    ros::ServiceClient takeoff_srv_client;
    int takeoff_landing = 0;

    ros::Publisher vel_cmd_pub;
    airsim_ros_pkgs::VelCmd vel_cmd;

    ros::Subscriber joy_sub;
    sensor_msgs::Joy joystat;
    bool joystat_received = false;

    double scale = 2;

    // Constructor
    TeleopAirSim()
    {
        ros::NodeHandle nh("~");

        // Publish velocity command at 20Hz
        ctr_timer = nh.createTimer(ros::Duration(0.05), &TeleopAirSim::CtrTimerCb, this);

        // Subscribe to the gamepad topic
        joy_sub = nh.subscribe("/joy", 10, &TeleopAirSim::JoyCb, this);

        // Get param for the scale factor
        nh.param("scale", scale, 2.0);

        vel_cmd_pub = nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_0/vel_cmd_body_frame", 1);
        takeoff_srv_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_0/takeoff");
    }

    void CtrTimerCb(const ros::TimerEvent &event)
    {
        if(!joystat_received)
            return;

        // Basic control by the sticks
        vel_cmd.twist.linear.x = scale*joystat.axes[XBA_VX];
        vel_cmd.twist.linear.y = scale*joystat.axes[XBA_VY];
        vel_cmd.twist.linear.z = scale*joystat.axes[XBA_VZ];
        
        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = scale*joystat.axes[XBA_YAW];

        // printf("Hello: %f %f %f. \n",
        //         joystat.axes[XBA_VX], joystat.axes[XBA_VY], joystat.axes[XBA_VZ]
        //       );

        // Overwriting the yaw rate if there is special value on the button
        if (joystat.axes[XBA_SVYAW] == -1 || joystat.axes[XBA_SVYAW] == 1)
            vel_cmd.twist.angular.z = scale*joystat.axes[XBA_SVYAW];

        // Overwriting the z velocity if there is special value on the button
        if (joystat.axes[XBA_SVZ] == -1 || joystat.axes[XBA_SVZ] == 1)
            vel_cmd.twist.linear.z = scale*joystat.axes[XBA_SVZ];

        // Overwriting the x velocity if there are special values on the buttons
        int velx = joystat.buttons[XBB_Y] - joystat.buttons[XBB_A];
        if (velx != 0)
            vel_cmd.twist.linear.x = scale*velx;

        // Overwriting the y velocity if there are special values on the buttons
        int vely = joystat.buttons[XBB_X] - joystat.buttons[XBB_B];
        if (vely != 0)
            vel_cmd.twist.linear.y = scale*vely;

        vel_cmd_pub.publish(vel_cmd);
    }

    void JoyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        joystat = *msg;
        joystat_received = true;
    }  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_joyctr");

    TeleopAirSim teleop;

    // Start the callbacks
    ros::spin();

    return 0;
}