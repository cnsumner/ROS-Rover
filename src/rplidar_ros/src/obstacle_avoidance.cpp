#include <ros/ros.h>
#include <rplidar.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Float32.h>

#define RAD2DEG(x) ((x)*180./M_PI)

float dist_data[360];
float angl_data[360];
float bubble[360];

ros::Publisher motor_pub;
ros::Publisher base_pub;
std_msgs::Float32 m_cmd;
geometry_msgs::Twist bcmd;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static int angle;
    float delta_t = scan->scan_time;
    float vel = 40.5;
   
    float aD_sum=0.0;
    float D_sum=0.0;

    bool obj_found = false;

    for(int i = 0; i < 360; i++)
    {
        float radians = scan->angle_min + scan->angle_increment*i;
        float degree = RAD2DEG(radians);
        
         if(degree >= 90 || degree <= -90)
         {
            dist_data[i] = scan->ranges[i];
            angl_data[i] = radians;
          
            if(dist_data[i] < bubble[i])
            {                
                obj_found = true;

                aD_sum += angl_data[i]*dist_data[i];
                D_sum += dist_data[i];
                ROS_INFO("degree: %f, dist: %f", degree, dist_data[i]);

                // TODO::move the ifs to the outside after using the rebound angle to calculate the trajectory
                //if(angl_data[i] >= 90)
                //{
                //    ROS_INFO("Turning left");
                //    m_cmd.data = 0x0C;
                //    motor_pub.publish(m_cmd);
                //}
                //if(angl_data[i] <= -90)
                //{
                //    ROS_INFO("Turning right");
                //    m_cmd.data = 0x03;
                //    motor_pub.publish(m_cmd);
                //}
            }
        }
               
    }


    if (obj_found)
    {
        float rebound = aD_sum/D_sum;
        //ROS_INFO("Rebound: %f, aDs: %f, Ds: %f ", rebound, aD_sum, D_sum);
        ROS_INFO("Rebound degrees: %f", RAD2DEG(rebound));
        
        //m_cmd.data = 0x70;
        m_cmd.data = rebound;
        motor_pub.publish(m_cmd);
    }
    else
    {
        float rebound = 0;
        //m_cmd.data = 0x0F;
        m_cmd.data = rebound;
        motor_pub.publish(m_cmd);
    }
    
    //bcmd.angular.z = rebound*M_PI/180;
    //base_pub.publish(bcmd);
}

int main(int argc, char** argv)
{
    for (int i=0; i<180; i++)
    {
        if (i>89)
        {
            bubble[i] = 0;
            bubble[359 - i] = 0;
        }
        else if (i > 29)
        {
            bubble[i] = (359 - i) * 0.5 / 329;
            bubble[359 - i] = (359 - i) * 0.5 / 329;
        }
        else
        {
            bubble[i] = 0.5;
            bubble[359 - i] = 0.5;
        }
    }
    ros::init(argc, argv, "obstacle_avoidance");

    ros::NodeHandle nh;
    //base_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel, 1");
    
    motor_pub = nh.advertise<std_msgs::Float32>("/motors2", 1000);
    ros:: Subscriber sub = nh.subscribe("/scan", 1000, lidarCallback);

    ros::spin();
    return 0;
}

