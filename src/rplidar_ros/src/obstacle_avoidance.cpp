#include <iostream>
#include <iostream>
#include <ros/ros.h>
#include <rplidar.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define SIZE 10000

float dist_data[360];
float angl_data[360];
float bubble[360];

ros::Publisher motor_pub;
std_msgs::Float32 m_cmd;

class Stack
{
    private:
        int top;
        float S[SIZE];

    public:
        Stack(){ top = -1; }
        ~Stack();

        void push(float angl)
        {
            if(isfull())
                ROS_INFO("Stack overflow");
            else
            {
                top++;
                S[top] = angl;
            }
        }

        void pop()
        {
            float ang;

            if(isempty())
                ROS_INFO("Stack empty");
            else
                top--;
        }

        float top_element()
        {
            return S[top];
        }

        bool isfull()
        {
            if(top >= SIZE)
                return true;
            else
                return false;

        }

        bool isempty()
        {
            if(top == -1)
                return true;
            else 
                return false;
        }

       // void print()
       // {
       //     for(int i = 0; i<=top; i++)
       //         ROS_INFO("%f", S[i]);
       // }
};


Stack *mystack = new Stack();


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
            }
        }
               
    }

    
    float rebound = 0;
    
    if (obj_found)
    {
        rebound = aD_sum/D_sum;
        //ROS_INFO("Rebound: %f, aDs: %f, Ds: %f ", rebound, aD_sum, D_sum);
        ROS_INFO("Rebound degrees: %f", RAD2DEG(rebound));
        
        //m_cmd.data = 0x70;
        
        mystack->push(rebound);

        m_cmd.data = rebound;
        motor_pub.publish(m_cmd);

        //ros::Duration(5.0);
    }
    else
    {
        
        //m_cmd.data = 0x0F;
        if(mystack->isempty())
            mystack->push(rebound);
        else
            rebound = mystack->top_element();   
        
        m_cmd.data =  0 - rebound;
        motor_pub.publish(m_cmd);

        mystack->pop();
    }
    

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
    
    motor_pub = nh.advertise<std_msgs::Float32>("/motors2", 1000);
    ros:: Subscriber sub = nh.subscribe("/scan", 1000, lidarCallback);

    ros::spin();
    return 0;
}

