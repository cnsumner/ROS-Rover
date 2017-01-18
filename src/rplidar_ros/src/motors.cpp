#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Float32.h>

using namespace std;

int fd;
bool drive = true;
bool stop = false;

void sendCommand(unsigned char data)
{
    if (stop || !digitalRead(3) || !digitalRead(12) || !digitalRead(13) || !digitalRead(14))
    {
	data = 0x00;

	if (!digitalRead(3))
	{
	    data = 0x80;
            ROS_INFO("Stopped Forward");
	}
	else if (!digitalRead(12))
	{
	    data = 0x30;	
            ROS_INFO("Stopped - reverse");
	}

    	if (!digitalRead(13))
	{
	    data |= 0x0C;
            ROS_INFO("Stopped - right");
	}
	else if (!digitalRead(14))
	{
            ROS_INFO("Stopped - left");
	    data |= 0x03;
	}

        stop = true;
    }
    else
    {
        data = 0x80 | (0x0F & data);
    }

    int result;

    result = wiringPiI2CWrite(fd, data);

    if (result == -1)
        ROS_INFO("Error: Err number is: %d", errno); 

}

void avoidCallback(const std_msgs::Float32::ConstPtr& cmd)
{
//if (cmd->data == 0)
    if (true)
        sendCommand(0x00);
        return;
    
    if (cmd->data < -80)
    {
        sendCommand(0x41);
        // turn right slightly
    }
    else if (cmd->data < -40)
    {
        sendCommand(0x42);
        // turn right middle
    }
    else if (cmd->data < 0)
    {
        sendCommand(0x43);
        // turn right
    }
    else if (cmd->data > 80)
    {
        sendCommand(0x44);
        //turn left
    }
    else if (cmd->data > 40)
    {
        sendCommand(0x48);
        // turn left middle
    }
    else if (cmd->data > 0)
    {
        sendCommand(0x4C);
        //turn left slightly
    }
}

void driveCallback(const std_msgs::Byte::ConstPtr& cmd)
{
    ROS_INFO("drive");
    if (!drive)
    {
        ROS_INFO("Passing");
        return;
    }

    sendCommand(cmd->data);
}

int main(int argc, char **argv)
{
    wiringPiSetup();
    fd = wiringPiI2CSetup(0x10);

    // Set up the emergency stop pins
    pinMode( 3, INPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    pinMode(14, INPUT);

    pullUpDnControl( 3, PUD_UP);
    pullUpDnControl(12, PUD_UP);
    pullUpDnControl(13, PUD_UP);
    pullUpDnControl(14, PUD_UP);

    try {
        ros::init(argc, argv, "motor_client");
        ros::NodeHandle n;
    
        ros::Subscriber sub = n.subscribe<std_msgs::Byte>("/motors", 1000, driveCallback);
        ros::Subscriber sub2 = n.subscribe<std_msgs::Float32>("/motors2", 1000, avoidCallback);
    
        ros::spin();
    }
    catch (int e)
    {
        wiringPiI2CWrite(fd, 0x00);
    }

    return 0;
}
