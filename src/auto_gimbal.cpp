#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <rdk_msgs/navigation.h>
#include <rdk_msgs/target.h>
#include "CppSerial.cpp"
#include <ros/master.h>
#include <sstream>


class SubscribeAndPublish
{
private:
  CppSerial ser;

  float X = 0;
  float Y = 0;
  float Xpoi = 200;
  float Ypoi = 0;
  float Heading = 0;

  ros::NodeHandle nh;
  ros::Subscriber navigationSub;
  ros::Subscriber targetSub;


public:
    SubscribeAndPublish() // This is the constructor
    {
      char portname[] = "/dev/ttyUSB0";
      ser.Open(portname, 115200);

      targetSub = nh.subscribe<rdk_msgs::target>("target_data", 5, &SubscribeAndPublish::target_callback, this);
      navigationSub = nh.subscribe("navigation_data", 5, &SubscribeAndPublish::navigation_callback, this);

    }

    int bound(int value, int mi, int ma)
    {
      int upper = std::max({mi, value});
      int lower = std::min({upper, ma});
      return lower;
    }

    int convertZeroPadedHexIntoByte(char *dataset, unsigned char *bytearray)
    {
    	int le = strlen(dataset);

      for(int j = 0; j < le; j++)
      {
        bytearray[j] = (dataset[j] < 0)?(dataset[j] + 256):dataset[j];
        //bytearray[j] = static_cast<unsigned char>(dataset[j]);
      }
      return 0;
    }

    void navigation_callback(const rdk_msgs::navigation msg)
    {
      X = msg.X;
      Y = msg.Y;
      Heading = msg.heading;
      return;
    }

    void target_callback(const rdk_msgs::target msg)
    {
      Xpoi = msg.Xpoi;
      Ypoi = msg.Ypoi;
      return;
    }

    void generateControl()
    {
      float peleng = atan2(Ypoi - Y, Xpoi - X);
      int gimbal_heading = round((peleng - Heading) * 180.0 / M_PI);
      float distance = sqrt(pow(Ypoi - Y, 2) + pow(Xpoi - X, 2));
      float optimal_distance = 5;  // meters
      int gimbal_pitch = -round((distance - optimal_distance) * 3);
      gimbal_heading = bound(gimbal_heading, -90, 90);
      gimbal_heading = (int) (gimbal_heading / 2);
      gimbal_pitch = bound(gimbal_pitch, -40, 20);

      char MSG[32] = {'\0'};
      memset(MSG, 0, sizeof(MSG));
      sprintf(MSG, "campos %d %d \n", gimbal_heading, gimbal_pitch);

      ser.FlushTransmit();
      //ser.Send((unsigned char*)MSG, sizeof(MSG));
      std::string das = std::string(MSG);
      ROS_INFO("Sent: %s\n", das.c_str());
      ser.SendString(das);
    }


};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_gimbal");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;

    // For cycling operation use
    ros::Rate rate(0.25); // ROS Rate at 20 Hz
    bool master_is_alive = true;
    while (ros::ok() && master_is_alive)
    {
      SAPObject.generateControl();

      if (!ros::master::check())
      {
        ROS_WARN_THROTTLE(5, "ROS Master is not found, shutting down...");
        master_is_alive = false;
      }
     ros::spinOnce();
     rate.sleep();
    }

  	return 0;
  }
