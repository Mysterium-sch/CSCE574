/*
 * Code modified by Peyton Chandarana
 * for CSCE 574 Homework 1
 * Spring 2024
 */

#include <cstdlib> // Needed for rand()
#include <ctime>   // Needed to seed random number generator with a time value

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <vector>

class RandomWalk
{
public:
  // Construst a new RandomWalk object and hook up this ROS node
  // to the simulated robot's velocity control and laser topics
  RandomWalk(ros::NodeHandle &nh)
      : rotateStartTime(ros::Time::now()),
        rotateDuration(0.f)
  {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the simulated robot's velocity command
    // topic (the second argument indicates that if multiple command messages
    // are in the queue to be sent, only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    poseSub = nh.subscribe("base_pose_ground_truth", 1,
                           &RandomWalk::poseCallback, this);

    // Subscribe to the simulated robot's laser scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
  };

  float d_safe = 1;
  float alpha = 0.5;
  float beta = 2;
  float epsilon = 0.05;
  float gamma = 10;
  float k = 0.75;
  float velocity = 2;
  float xd=5;
  float ideal_dis = 2;

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS)
  {
    geometry_msgs::Twist
        msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  // Process the incoming laser scan message
  void commandCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    float angle_mini = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_inc = msg->angle_increment;
    forcesx = 0;
    forcesy = 0;
        unsigned int minIndex =
        ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex =
        ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int closestIndex = minIndex;
    // Start at the minimum index
    closestRange = msg->ranges[minIndex];

    // calculate attractive force
      float dx = 1;
      float dy = 1;
      float Fax = gamma*dx;
      float Fay = gamma*dy;

    int count = 0;
    for (float i = angle_mini; i < (angle_max - angle_inc); i = i + angle_inc)
    {
      if (msg->ranges[count] < closestRange)
      {
        closestRange = msg->ranges[count];
        closestAngle = i;
      

      //repulsive forces
      float dx = msg->ranges[count] * cos(i);
      float dy = msg->ranges[count] * sin(i);
      float d = sqrt(dx * dx + dy * dy);
      if(d >= ideal_dis) {
        d -= ideal_dis;
        i += M_PI/2;
      }
      count = count + 1;

      if (d_safe + epsilon < d && d < beta)
      {
        float f = alpha / (abs((d - d_safe) * (d - d_safe)));
        forcesx = f * cos(i);
        forcesy = (f * sin(i));
      }
      else if (d < d_safe + epsilon)
      {
        float f = alpha / (epsilon * epsilon);
        forcesx=(f * cos(i));
        forcesy=(f * sin(i));
      }
      else
      {
        forcesx=0;
        forcesy=0;
      }
      }

      count++;
    }
  };

  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading = tf::getYaw(msg->pose.pose.orientation) + 0.5 * 3.14;
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin()
  {
    bool found = false;
    bool start = true;
    ros::Rate rate(10); // Specify the FSM loop rate in Hz
    while (ros::ok())
    { // Keep spinning loop until user presses Ctrl+C
      /////////////////////// ANSWER CODE BEGIN ///////////////////
      std::cout << "Closest range: " << closestRange << " At angle: " << closestAngle << " heading " << heading << "\n";

      // When the program starts up, go until find a wall then rotate to be perpendicular
      if (start)
      {
        if (closestRange < xd)
        {
          found = true;
          // Now that I have found a wall, I always want to be perpendicular to the wall at the closest range
          if (!(closestAngle > -1.45 && closestAngle < -1.35))
          {
            move(0, ROTATE_SPEED_RADPS);
          }
          else
          {
            start = false;
          }
        }
        else if (!found)
        {
          // Let's move forward untl we find the wall
          move(FORWARD_SPEED_MPS, 0);
        }
      }
      else
      {
        // potential field time
        float needed_ori = k * atan2(sum_forcesy, sum_forcesx) - heading;
        w = (((needed_ori > M_PI) ? (needed_ori - 2 * M_PI) : (needed_ori)));

        float projected_force = sum_forcesx * cos(heading) + sum_forcesy * sin(heading);

          if (closestAngle <= 0.18 || closestAngle >= -0.18) {
            while(!(closestAngle <= 0.18 || closestAngle >= -0.18)) {
              move(0, w);
            }
          } else {
            move(velocity, w);
          }
        }
      }
      /////////////////////// ANSWER CODE END ///////////////////
      ros::spinOnce(); // Need to call this function often to allow ROS to
                       // process incoming messages
      rate.sleep();    // Sleep for the rest of the cycle, to enforce the FSM loop
                       // rate
    }

  // Tunable parameters
  // TODO: tune parameters as you see fit
  constexpr static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
  constexpr static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
  constexpr static float PROXIMITY_RANGE_M =
      1.0; // Should be smaller than  sensor_msgs::LaserScan::range_max
  constexpr static double FORWARD_SPEED_MPS = 1.0;
  constexpr static double BACKWARD_SPEED_MPS = -0.5;
  constexpr static double ROTATE_SPEED_RADPS = M_PI / 10;

protected:
  ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
  ros::Subscriber laserSub;  // Subscriber to the simulated robot's laser scan topic
  ros::Subscriber poseSub;   // Subscriber to the current robot's ground truth pose topic

  ros::Time rotateStartTime;    // Start time of the rotation
  ros::Duration rotateDuration; // Duration of the rotation
  // Added for detecting stuck state:
  ros::Time reverseStartTime;
  ros::Duration reverseDuration;
  unsigned int prevClosestIndex;
  float prevClosestRange;
  float closestRange = 5;
  unsigned int direction = 0;

  float closestAngle;
  float sum_forcesy;
  float sum_forcesx;
  float forcesx;
  float forcesy;
  float w;
  float v;
  double x;       // in simulated Stage units, + = East/right
  double y;       // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
};

int main(int argc, char **argv)
{
  ros::init(argc, argv,
            "random_walk"); // Initiate new ROS node named "random_walk"
  ros::NodeHandle n;
  RandomWalk walker(n); // Create new random walk object
  walker.spin();        // Execute FSM loop
  return 0;
};