/*
 * Code modified by Peyton Chandarana
 * for CSCE 574 Homework 1
 * Spring 2024
 */

#include <cstdlib>  // Needed for rand()
#include <ctime>    // Needed to seed random number generator with a time value

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class RandomWalk {
 public:
  // Construst a new RandomWalk object and hook up this ROS node
  // to the simulated robot's velocity control and laser topics
  RandomWalk(ros::NodeHandle& nh)
      : fsm(FSM_MOVE_FORWARD),
        rotateStartTime(ros::Time::now()),
        rotateDuration(0.f) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the simulated robot's velocity command
    // topic (the second argument indicates that if multiple command messages
    // are in the queue to be sent, only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the simulated robot's laser scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist
        msg;  // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  // Process the incoming laser scan message
  void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    unsigned int minIndex =
        ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex =
        ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int closestIndex = minIndex;
    // Start at the minimum index
    float closestRange = msg->ranges[minIndex];
    // Loop through all the indices and find the the closestRange up until the
    // max index
    for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex;
         currIndex++) {
      if (msg->ranges[currIndex] < closestRange) {
        closestRange = msg->ranges[currIndex];
        closestIndex = currIndex;
      }
    }
    if (this->fsm == this->FSM_MOVE_FORWARD) {
      // ROS_INFO_STREAM("Prev Vals: " << this->prevClosestIndex << ", " <<
      // this->prevClosestRange); ROS_INFO_STREAM("Curr Vals: " << closestIndex
      // << ", " << closestRange);
      if (closestRange == this->prevClosestRange &&
          closestIndex == this->prevClosestIndex) {
        // Stuck
        this->fsm = this->FSM_STUCK;
        this->reverseStartTime = ros::Time::now();
        float reverseTime = (0.5 + (rand() % 4)) / (1.0 + (rand() % 8));
        this->reverseDuration = ros::Duration(reverseTime);
        ROS_INFO_STREAM("SET REVERSE TO: " << reverseTime);
      } else if (closestRange < this->PROXIMITY_RANGE_M &&
                 this->fsm != this->FSM_ROTATE &&
                 this->fsm != this->FSM_STUCK) {
        // Turn
        this->fsm = this->FSM_ROTATE;
        this->direction = rand() % 2;
        this->rotateStartTime = ros::Time::now();
        float rotationTime = (0.5 + (rand() % 4)) / (1.0 + (rand() % 8));
        this->rotateDuration = ros::Duration(rotationTime);
        ROS_INFO_STREAM("SET ROTATION TO: " << rotationTime);
      }
    }
    this->prevClosestIndex = closestIndex;
    this->prevClosestRange = closestRange;
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(10);  // Specify the FSM loop rate in Hz
    while (ros::ok()) {  // Keep spinning loop until user presses Ctrl+C
      /////////////////////// ANSWER CODE BEGIN ///////////////////
      if (this->fsm == this->FSM_ROTATE &&
          (ros::Time::now() - this->rotateStartTime) <= this->rotateDuration) {
        this->move(0, pow(-1.0, this->direction) * this->ROTATE_SPEED_RADPS);
      } else if (this->fsm == this->FSM_STUCK &&
                 (ros::Time::now() - this->reverseStartTime) <=
                     this->reverseDuration) {
        this->move((rand() % 3) * this->BACKWARD_SPEED_MPS,
                   1.0 / (1.0 + rand() % 4));
      } else {
        this->fsm = this->FSM_MOVE_FORWARD;
        this->move(this->FORWARD_SPEED_MPS, 0);
      }
      /////////////////////// ANSWER CODE END ///////////////////
      ros::spinOnce();  // Need to call this function often to allow ROS to
                        // process incoming messages
      rate.sleep();  // Sleep for the rest of the cycle, to enforce the FSM loop
                     // rate
    }
  };

  enum FSM { FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STUCK };

  // Tunable parameters
  // TODO: tune parameters as you see fit
  constexpr static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
  constexpr static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
  constexpr static float PROXIMITY_RANGE_M =
      1.0;  // Should be smaller than  sensor_msgs::LaserScan::range_max
  constexpr static double FORWARD_SPEED_MPS = 1.0;
  constexpr static double BACKWARD_SPEED_MPS = -0.5;
  constexpr static double ROTATE_SPEED_RADPS = M_PI / 2;

 protected:
  ros::Publisher
      commandPub;  // Publisher to the simulated robot's velocity command topic
  ros::Subscriber
      laserSub;  // Subscriber to the simulated robot's laser scan topic
  enum FSM fsm;  // Finite state machine for the random walk algorithm
  ros::Time rotateStartTime;     // Start time of the rotation
  ros::Duration rotateDuration;  // Duration of the rotation
  // Added for detecting stuck state:
  ros::Time reverseStartTime;
  ros::Duration reverseDuration;
  unsigned int prevClosestIndex;
  float prevClosestRange;
  unsigned int direction = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv,
            "random_walk");  // Initiate new ROS node named "random_walk"
  ros::NodeHandle n;
  RandomWalk walker(n);  // Create new random walk object
  walker.spin();         // Execute FSM loop
  return 0;
};
