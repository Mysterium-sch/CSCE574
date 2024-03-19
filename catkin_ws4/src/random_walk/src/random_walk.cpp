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

  void translate(double d) {
    double t = lin_time;

    move(1, 0);
    rospy.sleep(t);
    move(0,0);
  };


  void rotate_rel(double angle) {
    double t = rot_time;

    move(0, 0.5);
    rospy.sleep(t);
    move(0,0);
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    rate = rospy.Rate(10);
    translate(distance);
    rospy.sleep(30); // sleep for 30 seconds
    rotate_rel(angle);
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
  double distance = 0;
  double angle = 0;
  double line_time = 0;
  double rot_time = 0;

  bool printUsage=false;
  if(argc <= 4) {
    printUsage = true;
  } else {
    try {
    distance = boost::lexical_cast<double>(argv[1]);
    angle = boost::lexical_cast<double>(argv[2]);
    lin_time = boost::lexical_cast<double>(argv[3]);
    rot_time = boost::lexical_cast<double>(argv[4]);
    } catch (std::exection err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Robot will go " << distance << " meters and turn " << angle << " radians\n";
  }

  ros::init(argc, argv,
            "random_walk");  // Initiate new ROS node named "random_walk"
  ros::NodeHandle n;
  RandomWalk walker(n);  // Create new random walk object
  walker.spin();         // Execute FSM loop
  return 0;
};
