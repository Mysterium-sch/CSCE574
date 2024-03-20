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
  RandomWalk(ros::NodeHandle& nh, double d, double a, double lt, double rt)
      : distance(d), angle(a), line_time(lt), rot_time(rt),
        rotateStartTime(ros::Time::now()),
        rotateDuration(0.f) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the simulated robot's velocity command
    // topic (the second argument indicates that if multiple command messages
    // are in the queue to be sent, only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

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
    double t = line_time;
    t *= d;
    ros::Time rotateStartTime = ros::Time::now();
    ros::Duration rotateDuration = ros::Duration(t);
    while (ros::Time::now() < (rotateStartTime+rotateDuration)) {
				 move(1, 0);
		}
    move(0,0);
  };


  void rotate_rel_pos(double angle) {
    angle /=90;
    double t = rot_time;
    t *= angle;
    ros::Time rotateStartTime = ros::Time::now();
    ros::Duration rotateDuration = ros::Duration(t);
    while (ros::Time::now() < (rotateStartTime+rotateDuration)) {
				 move(0, M_PI/4);
		}
    move(0,0);
  };

    void rotate_rel_neg(double angle) {
    angle /=90;
    double t = rot_time;
    t *= angle;
    ros::Time rotateStartTime = ros::Time::now();
    ros::Duration rotateDuration = ros::Duration(t);
    while (ros::Time::now() < (rotateStartTime+rotateDuration)) {
				 move(0, -M_PI/4);
		}
    move(0,0);
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30);
    translate(distance);
    ros::Duration(10.0).sleep(); 
    if(angle < 0) {
      rotate_rel_neg(angle);
    } else {
      rotate_rel_pos(angle);
    }
  };

 protected:
   double distance = 0;
  double angle = 0;
  double line_time = 0;
  double rot_time = 0;
  ros::Publisher
      commandPub;  // Publisher to the simulated robot's velocity command topic
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
  double line_time = 2;
  double rot_time = 3.45;
  bool printUsage=false;
  if(argc <= 2) {
    printUsage = true;
  } else {
    try {
    distance = boost::lexical_cast<double>(argv[1]);
    angle = boost::lexical_cast<double>(argv[2]);
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Robot will go " << distance << " meters and turn " << angle << " radians\n";
  }

  ros::init(argc, argv,
            "random_walk");  // Initiate new ROS node named "random_walk"
  ros::NodeHandle n;
  RandomWalk walker(n, distance, angle, line_time, rot_time);  // Create new random walk object
  walker.spin();         // Execute FSM loop
  return 0;
};
