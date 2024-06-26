/*
 * Code modified by YOURNAME
 * for CSCE 574 Homework 1
 * Spring 2023
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class RandomWalk {
  public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	RandomWalk(ros::NodeHandle& nh):
 	fsm(FSM_MOVE_FORWARD),
	rotateStartTime(ros::Time::now()),
	rotateDuration(0.f) {
	// Initialize random time generator
		srand(time(NULL));

		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		// the queue to be sent, only the last command will be sent)
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
	};

	// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	};

	float check(const sensor_msgs::LaserScan::ConstPtr& msg) {
			unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			float closestRange = msg->ranges[minIndex];
			for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
				if (msg->ranges[currIndex] <closestRange) {
					closestRange = msg->ranges[currIndex];
				}
			}
		return closestRange;

	}

	// Process the incoming laser scan message
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		if (fsm == FSM_MOVE_FORWARD) {
			// Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
			//
			// NOTE: ideally, the following loop should have additional checks to ensure
			// 	that indices are not out of bounds, by computing:
			//
			//   -currAngle = msg->angle_min + msg->angle_increment*currIndex
			//
			//       and then ensuring that currAngle <= msg->angle_max
			unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			float closestRange = msg->ranges[minIndex];
			for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
				if (msg->ranges[currIndex] <closestRange) {
					closestRange = msg->ranges[currIndex];
				}
			}
			ROS_INFO_STREAM("Range: " << closestRange);
			
			// TODO: if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime,  
      			// and also choose a reasonable rotateDuration (keeping in mind of the value  // of ROTATE_SPEED_RADPS) 
			// 
			// HINT: you can obtain the current time by calling: 
			// 
			// - ros::Time::now() 
			// 
			// HINT: you can set a ros::Duration by calling: 
			// 
			// - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)  // 
			// HINT: you can generate a random number between 0 and 99 by calling:  // 
			// - rand() % 100 
			// 
			// see http://www.cplusplus.com/reference/clibrary/cstdlib/rand/ for more details  
      			/////////////////////// ANSWER CODE BEGIN /////////////////// 
			
			if(closestRange < 1.1) {
				fsm = FSM_ROTATE;
				float rotationTime = 10/rotate;
				rotateDuration = ros::Duration(rotationTime);
				if(rotationTime < 0.1) {
					rotateDuration = ros::Duration(0.1);
				}
				rotateStartTime = ros::Time::now();

			} else {
				fsm = FSM_MOVE_FORWARD;
				rotate = 1;
			}
			/////////////////////// ANSWER CODE END /////////////////// 
		}
	};
	
	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(10); // Specify the FSM loop rate in Hz
		while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C	
			// TODO: Either call:
			// 
			// - move(0, ROTATE_SPEED_RADPS); // Rotate right 
			// 
			// or 
			// 
			// - move(FORWARD_SPEED_MPS, 0); // Move foward 
			// 
			// depending on the FSM state; also change the FSM state when appropriate  
      			/////////////////////// ANSWER CODE BEGIN /////////////////// 
		if (fsm == FSM_ROTATE) {
			if (ros::Time::now() < (rotateStartTime+rotateDuration)) {
				move(0, ROTATE_SPEED_RADPS);
			} else {
				fsm = FSM_MOVE_FORWARD;
				rotate++;
			}
		} else {
			move(FORWARD_SPEED_MPS, 0);
		}

			/////////////////////// ANSWER CODE END /////////////////// 
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	};

	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};

	// Tunable parameters
  // TODO: tune parameters as you see fit 
	constexpr static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
	constexpr static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
	constexpr static float PROXIMITY_RANGE_M = 2.0; // Should be smaller than  sensor_msgs::LaserScan::range_max 
	constexpr static double FORWARD_SPEED_MPS = 1.0;
	constexpr static double ROTATE_SPEED_RADPS = M_PI/8;

  	protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	enum FSM fsm; // Finite state machine for the random walk algorithm
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
	int rotate = 1;
};

int main(int argc, char**argv) {
	ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	RandomWalk walker(n); // Create new random walk object
	walker.spin(); // Execute FSM loop
	return 0;
};
