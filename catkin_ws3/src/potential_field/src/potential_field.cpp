#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles
#include <tf/transform_listener.h>



class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy) {
    // Initialize random time generator
    srand(time(NULL));

    d_safe = 1;
    alpha = 0.5;
    beta = 2;
    epsilon = 0.05;
    gamma = 10;
    k = 0.75;
    velocity = 2;

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
      poseSub = nh.subscribe("base_pose_ground_truth", 1, \
      &PotFieldBot::poseCallback, this);
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation)+0.5*3.14;
  };
  


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (int i = 0; i < numRobots; i++) {

      // Calculate attractive forces
      float dx = goalX-x;
      float dy = goalY-y;
      float Fax = gamma*dx;
      float Fay = gamma*dy;

      float angle_mini = msg->angle_min;
      float angle_max = msg->angle_max;
      float angle_inc = msg->angle_increment;
      int count = 0;
      sum_forcesy=0;
      sum_forcesx=0;
      forcesx.clear();
      forcesy.clear();

      // calculate repulsive forces
      for(float i = angle_mini; i<(angle_max-angle_inc); i=i+angle_inc) {

          float dx = msg->ranges[count]*cos(i);
          float dy = msg->ranges[count]*sin(i);
          float d = sqrt(dx*dx + dy*dy);
          count = count + 1;

          if(d_safe+epsilon <  d&&  d< beta) {
            float f = alpha/(abs((d-d_safe)*(d-d_safe)));
            forcesx.push_back(f*cos(i));
            forcesy.push_back(f*sin(i));
          } else if (d< d_safe+epsilon) {
            float f = alpha/(epsilon*epsilon);
            forcesx.push_back(f*cos(i));
            forcesy.push_back(f*sin(i));
          } else {
            forcesx.push_back(0);
            forcesy.push_back(0);
          }
      }

      // calculate total forces
        for (auto& n : forcesx)
            sum_forcesx += n;

        sum_forcesx += Fax;
        
        for (auto& n : forcesy)
            sum_forcesy += n;

        sum_forcesy += Fay;

    }
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove demo code, compute potential function, actuate robot
      for (int i = 0; i < numRobots; i++) {
        float needed_ori = k*atan2(sum_forcesy, sum_forcesx) - heading;
        w = (((needed_ori > M_PI) ? (needed_ori-2*M_PI) : (needed_ori)));

      float projected_force = sum_forcesx * cos(heading) + sum_forcesy * sin(heading);
      if (!(abs(goalX + x) <= d_safe && abs(goalY - y) <= d_safe)) {
      if(sum_forcesx * cos(heading) > 0 && sum_forcesy * sin(heading) >0) {
        v =  std::min(projected_force, velocity);
          move(v, 0);
          std::cout << "X total: " << sum_forcesx *cos(heading)<< " Y total: " << sum_forcesy* sin(heading) << "\n";
           std::cout << "Linear Velcoity: " << v << " Angular Velocity: " << w << " Heading: " << heading << " Pose: (" << x << "," << y << ")\n";
        } else if (projected_force == 0){
          move(-2, 0);
           std::cout << "Linear Velcoity: " << -2 << " Angular Velocity: " << 0<< " Heading: " << heading << " Pose: (" << x << "," << y << ")\n";
        } else {
        move(0, w);
        std::cout << "X total: " << sum_forcesx*cos(heading) << " Y total: " << sum_forcesy* sin(heading) << "\n";
        std::cout << "Linear Velcoity: " << 0 << " Angular Velocity: " << w << " Heading: " << heading << " Pose: (" << x << "," << y << ")\n";
      } 
      } else {
        this->move(0,0);
      }
      }

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };

  // Tunable motion controller parameters
  constexpr static float FORWARD_SPEED_MPS = 1.0;
  constexpr static double ROTATE_SPEED_RADPS = M_PI/2;
  constexpr static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
  constexpr static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // List of subscribers to all robots' pose topics
  int ID; // 0-indexed robot ID
  int numRobots; // Number of robots, positive value
  double goalX, goalY; // Coordinates of goal
  float x;
  float y;
  float heading;
  float d_safe;
  float alpha;
  float beta;
  float epsilon;
  float gamma;
  float k;
  float sum_forcesy;
  float sum_forcesx;
  float velocity;
  std::vector<float> forcesx;
  std::vector<float> forcesy;
  float w;
  float v;

};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalX *= -1;
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  ros::NodeHandle n; // Create named handle "robot_#"
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
