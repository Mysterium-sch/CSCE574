#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib>                    // Needed for rand()
#include <ctime>                      // Needed to seed random number generator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles

struct Pose
{
  double x;       // in simulated Stage units
  double y;       // in simulated Stage units
  double heading; // in radians
  ros::Time t;    // last received time

  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0){};

  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                      msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };
};

struct Laser
{
  constexpr static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
  constexpr static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
  constexpr static float PROXIMITY_RANGE_M = 2.0; // Should be smaller than  sensor_msgs::LaserScan::range_max
  constexpr static double FORWARD_SPEED_MPS = 1.0;
  constexpr static double ROTATE_SPEED_RADPS = M_PI / 8;
  enum FSM
  {
    FSM_MOVE_FORWARD,
    FSM_ROTATE
  };

  float d_safe;
  float alpha;
  float beta;
  float epsilon;
  float gamma;
  float k;
  float Fax;
  float Fay;
  float sum_forcesy;
  float sum_forcesx;
  std::vector<float> forcesx;
  std::vector<float> forcesy;
  ros::Time rotateStartTime;    // Start time of the rotation
  ros::Duration rotateDuration; // Duration of the rotation
  enum FSM fsm;
  bool isleader;

  Laser() : d_safe(1), alpha(0.5), beta(2), epsilon(0.05), gamma(10), k(0.75), sum_forcesx(0), sum_forcesy(0){};

  Laser(bool b) : d_safe(1), alpha(0.5), beta(2), epsilon(0.05), gamma(10), k(0.75), sum_forcesx(0), sum_forcesy(0), isleader(b) {};

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    float closestRange = msg->ranges[minIndex];
    float angle_mini = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_inc = msg->angle_increment;
    sum_forcesy = 0;
    sum_forcesx = 0;
    int count = 0;
    forcesx.clear();
    forcesy.clear();

    Fax = gamma;
    Fay = gamma;

    if(isleader) {

      	for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
				if (msg->ranges[currIndex] <closestRange) {
					closestRange = msg->ranges[currIndex];
				}
			}

			if(closestRange < 1.1) {
				fsm = FSM_ROTATE;
				float rotationTime = 2;
				rotateDuration = ros::Duration(rotationTime);
				rotateStartTime = ros::Time::now();

			} else {
				fsm = FSM_MOVE_FORWARD;
			}
    } else {

    for (float i = angle_mini; i < (angle_max - angle_inc); i = i + angle_inc)
    {
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
        
        for (auto& n : forcesy)
            sum_forcesy += n;

    }
  }
  };

class PotFieldBot
{
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle &nh, int robotID, int n,
              int l) : ID(robotID), numRobots(n),
                       leader(l)
  {
    // Initialize random time generator
    srand(time(NULL));

    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++)
    {
      pose.push_back(Pose());
      laser.push_back(Laser());
      laser[i].isleader = (leader==i);
    }
    for (int i = 0; i < numRobots; i++)
    {
      poseSubs.push_back(nh.subscribe("/robot_" +
                                          boost::lexical_cast<std::string>(i) +
                                          "/base_pose_ground_truth",
                                      1,
                                      &Pose::poseCallback, &pose[i]));
    }

    for (int i = 0; i < numRobots; i++)
    {
      laserSub.push_back(nh.subscribe("/robot_" +
                                          boost::lexical_cast<std::string>(i) +
                                          "/base_scan",
                                      1,
                                      &Laser::laserCallback, &laser[i]));
    }

    for (int i = 0; i < numRobots; i++)
    {
      commandPub.push_back(nh.advertise<geometry_msgs::Twist>("/robot_" +
                                                                  boost::lexical_cast<std::string>(i) +
                                                                  "/cmd_vel",
                                                              1));
    }

    for (int i = 0; i < numRobots; i++) {
      goalX.push_back(0);
      goalY.push_back(0);
    }
  };

  void move(double linearVelMPS, double angularVelRadPS, int i)
  {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub[i].publish(msg);
  };

  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin()
  {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz
    std::cout << "Starting program\n";

    while (ros::ok())
    { // Keep spinning loop until user presses Ctrl+C

      // Demo code: print each robot's pose
      for (int i = 0; i < numRobots; i++)
      {
        std::cout << std::endl;
        std::cout << i << "        ";
        std::cout << "Pose: " << pose[i].x << ", " << pose[i].y << ", " << pose[i].heading << std::endl;
        goalX[i] = pose[leader].x - pose[i].x;
        goalY[i] = pose[leader].y - pose[i].y;
        if (i == leader)
        {
          if (laser[i].fsm == laser[i].FSM_ROTATE)
          {
            if (ros::Time::now() < (laser[i].rotateStartTime + laser[i].rotateDuration))
            {
              move(0, ROTATE_SPEED_RADPS, i);
            }
            else
            {
              laser[i].fsm = laser[i].FSM_MOVE_FORWARD;
            }
          }
          else
          {
            move(FORWARD_SPEED_MPS, 0, i);
          }
        }
        else
        {
          laser[i].Fax *= goalX[i];
          laser[i].Fay *= goalY[i];
          laser[i].sum_forcesx += laser[i].Fax;
          laser[i].sum_forcesy += laser[i].Fay;

          float needed_ori = laser[i].k * atan2(laser[i].sum_forcesy, laser[i].sum_forcesx) - pose[i].heading;
          float w = (((needed_ori > M_PI) ? (needed_ori - 2 * M_PI) : (needed_ori)));

          float projected_force = laser[i].sum_forcesx * cos(pose[i].heading) + laser[i].sum_forcesy * sin(pose[i].heading);
          if (!(abs(goalX[i] + pose[i].x) <= d_safe && abs(goalY[i] - pose[i].y) <= d_safe))
          {
            if (laser[i].sum_forcesx * cos(pose[i].heading) > 0 && laser[i].sum_forcesy * sin(pose[i].heading) > 0)
            {
              float v = std::min(projected_force, (float)2.0);
              move(v, 0, i);
            }
            else if (projected_force == 0)
            {
              move(-2, 0, i);
            }
            else
            {
              move(0, w, i);
            }
          }
          else
          {
            this->move(0, 0, i);
          }
        }
      }

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep();    // Sleep for the rest of the cycle, to enforce the FSM loop rate
      }
    };

    // Tunable motion controller parameters
    constexpr static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
    constexpr static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
    constexpr static float PROXIMITY_RANGE_M = 2.0; // Should be smaller than  sensor_msgs::LaserScan::range_max
    constexpr static double FORWARD_SPEED_MPS = 1.0;
    constexpr static double ROTATE_SPEED_RADPS = M_PI / 8;

  protected:
    std::vector<ros::Publisher> commandPub; // Publisher to the current robot's velocity command topic
    std::vector<ros::Subscriber> laserSub;  // Subscriber to the current robot's laser scan topic
    std::vector<ros::Subscriber> poseSubs;  // List of subscribers to all robots' pose topics
    std::vector<Pose> pose;                 // List of pose objects for all robots
    std::vector<Laser> laser;
    std::vector<float> goalX;
    std::vector<float> goalY;
    int ID;        // 0-indexed robot ID
    int numRobots; // Number of robots, positive value
    int leader = 0;
    int d_safe = 2;
  };

  int main(int argc, char **argv)
  {
    int robotID = -1, numRobots = 0;
    double goalX, goalY;
    bool printUsage = false;
    int leader = 0;

    // Parse and validate input arguments
    if (argc <= 3)
    {
      printUsage = true;
    }
    else
    {
      try
      {
        robotID = boost::lexical_cast<int>(argv[1]);
        numRobots = boost::lexical_cast<int>(argv[2]);
        leader = boost::lexical_cast<int>(argv[3]);

        if (robotID < 0)
        {
          printUsage = true;
        }
        if (numRobots <= 0)
        {
          printUsage = true;
        }
        if (leader < 0 || leader > numRobots)
        {
          printUsage = true;
        }
      }
      catch (std::exception err)
      {
        printUsage = true;
      }
    }
    if (printUsage)
    {
      std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [leader]" << std::endl;
      return EXIT_FAILURE;
    }

    ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
    ros::NodeHandle n("robot_" + std::string(argv[1]));           // Create named handle "robot_#"
    PotFieldBot robbie(n, robotID, numRobots, leader);            // Create new random walk object
    robbie.spin();                                                // Execute FSM loop

    return EXIT_SUCCESS;
  };