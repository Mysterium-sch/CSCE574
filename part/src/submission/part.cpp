#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct Part {
    float x;
    float y;
};

const int part_num = 100;
std::vector<Part> cloud(part_num, {0.0, 0.0});
std::vector<float> x_pos(part_num);
std::vector<float> y_pos(part_num);
const float n = 0.1;

void velocityCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
    float v = msg->linear.x;
    float w = msg->angular.z;

    for (auto& part : cloud) {
        part.x += v*cos(w);
        part.y += v*sin(w);
        part.x += n*(rand() / (RAND_MAX + 1.0));
        part.y += n*(rand() / (RAND_MAX + 1.0));
    }

    for (int i = 0; i < part_num; ++i) {
        x_pos[i] = cloud[i].x;
        y_pos[i] = cloud[i].y;
    }

    plt::clf();
    plt::scatter(x_pos, y_pos);
    plt::title("Particle Filter");
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::xlim(-150.0, 150.0);
    plt::ylim(-100.0, 100.0);
    plt::pause(0.01);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "part");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, velocityCommandCallback);

    plt::ion();

    ros::spin();

    return 0;
}

