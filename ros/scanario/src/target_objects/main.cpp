#include <ros/ros.h>

#include "node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_objects");

    ros::NodeHandle n;
    
    Node node(n);

    ros::spin();

    return 0;
}
