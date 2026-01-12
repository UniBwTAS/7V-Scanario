#pragma once

#include <ros/ros.h>
#include <object_msgs/Objects.h>

class Node
{

public:
    Node(ros::NodeHandle& node_handle);
    ~Node();


private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_syncedObjects_;
    ros::Publisher 	publisher_marker_;

private:
    // ROS data reception callbacks
    void rosCallback_syncedObjects(const object_msgs::Objects::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_objectsInput;
        std::string topic_markerOutput;
    }   configuration_;
};
