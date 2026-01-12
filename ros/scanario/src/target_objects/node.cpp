#include "node.h"
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#define DEBUG \
    if (0)    \
    ROS_INFO

Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle)
{
    /// Parameter
    // Topics
    node_handle.param<std::string>(
        "topic_objectsInput", configuration_.topic_objectsInput, "objects");
    node_handle.param<std::string>("topic_markerOutput", configuration_.topic_markerOutput, "objects_marker");

    /// Subscribing & Publishing
    subscriber_syncedObjects_ =
        ros_handle_.subscribe(configuration_.topic_objectsInput, 100, &Node::rosCallback_syncedObjects, this);
    publisher_marker_ = ros_handle_.advertise<visualization_msgs::Marker>(configuration_.topic_markerOutput, 100);
}

Node::~Node()
{
}

void Node::rosCallback_syncedObjects(const object_msgs::Objects::ConstPtr& msg)
{
    // Paint 3D Object Models (Scans)
    auto publish_mesh = [this, msg](object_msgs::Object const* obj, const std::string& mesh_path)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "utm";
        marker.header.stamp    = msg->header.stamp;

        marker.ns    = "mesh";
        marker.id    = obj->object_id;
        marker.type  = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.frame_locked = true;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;

        marker.pose.position.x = obj->pose.x;
        marker.pose.position.y = obj->pose.y;
        marker.pose.position.z = obj->pose.z;

        tf2::Quaternion q_utm;
        q_utm.setRPY(obj->pose.roll, obj->pose.pitch, obj->pose.yaw);
        marker.pose.orientation = tf2::toMsg(q_utm);

        marker.mesh_resource = mesh_path;
        marker.mesh_use_embedded_materials = 1;

        publisher_marker_.publish(marker);
    };

    // Paint 3D Bounding Boxes
    auto publish_bbox = [this, msg](object_msgs::Object const* obj)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "utm";
        marker.header.stamp    = msg->header.stamp;

        marker.ns    = "bounding_box";
        marker.id    = obj->object_id;
        marker.type  = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.frame_locked = true;

        marker.pose.position.x = obj->pose.x;
        marker.pose.position.y = obj->pose.y;
        marker.pose.position.z = obj->pose.z;

        tf2::Quaternion q_utm;
        q_utm.setRPY(obj->pose.roll, obj->pose.pitch, obj->pose.yaw);
        marker.pose.orientation = tf2::toMsg(q_utm);

        marker.scale.x = 0.05;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        const double xf =  obj->dimension.length_to_front;
        const double xr = obj->dimension.length_to_rear;
        const double yl =  obj->dimension.width_to_left;
        const double yr = obj->dimension.width_to_right;
        const double zu =  obj->dimension.height_to_upper;
        const double zb = obj->dimension.height_to_bottom;

        geometry_msgs::Point flu, fru, fld, frd; // front-left/up/down, front-right/up/down
        geometry_msgs::Point rlu, rru, rld, rrd; // rear-left/up/down, rear-right/up/down

        // Front (x = xf)
        flu.x = xf; flu.y = yl; flu.z = zu; // front, left, upper
        fru.x = xf; fru.y = yr; fru.z = zu; // front, right, upper
        fld.x = xf; fld.y = yl; fld.z = zb; // front, left, lower
        frd.x = xf; frd.y = yr; frd.z = zb; // front, right, lower

        // Back (x = xr)
        rlu.x = xr; rlu.y = yl; rlu.z = zu; // rear, left, upper
        rru.x = xr; rru.y = yr; rru.z = zu; // rear, right, upper
        rld.x = xr; rld.y = yl; rld.z = zb; // rear, left, lower
        rrd.x = xr; rrd.y = yr; rrd.z = zb; // rear, right, lower

        auto add_edge = [&marker](const geometry_msgs::Point& a, const geometry_msgs::Point& b)
        {
            marker.points.push_back(a);
            marker.points.push_back(b);
        };

        add_edge(flu, fru);
        add_edge(fru, rru);
        add_edge(rru, rlu);
        add_edge(rlu, flu);

        add_edge(fld, frd);
        add_edge(frd, rrd);
        add_edge(rrd, rld);
        add_edge(rld, fld);

        add_edge(flu, fld);
        add_edge(fru, frd);
        add_edge(rlu, rld);
        add_edge(rru, rrd);

        publisher_marker_.publish(marker);
    };

    // Vehicle-specific processing
    for (int i = 0; i < msg->objects.size(); i += 1)
    {
        const object_msgs::Object& obj = msg->objects.at(i);

        if (obj.object_id == 41953) // E87
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/e87.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 41952) // i10
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/i10.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 220) // A6
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/a6.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 40786) // Tiguan
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/tiguan.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 41948) // Crafter
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/crafter.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 5416) // STS
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/streetscooter.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
        else if (obj.object_id == 40784) // Q8
        {
            std::string path = "file://" + ros::package::getPath("scanario") + "/meshes/q8.dae";
            publish_mesh(&obj, path);
            publish_bbox(&obj);
        }
    }
}
