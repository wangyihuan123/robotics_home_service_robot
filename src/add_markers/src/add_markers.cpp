#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"


void displayVirtualObject(ros::Publisher marker_pub,
                          const char *info,
                          int duration,
                          float color_a,
                          float pos_x = 0.0, float pos_y = 0.0,
                          float o_x = 0.0, float o_y = 0.0, float o_z = 0.0, float o_w = 1.0
) {

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // display object in pick up zone for 5 second
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";  // Fixed Frame in the rviz.
    // Without this, rviz can't display the marker,
    // although topic visualization_marker still has the info
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = o_x;
    marker.pose.orientation.y = o_y;
    marker.pose.orientation.z = o_z;
    marker.pose.orientation.w = o_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = color_a;

    if (duration <= 0) {
        marker.lifetime = ros::Duration();
    } else {
        marker.lifetime = ros::Duration(duration);
    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    ROS_INFO(info);
    marker_pub.publish(marker);

    return;
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}


class Listener {
public:
    Listener(ros::Publisher publisher): marker_publisher(publisher){};

    void callback(const nav_msgs::Odometry::ConstPtr &msg) {
        float pickup_position_x = -2.0;
        float pickup_position_y = 0.0;
        float pickup_position_z = 0.0;

        float pickup_orientation_x = 0.0;
        float pickup_orientation_y = 0.0;
        float pickup_orientation_z = 0.0;
        float pickup_orientation_w = 1.0;

        float color_alpha;
        int duration;
//    int state = 0;

        if (fabs(msg->pose.pose.position.x - pickup_position_x) < 0.2 &&
            fabs(msg->pose.pose.position.y - pickup_position_y) < 0.2 &&
            fabs(msg->pose.pose.position.z - pickup_position_z) < 0.2) {
            // arrive pick up zone, hide the object
            if (object_state_ == 0) {
                color_alpha = 0.0; // transparency
                duration = 0;
                displayVirtualObject(this->marker_publisher, "Robot picked up the object", duration, color_alpha, -2);
                object_state_ = 1;
            }
        }

        // debug
//    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
//             msg->pose.pose.position.z);
//    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
//             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }

private:
    int object_state_ = 0;  // 0:init, 1:pick up, 2: drop off
    ros::Publisher marker_publisher;

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    int duration = 0; // seconds, 0: forever
    float color_alpha = 1.0; // not transparent at all

    displayVirtualObject(marker_pub, "Robot is travelling to the pick up zone", duration, color_alpha, -2);

    // from subscriber tutorial:
    // http://wiki.ros.org/evarobot_odometry/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Odometry
    // http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    Listener listener(marker_pub);
    ros::Subscriber sub = n.subscribe("odom", 1000, &Listener::callback, &listener);
    ros::spin();


//    displayVirtualObject(marker_pub, "Robot is travelling to the pick up zone", duration, color_alpha, -2);
//    sleep(5);
//
//    // after object duration, hide the object for 5 seconds
//    ROS_INFO("Robot is travelling to the drop off zone");
//    sleep(5);
//
//    duration = 0; // object last forever
//    displayVirtualObject(marker_pub, "Robot drop off the object", duration, color_alpha, 1);

//    while (true) {
//        sleep(10);
//    }

    return 0;

}
