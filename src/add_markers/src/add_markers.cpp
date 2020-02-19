#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "add_markers/AddMarkers.h" // also check this https://answers.ros.org/question/242427/how-to-use-a-service-defined-in-another-package/
#include "geometry_msgs/Pose.h"

class AddMarkers {
private:
    ros::NodeHandle n_;
    ros::Subscriber subscription_;
    ros::Publisher publisher_;
    ros::ServiceServer service_server_;
    geometry_msgs::Pose source_pose_;
    geometry_msgs::Pose destination_pose_;
    int robot_state_;
public:

    AddMarkers(geometry_msgs::Pose source_pose, geometry_msgs::Pose destination_pose) :
            source_pose_(source_pose), destination_pose_(destination_pose), robot_state_(0) {
        this->service_server_ = this->n_.advertiseService("add_markers", &AddMarkers::addMarkersCallback, this);
//        this->subscription_ = this->n_.subscribe("odom", 1000, &AddMarkers::odomCallback, this);  // no need odom any more 
        this->publisher_ = this->n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void setObjectDefault() {
        float duration = 0;
        float color_alpha = 1.0;
        displayVirtualObject( "Markers are ready", duration, color_alpha, this->source_pose_);
    }

    bool addMarkersCallback(add_markers::AddMarkers::Request &req,
                            add_markers::AddMarkers::Response &res) {
//        ROS_INFO(" request: %s", req.str_request.c_str());

        if (!req.str_request.compare("pickup")) {
            float color_alpha = 0.0; // transparency
            float duration = 0;
            this->displayVirtualObject("Robot picked up the object", duration, color_alpha, this->source_pose_);

            // wait for 5 second for simulate picking up action
            ROS_INFO("Picking up");
            ros::Duration(5).sleep();

            res.str_response = "done";
        } else if (!req.str_request.compare("dropoff")) {
            float color_alpha = 1.0;
            float duration = 0;
            this->displayVirtualObject("Robot dropped off the object", duration, color_alpha, this->destination_pose_);

            // wait for 5 second for simulate dropping off action
            ROS_INFO("Dropping off");
            ros::Duration(5).sleep();

            res.str_response = "done";
        } else {
            res.str_response = "what";
            ROS_INFO("Unknown request: %s", req.str_request.c_str());  // todo: warning
        }

        return true;
    }

    // don't need read odom from add_markers any more.
//    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//    }


    void displayVirtualObject(
            const char *info,
            int duration,
            float color_a,
            geometry_msgs::Pose pose
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
        marker.pose = pose;

        // default setting
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;

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
        while (this->publisher_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        ROS_INFO(info);
        this->publisher_.publish(marker);

        return;
    }


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    geometry_msgs::Pose pickup_pose, dropoff_pose;

    pickup_pose.position.x = -2.0;
    pickup_pose.position.y = 0;
    pickup_pose.position.z = 0;
    pickup_pose.orientation.x = 0;
    pickup_pose.orientation.y = 0;
    pickup_pose.orientation.z = 0;
    pickup_pose.orientation.w = 1.0;

    dropoff_pose.position.x = -3.0;
    dropoff_pose.position.y = 1.5;
    dropoff_pose.position.z = 0;
    dropoff_pose.orientation.x = 0;
    dropoff_pose.orientation.y = 0;
    dropoff_pose.orientation.z = 0;
    dropoff_pose.orientation.w = 1.0;

    AddMarkers addmarkers_node(pickup_pose, dropoff_pose);
    addmarkers_node.setObjectDefault();
    ros::spin();


    return 0;

}
