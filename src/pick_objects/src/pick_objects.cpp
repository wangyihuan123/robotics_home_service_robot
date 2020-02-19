#include <sstream> // for ostringstream
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include <functional> // std::bind
#include <boost/assign/list_of.hpp>
#include "std_msgs/String.h"
#include "add_markers/AddMarkers.h" // many thanks for this: https://answers.ros.org/question/242427/how-to-use-a-service-defined-in-another-package/
#include "geometry_msgs/Pose.h"


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

void debug_print(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}

enum class Home_Service_State {
    BEGIN = 0,
    ON_THE_WAY_TO_PICKUP_ZONE,
    START_PICKING_UP,
    FINISH_PICKING_UP,
    ON_THE_WAY_TO_DROPOFF_ZONE,
    START_DROPPING_OFF,
    FINISH_DROPPING_OFF,
    END
};
std::string robot_state_str_[] = {
        "BEGIN",
        "ON_THE_WAY_TO_PICKUP_ZONE",
        "START_PICKING_UP",
        "FINISH_PICKING_UP",
        "ON_THE_WAY_TO_DROPOFF_ZONE",
        "START_DROPPING_OFF",
        "FINISH_DROPPING_OFF",
        "END"};


class Listener {
public:
    Listener(MoveBaseClient *ac, ros::ServiceClient *sc,
            geometry_msgs::Pose source_pose,  geometry_msgs::Pose destination_pose)
            : ac_(ac), sc_(sc), robot_state_(Home_Service_State::BEGIN), distance_error(0.3), debug(false),
            source_pose_(source_pose), destination_pose_(destination_pose){
    };

    void setJob(geometry_msgs::Pose source_pose,  geometry_msgs::Pose destination_pose) {
        source_pose_ = source_pose;
        destination_pose_ = destination_pose;
    }

    void clear() {
        this->ac_->cancelAllGoals();
    }

    void movebaseDoneCallback(void) {
        ROS_INFO("movebaseDoneCallback");
        if (robot_state_ == Home_Service_State::ON_THE_WAY_TO_PICKUP_ZONE) {
            clear();
            startJob();
        } else if (robot_state_ == Home_Service_State::ON_THE_WAY_TO_DROPOFF_ZONE) {
            clear();
            finishJob();
        }
    }

    void move(const char *start_print, const char *end_print) {
        //////////////  send the goal  /////////////////
        move_base_msgs::MoveBaseGoal goal;

        // set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = this->goal_pose_;

        ROS_INFO("%s => x: %f, y: %f", start_print, this->goal_pose_.position.x, this->goal_pose_.position.y);

        // set doneCallback function
        // http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
        this->ac_->sendGoal(goal, boost::bind(&Listener::movebaseDoneCallback, this));

    }

    bool operateObject(const char *req) {
        add_markers::AddMarkers srv;
        srv.request.str_request = req;

//        ROS_INFO("str_request: %s", srv.request.str_request.c_str());
        if (this->sc_->call(srv)) {
            ROS_INFO("Response: %s", srv.response.str_response.c_str()); // todo: warning
            return true;
        } else {
            ROS_ERROR("Failed to call service add_markers");
            return false;
        }
    }

    void startJob(void) {
        // pickup the object:tell add_markers node to hide the object
        ROS_INFO("Picking up... (about 5 sec)");
        if (this->operateObject("pickup") == false) {
            // error? check or print debug?
//            debug_print(msg);
            return;
        }
        robot_state_ = Home_Service_State::FINISH_PICKING_UP;

        // set drop off goal
        goal_pose_ = destination_pose_;
        move("heading to drop off", "drop off zone arrived");
        robot_state_ = Home_Service_State::ON_THE_WAY_TO_DROPOFF_ZONE;
        printCurrentState();
    }

    void finishJob(void) {
        // pickup the object:tell add_markers node to show the object
        ROS_INFO("Dropping off... (about 5 sec)");
        if (this->operateObject("dropoff") == false) {
            // error? check or print debug?
            return;
        }
        robot_state_ = Home_Service_State::FINISH_DROPPING_OFF;
        printCurrentState();

    }

    void printCurrentState(void) {
        ROS_INFO("state: %s", robot_state_str_[static_cast<int>(robot_state_)].c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//        printCurrentState();

        switch (robot_state_) {
            case Home_Service_State::BEGIN : // beginning, set goal to pick up zone
                goal_pose_ = source_pose_;
                move("heading to pick up", "pick up zone arrived");
                this->robot_state_ = Home_Service_State::ON_THE_WAY_TO_PICKUP_ZONE;
                printCurrentState();
                break;
            case Home_Service_State::ON_THE_WAY_TO_PICKUP_ZONE : // on the way to pick up zone
                // debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_pose_.position.x) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_pose_.position.y) < this->distance_error) {
                    this->clear();  // whatever the goal has finished or not, clear all the actions,
                    // as according to the odom data, the robot already arrived the destination
                    startJob();
                }
                break;
            case Home_Service_State::ON_THE_WAY_TO_DROPOFF_ZONE : // on the way to drop off
//                debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_pose_.position.x) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_pose_.position.y) < this->distance_error) {
                    this->clear();
                    finishJob();
                }
                break;
            default:
                break;
        }

        return;
    }

private:
    MoveBaseClient *ac_;
    ros::ServiceClient *sc_;
    geometry_msgs::Pose goal_pose_;
    geometry_msgs::Pose source_pose_;
    geometry_msgs::Pose destination_pose_;
    Home_Service_State robot_state_;  // 0:init, 1:on the way to pick up zone, 2: on the way to drop off zone, 3. none
    bool debug;
    float distance_error;

};


int main(int argc, char **argv) {

    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;

    // 1. pick_object set pickup goal for robot
    // 2. pick_object subscribe topic odom and check the postion
    // 3. if robot arrives pick up zone, send a request to add_markers to hide the object
    // 4. after pick_object receive the response (hide action done), set drop off goal for robot
    // 5. wait and check position again
    // 6. if robot arrives drop off zone, send another request to add_markers to display the object
    // 7. receive the response and print "done"

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // important notes:
    // Regard the robot as finished the goal if either ac_ return succeed or pose from odom == goal.pose


    // todo: but still need to check the odom data, whether the robot is in the pick up zone or not
    // otherwise, add_marker and pick_object can not synchronize the action.
    // method 2: use class to add additional parameters for callback


    // The key point is to communicate with add_marker
    // However, I prefer service/client way
    // http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
    ros::ServiceClient client = n.serviceClient<add_markers::AddMarkers>("add_markers");

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

    Listener listener(&ac, &client, pickup_pose, dropoff_pose);
//    listener.setJob(pickup_pose, dropoff_pose);

    ros::Subscriber sub = n.subscribe("odom", 1000, &Listener::odomCallback, &listener);

    ros::spin();

    return 0;
}
