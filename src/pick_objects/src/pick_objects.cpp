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

void movebaseDoneCallback(void) {
    ROS_INFO("movebaseDoneCallback");
//        ROS_INFO("Finished in state [%s]", state.toString().c_str());
//        ROS_INFO("Answer: %i", result->result);
}

void movebaseActiveCallback(void) {
    ROS_INFO("movebaseActiveCallback");
//        ROS_INFO("Finished in state [%s]", state.toString().c_str());
//        ROS_INFO("Answer: %i", result->result);
}

void movebaseFeedbackCallback(void) {
    ROS_INFO("movebaseFeedbackCallback");
//        ROS_INFO("Finished in state [%s]", state.toString().c_str());
//        ROS_INFO("Answer: %i", result->result);
}


class Listener {
public:
    Listener(MoveBaseClient *ac, ros::ServiceClient *sc)
            : ac_(ac), sc_(sc), robot_state_(0), distance_error(0.5), debug(false) {
        // set the first goal for pick up zone:
        goal_pose_.position.x = -2.0;
        goal_pose_.position.y = 0;
        goal_pose_.position.z = 0;
        goal_pose_.orientation.x = 0;
        goal_pose_.orientation.y = 0;
        goal_pose_.orientation.z = 0;
        goal_pose_.orientation.w = 1.0;
    };

    void stop() {
        this->ac_->cancelAllGoals();
    }

//    void doneCallback(const actionlib::SimpleClientGoalState &state
//                      const move_base_msgs::MoveBaseActionResult &result) {
//        ROS_INFO("Finished in state [%s]", state.toString().c_str());
////        ROS_INFO("Answer: %i", result->result);
//    }

    void movebaseDoneCallback(void) {
        ROS_INFO("movebaseDoneCallback");
        if (robot_state_ == 1) {
            startJob();
        } else if (robot_state_ == 2) {
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

        // callback
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

        // set drop off goal
        this->goal_pose_.position.x = -3.0;
        this->goal_pose_.position.y = 1.5;
        move("heading to drop off", "drop off zone arrived");
        robot_state_++;
        ROS_INFO("state: %d", robot_state_);
    }

    void finishJob(void) {
        // pickup the object:tell add_markers node to show the object
        ROS_INFO("Dropping off... (about 5 sec)");
        if (this->operateObject("dropoff") == false) {
            // error? check or print debug?
            return;
        }

        ROS_INFO("Job done");

        robot_state_++;
        ROS_INFO("state: %d", this->robot_state_);
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//        ROS_INFO("state: %d", this->robot_state_);

        switch (robot_state_) {
            case 0: // beginning, set goal to pick up zone
//                memset(buff, 0, sizeof(buff));
//                sprintf(buff, "heading to pick up => x: %f, y: %f", this->goal_pose_.position.x, this->goal_pose_.position.y);
                move("heading to pick up", "pick up zone arrived");
                this->robot_state_++;
                ROS_INFO("state: %d", this->robot_state_);
                break;
            case 1: // on the way to pick up zone
                // debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_pose_.position.x) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_pose_.position.y) < this->distance_error) {
                    this->stop();  // whatever the goal has finished or not, stop all the actions,
                    // as according to the odom data we ready arrive the destination
                    startJob();

                }
                break;
            case 2: // on the way to drop off
                // todo: need to wait for display?? do this later
                // on the way to drop off
                // do nothing
//                debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_pose_.position.x) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_pose_.position.y) < this->distance_error) {
                    this->stop();

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
    int robot_state_;  // 0:init, 1:on the way to pick up zone, 2: on the way to drop off zone, 3. none
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


    // The key point is how to communicate with add_marker
    // However, I prefer service/client way
    // http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
    ros::ServiceClient client = n.serviceClient<add_markers::AddMarkers>("add_markers");
    Listener listener(&ac, &client);
    ros::Subscriber sub = n.subscribe("odom", 1000, &Listener::odomCallback, &listener);

    ros::spin();

    return 0;
}
